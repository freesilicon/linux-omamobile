// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room character device (/dev/edgetpu-soc) and 0xED ioctl ABI 1.4
 * implementation for Google Tensor G5 (Laguna) EdgeTPU.
 */

#include <linux/bitmap.h>
#include <linux/compat.h>
#include <linux/err.h>
#include <linux/eventfd.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "edgetpu_laguna.h"

static void laguna_client_release_kref(struct kref *kref)
{
	struct laguna_client *client = container_of(kref, struct laguna_client, ref);

	kfree(client);
}

void edgetpu_laguna_client_get(struct laguna_client *client)
{
	kref_get(&client->ref);
}

void edgetpu_laguna_client_put(struct laguna_client *client)
{
	kref_put(&client->ref, laguna_client_release_kref);
}

void edgetpu_laguna_client_notify_fatal(struct laguna_client *client, u32 err_mask)
{
	unsigned long flags;

	spin_lock_irqsave(&client->resp_lock, flags);
	client->fatal_errors |= err_mask;
	spin_unlock_irqrestore(&client->resp_lock, flags);

	if (client->fatal_eventfd)
		eventfd_signal(client->fatal_eventfd);
}

void edgetpu_laguna_broadcast_fatal(struct edgetpu_laguna_dev *etdev, u32 err_mask)
{
	struct laguna_client *snapshot[LAGUNA_NUM_PASIDS] = { };
	unsigned long flags;
	u32 i;

	spin_lock_irqsave(&etdev->clients_lock, flags);
	for (i = 0; i < LAGUNA_NUM_PASIDS; i++) {
		if (etdev->pasid_clients[i]) {
			snapshot[i] = etdev->pasid_clients[i];
			edgetpu_laguna_client_get(snapshot[i]);
		}
	}
	spin_unlock_irqrestore(&etdev->clients_lock, flags);

	for (i = 0; i < LAGUNA_NUM_PASIDS; i++) {
		if (snapshot[i]) {
			edgetpu_laguna_client_notify_fatal(snapshot[i], err_mask);
			edgetpu_laguna_client_put(snapshot[i]);
		}
	}
}

static int laguna_alloc_vcid(struct edgetpu_laguna_dev *etdev, s16 *vcid_out)
{
	int vcid;

	mutex_lock(&etdev->mmu_lock);
	vcid = find_first_zero_bit(etdev->vcid_bitmap, LAGUNA_NUM_VCIDS);
	if (vcid >= LAGUNA_NUM_VCIDS) {
		mutex_unlock(&etdev->mmu_lock);
		return -EBUSY;
	}
	set_bit(vcid, etdev->vcid_bitmap);
	mutex_unlock(&etdev->mmu_lock);

	*vcid_out = (s16)vcid;
	return 0;
}

static void laguna_free_vcid(struct edgetpu_laguna_dev *etdev, s16 vcid)
{
	if (vcid < 0 || vcid >= LAGUNA_NUM_VCIDS)
		return;

	mutex_lock(&etdev->mmu_lock);
	clear_bit(vcid, etdev->vcid_bitmap);
	mutex_unlock(&etdev->mmu_lock);
}

static int laguna_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct edgetpu_laguna_dev *etdev = container_of(mdev, struct edgetpu_laguna_dev, mdev);
	struct laguna_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	kref_init(&client->ref);
	client->etdev = etdev;
	client->vcid = -1;
	client->first_open = true;
	mutex_init(&client->lock);
	mutex_init(&client->map_lock);
	spin_lock_init(&client->resp_lock);
	INIT_LIST_HEAD(&client->mappings);
	INIT_LIST_HEAD(&client->resp_ready);

	file->private_data = client;
	return 0;
}

static int laguna_release_wakelock_locked(struct laguna_client *client)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;

	if (!client->wakelock_held)
		return 0;

	if (client->vmbox_allocated && client->etdom) {
		edgetpu_laguna_kci_release_vmbox(etdev, client->etdom->pasid);
		client->vmbox_allocated = false;
	}
	client->wakelock_held = false;
	edgetpu_laguna_pm_put(etdev);
	return 0;
}

static int laguna_release(struct inode *inode, struct file *file)
{
	struct laguna_client *client = file->private_data;
	struct edgetpu_laguna_dev *etdev = client->etdev;
	unsigned long flags;

	mutex_lock(&client->lock);
	laguna_release_wakelock_locked(client);
	edgetpu_laguna_ikv_cancel_client(client);
	edgetpu_laguna_unmap_all_client(client);

	if (client->etdom) {
		u32 pasid = client->etdom->pasid;

		spin_lock_irqsave(&etdev->clients_lock, flags);
		if (pasid < LAGUNA_NUM_PASIDS && etdev->pasid_clients[pasid] == client)
			etdev->pasid_clients[pasid] = NULL;
		spin_unlock_irqrestore(&etdev->clients_lock, flags);

		edgetpu_laguna_domain_free(etdev, client->etdom);
		client->etdom = NULL;
	}

	if (client->vcid >= 0) {
		laguna_free_vcid(etdev, client->vcid);
		client->vcid = -1;
	}
	if (client->resp_eventfd) {
		eventfd_ctx_put(client->resp_eventfd);
		client->resp_eventfd = NULL;
	}
	if (client->fatal_eventfd) {
		eventfd_ctx_put(client->fatal_eventfd);
		client->fatal_eventfd = NULL;
	}
	mutex_unlock(&client->lock);

	edgetpu_laguna_client_put(client);
	return 0;
}

static int laguna_ioctl_create_group(struct laguna_client *client,
				     const struct edgetpu_mailbox_attr *attr)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_domain *etdom;
	unsigned long flags;
	s16 vcid;
	int ret;

	if (client->group_created)
		return -EINVAL;

	ret = laguna_alloc_vcid(etdev, &vcid);
	if (ret)
		return ret;

	etdom = edgetpu_laguna_domain_alloc(etdev);
	if (IS_ERR(etdom)) {
		laguna_free_vcid(etdev, vcid);
		return PTR_ERR(etdom);
	}

	spin_lock_irqsave(&etdev->clients_lock, flags);
	etdev->pasid_clients[etdom->pasid] = client;
	spin_unlock_irqrestore(&etdev->clients_lock, flags);

	client->etdom = etdom;
	client->vcid = vcid;
	client->client_priv = !!attr->client_priv;
	client->group_created = true;
	client->first_open = true;
	return 0;
}

static int laguna_ioctl_acquire_wakelock(struct laguna_client *client)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	int ret;

	if (!client->group_created || !client->etdom)
		return -EINVAL;
	if (client->wakelock_held)
		return 0;

	ret = edgetpu_laguna_pm_get(etdev);
	if (ret)
		return ret;

	ret = edgetpu_laguna_kci_allocate_vmbox(etdev, client->etdom->pasid,
						(u8)client->vcid,
						client->first_open,
						client->client_priv);
	if (ret) {
		edgetpu_laguna_pm_put(etdev);
		return ret;
	}

	client->first_open = false;
	client->vmbox_allocated = true;
	client->wakelock_held = true;
	return 0;
}

static int laguna_ioctl_set_eventfd(struct laguna_client *client,
				    const struct edgetpu_event_register *ev)
{
	struct eventfd_ctx *ctx;
	struct eventfd_ctx **slot;

	if (ev->event_id == EDGETPU_EVENT_RESPDATA)
		slot = &client->resp_eventfd;
	else if (ev->event_id == EDGETPU_EVENT_FATAL_ERROR)
		slot = &client->fatal_eventfd;
	else
		return -EINVAL;

	ctx = eventfd_ctx_fdget(ev->eventfd);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	if (*slot)
		eventfd_ctx_put(*slot);
	*slot = ctx;
	return 0;
}

static int laguna_ioctl_unset_event(struct laguna_client *client, u32 event_id)
{
	if (event_id == EDGETPU_EVENT_RESPDATA) {
		if (client->resp_eventfd) {
			eventfd_ctx_put(client->resp_eventfd);
			client->resp_eventfd = NULL;
		}
		return 0;
	}
	if (event_id == EDGETPU_EVENT_FATAL_ERROR) {
		if (client->fatal_eventfd) {
			eventfd_ctx_put(client->fatal_eventfd);
			client->fatal_eventfd = NULL;
		}
		return 0;
	}
	return -EINVAL;
}

static long laguna_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct laguna_client *client = file->private_data;
	struct edgetpu_laguna_dev *etdev = client->etdev;
	void __user *argp = (void __user *)arg;
	long ret = 0;

	BUILD_BUG_ON(sizeof(struct edgetpu_map_ioctl) != 32);
	BUILD_BUG_ON(sizeof(struct edgetpu_event_register) != 8);
	BUILD_BUG_ON(sizeof(struct edgetpu_mailbox_attr) != 20);
	BUILD_BUG_ON(sizeof(struct edgetpu_sync_ioctl) != 32);
	BUILD_BUG_ON(sizeof(struct edgetpu_map_dmabuf_ioctl) != 40);
	BUILD_BUG_ON(sizeof(struct edgetpu_fw_version) != 16);
	BUILD_BUG_ON(sizeof(struct edgetpu_vii_litebuf_command_ioctl) != 56);
	BUILD_BUG_ON(sizeof(struct edgetpu_vii_litebuf_response_ioctl) != 24);
	BUILD_BUG_ON(sizeof(struct edgetpu_interface_version_ioctl) != 68);

	mutex_lock(&client->lock);

	switch (cmd) {
	case EDGETPU_CREATE_GROUP: {
		struct edgetpu_mailbox_attr attr;

		if (copy_from_user(&attr, argp, sizeof(attr))) {
			ret = -EFAULT;
			break;
		}
		ret = laguna_ioctl_create_group(client, &attr);
		break;
	}
	case EDGETPU_FINALIZE_GROUP:
		ret = client->group_created ? 0 : -EINVAL;
		break;
	case EDGETPU_SET_EVENTFD: {
		struct edgetpu_event_register ev;

		if (copy_from_user(&ev, argp, sizeof(ev))) {
			ret = -EFAULT;
			break;
		}
		ret = laguna_ioctl_set_eventfd(client, &ev);
		break;
	}
	case EDGETPU_UNSET_EVENT:
		ret = laguna_ioctl_unset_event(client, (u32)arg);
		break;
	case EDGETPU_ACQUIRE_WAKE_LOCK:
	case EDGETPU_ACQUIRE_WAKE_LOCK_FLAGS:
		ret = laguna_ioctl_acquire_wakelock(client);
		break;
	case EDGETPU_RELEASE_WAKE_LOCK:
		ret = laguna_release_wakelock_locked(client);
		break;
	case EDGETPU_GET_CREDIT_SIZE: {
		__u64 credits = LAGUNA_NUM_VII_CREDITS;

		if (copy_to_user(argp, &credits, sizeof(credits)))
			ret = -EFAULT;
		break;
	}
	case EDGETPU_FIRMWARE_VERSION: {
		struct edgetpu_fw_version ver = {
			.major_version = etdev->fw_versions[0],
			.minor_version = etdev->fw_versions[1],
			.vii_version = etdev->fw_versions[2],
			.kci_version = etdev->fw_versions[3],
		};

		if (copy_to_user(argp, &ver, sizeof(ver)))
			ret = -EFAULT;
		break;
	}
	case EDGETPU_GET_INTERFACE_VERSION: {
		struct edgetpu_interface_version_ioctl iface = {
			.version_major = LAGUNA_IFACE_VER_MAJOR,
			.version_minor = LAGUNA_IFACE_VER_MINOR,
		};

		strscpy(iface.version_build, "laguna-mainline",
			sizeof(iface.version_build));
		if (copy_to_user(argp, &iface, sizeof(iface)))
			ret = -EFAULT;
		break;
	}
	case EDGETPU_MAP_DMABUF: {
		struct edgetpu_map_dmabuf_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_map_dmabuf(client, &req);
		if (!ret && copy_to_user(argp, &req, sizeof(req))) {
			edgetpu_laguna_unmap_dmabuf(client, &req);
			ret = -EFAULT;
		}
		break;
	}
	case EDGETPU_UNMAP_DMABUF: {
		struct edgetpu_map_dmabuf_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_unmap_dmabuf(client, &req);
		break;
	}
	case EDGETPU_MAP_BUFFER: {
		struct edgetpu_map_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_map_user(client, &req);
		if (!ret && copy_to_user(argp, &req, sizeof(req))) {
			edgetpu_laguna_unmap_user(client, &req);
			ret = -EFAULT;
		}
		break;
	}
	case EDGETPU_UNMAP_BUFFER: {
		struct edgetpu_map_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_unmap_user(client, &req);
		break;
	}
	case EDGETPU_SYNC_BUFFER: {
		struct edgetpu_sync_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_sync_buffer(client, &req);
		break;
	}
	case EDGETPU_VII_LITEBUF_COMMAND: {
		struct edgetpu_vii_litebuf_command_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_ikv_submit(client, &req);
		break;
	}
	case EDGETPU_VII_LITEBUF_RESPONSE: {
		struct edgetpu_vii_litebuf_response_ioctl req;

		if (copy_from_user(&req, argp, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		ret = edgetpu_laguna_ikv_pop_resp(client, &req);
		if (!ret && copy_to_user(argp, &req, sizeof(req)))
			ret = -EFAULT;
		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&client->lock);
	return ret;
}

static const struct file_operations laguna_fops = {
	.owner = THIS_MODULE,
	.open = laguna_open,
	.release = laguna_release,
	.unlocked_ioctl = laguna_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

int edgetpu_laguna_fs_register(struct edgetpu_laguna_dev *etdev)
{
	etdev->mdev.minor = MISC_DYNAMIC_MINOR;
	etdev->mdev.name = "edgetpu-soc";
	etdev->mdev.fops = &laguna_fops;
	etdev->mdev.parent = etdev->dev;

	return misc_register(&etdev->mdev);
}

void edgetpu_laguna_fs_unregister(struct edgetpu_laguna_dev *etdev)
{
	misc_deregister(&etdev->mdev);
}
