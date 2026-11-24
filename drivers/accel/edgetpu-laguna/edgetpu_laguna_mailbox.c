// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room KCI and Litebuf VII (IKV) hardware mailbox queues for Laguna EdgeTPU.
 *
 * Uses direct MMIO mailbox CSRs (top + 0xc000 / 0xe000) with GICv3 SPIs
 * mbox_0 (KCI) and mbox_1 (IKV, 16-credit Litebuf VII).
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/eventfd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "edgetpu_laguna.h"

#define LAGUNA_KCI_TIMEOUT_MS		2000

struct laguna_kci_waiter {
	struct list_head node;
	u64 seq;
	struct completion done;
	struct laguna_kci_resp_elem resp;
};

struct laguna_ikv_inflight {
	struct list_head node;
	u32 hw_seq;
	u32 client_seq;
	u32 pasid;
	struct laguna_iremap_mem large_cmd_mem;
};

struct laguna_ikv_ready_resp {
	struct list_head node;
	u32 seq;
	u16 code;
	u8 payload[LAGUNA_VII_RESP_PAYLOAD_BYTES];
	struct laguna_iremap_mem large_cmd_mem;
};

static inline u32 mbox_read32(struct laguna_mailbox *mbox, u32 off)
{
	return readl(mbox->csr_base + off);
}

static inline void mbox_write32(struct laguna_mailbox *mbox, u32 off, u32 val)
{
	writel(val, mbox->csr_base + off);
}

static int laguna_mbox_program_queues(struct laguna_mailbox *mbox,
				      dma_addr_t cmd_daddr, u32 cmd_size,
				      dma_addr_t rsp_daddr, u32 rsp_size)
{
	u32 cmd_hi = upper_32_bits(cmd_daddr);
	u32 rsp_hi = upper_32_bits(rsp_daddr);

	if (!cmd_size || cmd_size >= LAGUNA_QUEUE_WRAP_BIT ||
	    !rsp_size || rsp_size >= LAGUNA_QUEUE_WRAP_BIT)
		return -EINVAL;
	if ((cmd_hi & ~0xfU) || (rsp_hi & ~0xfU))
		return -EINVAL;

	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_DB_EN, 0);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_DB_EN, 0);
	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_DB_CLR, 1);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_DB_CLR, 1);

	mbox->cmd_queue_size = cmd_size;
	mbox->cmd_queue_tail = 0;
	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_ADDR_LO, lower_32_bits(cmd_daddr));
	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_ADDR_HI, cmd_hi);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_SIZE, cmd_size);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_TAIL, 0);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_HEAD, 0);

	mbox->resp_queue_size = rsp_size;
	mbox->resp_queue_head = 0;
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_ADDR_LO, lower_32_bits(rsp_daddr));
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_ADDR_HI, rsp_hi);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_SIZE, rsp_size);
	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_HEAD, 0);
	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_TAIL, 0);

	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_DB_EN, 1);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_DB_EN, 1);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_ENABLE, 1);
	return 0;
}

static irqreturn_t laguna_mbox_irq_handler(int irq, void *arg)
{
	struct laguna_mailbox *mbox = arg;

	if (!mbox_read32(mbox, LAGUNA_MBOX_RSPQ_DB_STATUS))
		return IRQ_NONE;

	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_DB_CLR, 1);
	if (mbox->handle_irq)
		mbox->handle_irq(mbox);

	return IRQ_HANDLED;
}

static struct laguna_mailbox *
laguna_mbox_create(struct edgetpu_laguna_dev *etdev, u32 index,
		   void (*handler)(struct laguna_mailbox *))
{
	struct laguna_mailbox *mbox;
	int ret;

	if (index >= ARRAY_SIZE(etdev->mbox_irqs))
		return ERR_PTR(-EINVAL);

	mbox = kzalloc(sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return ERR_PTR(-ENOMEM);

	mbox->etdev = etdev;
	mbox->index = index;
	mbox->irq = etdev->mbox_irqs[index];
	mbox->csr_base = etdev->top_regs + LAGUNA_MBOX_BASE +
			 index * LAGUNA_MBOX_STRIDE;
	mbox->handle_irq = handler;

	if (mbox->irq > 0) {
		ret = request_irq(mbox->irq, laguna_mbox_irq_handler, 0,
				  dev_name(etdev->dev), mbox);
		if (ret) {
			kfree(mbox);
			return ERR_PTR(ret);
		}
	}

	return mbox;
}

static void laguna_mbox_destroy(struct laguna_mailbox *mbox)
{
	if (!mbox)
		return;

	mbox_write32(mbox, LAGUNA_MBOX_CTX_CMD_DB_EN, 0);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_RSP_DB_EN, 0);
	mbox_write32(mbox, LAGUNA_MBOX_CTX_ENABLE, 0);
	if (mbox->irq > 0)
		free_irq(mbox->irq, mbox);
	kfree(mbox);
}

/*
 * KCI (Kernel Control Interface, mbox_0)
 */

static void laguna_rkci_work_fn(struct work_struct *work)
{
	struct laguna_kci *kci = container_of(work, struct laguna_kci, rkci_work);
	struct edgetpu_laguna_dev *etdev = kci->etdev;
	struct laguna_kci_resp_elem ev;
	unsigned long flags;
	bool has_ev;

	for (;;) {
		spin_lock_irqsave(&kci->rkci_lock, flags);
		has_ev = (kci->rkci_head != kci->rkci_tail);
		if (has_ev) {
			ev = kci->rkci_ring[kci->rkci_head];
			kci->rkci_head = (kci->rkci_head + 1) % ARRAY_SIZE(kci->rkci_ring);
		}
		spin_unlock_irqrestore(&kci->rkci_lock, flags);

		if (!has_ev)
			break;

		switch (ev.code) {
		case LAGUNA_RKCI_FIRMWARE_CRASH:
		case LAGUNA_RKCI_JOB_LOCKUP:
			dev_err(etdev->dev, "RKCI fatal firmware event code=%#x\n",
				ev.code);
			edgetpu_laguna_broadcast_fatal(etdev, BIT(0));
			break;
		case LAGUNA_RKCI_CLIENT_FATAL_ERROR: {
			u32 pasid = ev.rkci_value2 & 0xffffU;
			struct laguna_client *client = NULL;

			if (pasid < LAGUNA_NUM_PASIDS) {
				spin_lock_irqsave(&etdev->clients_lock, flags);
				client = etdev->pasid_clients[pasid];
				if (client)
					edgetpu_laguna_client_get(client);
				spin_unlock_irqrestore(&etdev->clients_lock, flags);
			}
			if (client) {
				edgetpu_laguna_client_notify_fatal(client, BIT(6));
				edgetpu_laguna_client_put(client);
			}
			break;
		}
		default:
			dev_dbg(etdev->dev, "RKCI unhandled code=%#x\n", ev.code);
			break;
		}
	}
}

static void laguna_kci_handle_irq(struct laguna_mailbox *mbox)
{
	struct laguna_kci *kci = &mbox->etdev->kci;
	struct laguna_kci_resp_elem *ring = kci->rspq_mem.vaddr;
	u32 size = mbox->resp_queue_size;
	u32 tail = mbox_read32(mbox, LAGUNA_MBOX_RSPQ_TAIL);
	unsigned long flags;
	bool queue_rkci = false;

	dma_rmb();

	spin_lock_irqsave(&kci->rspq_lock, flags);
	while (mbox->resp_queue_head != tail) {
		u32 idx = laguna_circ_real_idx(mbox->resp_queue_head);
		struct laguna_kci_resp_elem elem = ring[idx];

		mbox->resp_queue_head = laguna_circ_inc(mbox->resp_queue_head, 1, size);

		if (elem.seq & LAGUNA_KCI_REVERSE_FLAG) {
			spin_lock(&kci->rkci_lock);
			u32 next_tail = (kci->rkci_tail + 1) % ARRAY_SIZE(kci->rkci_ring);

			if (next_tail != kci->rkci_head) {
				kci->rkci_ring[kci->rkci_tail] = elem;
				kci->rkci_tail = next_tail;
				queue_rkci = true;
			}
			spin_unlock(&kci->rkci_lock);
		} else {
			struct laguna_kci_waiter *w;

			spin_lock(&kci->waiter_lock);
			list_for_each_entry(w, &kci->waiters, node) {
				if (w->seq == elem.seq) {
					w->resp = elem;
					complete(&w->done);
					break;
				}
			}
			spin_unlock(&kci->waiter_lock);
		}
	}
	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_HEAD, mbox->resp_queue_head);
	spin_unlock_irqrestore(&kci->rspq_lock, flags);

	if (queue_rkci && kci->rkci_wq)
		queue_work(kci->rkci_wq, &kci->rkci_work);
}

static int laguna_kci_send_elem(struct edgetpu_laguna_dev *etdev,
				struct laguna_kci_cmd_elem *cmd,
				struct laguna_kci_resp_elem *out_resp)
{
	struct laguna_kci *kci = &etdev->kci;
	struct laguna_mailbox *mbox = kci->mbox;
	struct laguna_kci_cmd_elem *ring = kci->cmdq_mem.vaddr;
	struct laguna_kci_waiter waiter;
	unsigned long flags;
	u32 head, tail, idx;
	long timeout;

	if (!mbox)
		return -ENODEV;

	INIT_LIST_HEAD(&waiter.node);
	init_completion(&waiter.done);

	mutex_lock(&kci->cmdq_lock);
	cmd->seq = (u64)atomic64_inc_return(&kci->seq) & ~LAGUNA_KCI_REVERSE_FLAG;
	waiter.seq = cmd->seq;

	head = mbox_read32(mbox, LAGUNA_MBOX_CMDQ_HEAD);
	tail = mbox->cmd_queue_tail;
	if (head == (tail ^ LAGUNA_QUEUE_WRAP_BIT)) {
		mutex_unlock(&kci->cmdq_lock);
		return -EBUSY;
	}

	spin_lock_irqsave(&kci->waiter_lock, flags);
	list_add_tail(&waiter.node, &kci->waiters);
	spin_unlock_irqrestore(&kci->waiter_lock, flags);

	idx = laguna_circ_real_idx(tail);
	ring[idx] = *cmd;
	dma_wmb();

	mbox->cmd_queue_tail = laguna_circ_inc(tail, 1, mbox->cmd_queue_size);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_TAIL, mbox->cmd_queue_tail);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_DB_SET, 1);
	mutex_unlock(&kci->cmdq_lock);

	timeout = wait_for_completion_timeout(&waiter.done,
					      msecs_to_jiffies(LAGUNA_KCI_TIMEOUT_MS));

	spin_lock_irqsave(&kci->waiter_lock, flags);
	list_del(&waiter.node);
	spin_unlock_irqrestore(&kci->waiter_lock, flags);

	if (!timeout) {
		dev_err(etdev->dev, "KCI cmd %u seq %llu timed out\n",
			cmd->code, cmd->seq);
		return -ETIMEDOUT;
	}

	if (out_resp)
		*out_resp = waiter.resp;

	return waiter.resp.code ? -EIO : 0;
}

static int laguna_kci_send_with_data(struct edgetpu_laguna_dev *etdev,
				     struct laguna_kci_cmd_elem *cmd,
				     const void *payload, size_t size)
{
	struct laguna_iremap_mem mem;
	int ret;

	ret = edgetpu_laguna_iremap_alloc(etdev, size, &mem);
	if (ret)
		return ret;

	memcpy(mem.vaddr, payload, size);
	dma_wmb();
	cmd->dma.address = mem.daddr;
	cmd->dma.size = size;

	ret = laguna_kci_send_elem(etdev, cmd, NULL);
	edgetpu_laguna_iremap_free(etdev, &mem);
	return ret;
}

int edgetpu_laguna_kci_init(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_kci *kci = &etdev->kci;
	size_t cmdq_bytes = LAGUNA_KCI_QUEUE_SIZE * sizeof(struct laguna_kci_cmd_elem);
	size_t rspq_bytes = LAGUNA_KCI_QUEUE_SIZE * sizeof(struct laguna_kci_resp_elem);
	int ret;

	kci->etdev = etdev;
	mutex_init(&kci->cmdq_lock);
	spin_lock_init(&kci->rspq_lock);
	spin_lock_init(&kci->waiter_lock);
	spin_lock_init(&kci->rkci_lock);
	INIT_LIST_HEAD(&kci->waiters);
	INIT_WORK(&kci->rkci_work, laguna_rkci_work_fn);
	atomic64_set(&kci->seq, 0);

	kci->rkci_wq = alloc_ordered_workqueue("edgetpu_rkci", 0);
	if (!kci->rkci_wq)
		return -ENOMEM;

	kci->mbox = laguna_mbox_create(etdev, LAGUNA_KCI_MBOX_INDEX,
				       laguna_kci_handle_irq);
	if (IS_ERR(kci->mbox)) {
		ret = PTR_ERR(kci->mbox);
		kci->mbox = NULL;
		goto err_wq;
	}

	ret = edgetpu_laguna_iremap_alloc(etdev, cmdq_bytes, &kci->cmdq_mem);
	if (ret)
		goto err_mbox;

	ret = edgetpu_laguna_iremap_alloc(etdev, rspq_bytes, &kci->rspq_mem);
	if (ret)
		goto err_cmdq;

	ret = edgetpu_laguna_kci_reinit(etdev);
	if (ret)
		goto err_rspq;

	return 0;

err_rspq:
	edgetpu_laguna_iremap_free(etdev, &kci->rspq_mem);
err_cmdq:
	edgetpu_laguna_iremap_free(etdev, &kci->cmdq_mem);
err_mbox:
	laguna_mbox_destroy(kci->mbox);
	kci->mbox = NULL;
err_wq:
	destroy_workqueue(kci->rkci_wq);
	kci->rkci_wq = NULL;
	return ret;
}

int edgetpu_laguna_kci_reinit(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_kci *kci = &etdev->kci;

	if (!kci->mbox)
		return -ENODEV;

	memset(kci->cmdq_mem.vaddr, 0, kci->cmdq_mem.size);
	memset(kci->rspq_mem.vaddr, 0, kci->rspq_mem.size);
	dma_wmb();

	mbox_write32(kci->mbox, LAGUNA_MBOX_CTX_SPARE_0,
		     LAGUNA_DRIVER_FW_IFACE_VERSION);
	return laguna_mbox_program_queues(kci->mbox,
					  kci->cmdq_mem.daddr, LAGUNA_KCI_QUEUE_SIZE,
					  kci->rspq_mem.daddr, LAGUNA_KCI_QUEUE_SIZE);
}

void edgetpu_laguna_kci_exit(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_kci *kci = &etdev->kci;

	if (kci->mbox) {
		laguna_mbox_destroy(kci->mbox);
		kci->mbox = NULL;
	}
	if (kci->rkci_wq) {
		cancel_work_sync(&kci->rkci_work);
		destroy_workqueue(kci->rkci_wq);
		kci->rkci_wq = NULL;
	}
	edgetpu_laguna_iremap_free(etdev, &kci->rspq_mem);
	edgetpu_laguna_iremap_free(etdev, &kci->cmdq_mem);
}

int edgetpu_laguna_kci_exchange_info(struct edgetpu_laguna_dev *etdev,
				     struct laguna_fw_info *info)
{
	struct laguna_kci_cmd_elem cmd = {
		.code = LAGUNA_KCI_EXCHANGE_INFO,
	};
	struct laguna_iremap_mem mem;
	int ret;

	memset(info, 0, sizeof(*info));
	ret = edgetpu_laguna_iremap_alloc(etdev, sizeof(*info), &mem);
	if (!ret) {
		memset(mem.vaddr, 0, sizeof(*info));
		dma_wmb();
		cmd.dma.address = mem.daddr;
		cmd.dma.size = sizeof(*info);
	}

	ret = laguna_kci_send_elem(etdev, &cmd, NULL);
	if (cmd.dma.address) {
		dma_rmb();
		memcpy(info, mem.vaddr, sizeof(*info));
		edgetpu_laguna_iremap_free(etdev, &mem);
	}
	return ret;
}

int edgetpu_laguna_kci_map_telemetry(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_kci_cmd_elem log_cmd = {
		.code = LAGUNA_KCI_MAP_LOG_BUFFER,
		.dma = {
			.address = etdev->shared_daddr,
			.size = etdev->log_size,
		},
	};
	struct laguna_kci_cmd_elem trace_cmd = {
		.code = LAGUNA_KCI_MAP_TRACE_BUFFER,
		.dma = {
			.address = etdev->shared_daddr +
				   etdev->log_size * etdev->num_telemetry,
			.size = etdev->trace_size,
		},
	};
	int ret;

	ret = laguna_kci_send_elem(etdev, &log_cmd, NULL);
	if (ret)
		return ret;

	return laguna_kci_send_elem(etdev, &trace_cmd, NULL);
}

int edgetpu_laguna_kci_open_device(struct edgetpu_laguna_dev *etdev, u32 mailbox_map,
				   u32 client_priv, s16 vcid, bool first_open)
{
	struct laguna_kci_open_device_detail detail = {
		.client_priv = client_priv,
		.vcid = vcid,
		.flags = (mailbox_map << 1) | (first_open ? 1U : 0U),
	};
	struct laguna_kci_cmd_elem cmd = {
		.code = LAGUNA_KCI_OPEN_DEVICE,
		.dma = { .flags = mailbox_map },
	};

	if (vcid < 0)
		return laguna_kci_send_elem(etdev, &cmd, NULL);

	return laguna_kci_send_with_data(etdev, &cmd, &detail, sizeof(detail));
}

int edgetpu_laguna_kci_allocate_vmbox(struct edgetpu_laguna_dev *etdev, u32 client_id,
				      u8 slice_index, bool first_open, bool first_party)
{
	struct laguna_kci_allocate_vmbox_detail detail = {
		.client_id = client_id,
		.slice_index = slice_index,
		.first_open = first_open ? 1 : 0,
		.first_party = first_party ? 1 : 0,
	};
	struct laguna_kci_cmd_elem cmd = {
		.code = LAGUNA_KCI_ALLOCATE_VMBOX,
	};

	return laguna_kci_send_with_data(etdev, &cmd, &detail, sizeof(detail));
}

int edgetpu_laguna_kci_release_vmbox(struct edgetpu_laguna_dev *etdev, u32 client_id)
{
	struct laguna_kci_release_vmbox_detail detail = {
		.client_id = client_id,
	};
	struct laguna_kci_cmd_elem cmd = {
		.code = LAGUNA_KCI_RELEASE_VMBOX,
	};

	return laguna_kci_send_with_data(etdev, &cmd, &detail, sizeof(detail));
}

int edgetpu_laguna_kci_shutdown(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_kci_cmd_elem cmd = {
		.code = LAGUNA_KCI_SHUTDOWN,
	};

	return laguna_kci_send_elem(etdev, &cmd, NULL);
}

/*
 * IKV (In-Kernel Virtual Inference Interface, Litebuf VII over mbox_1)
 */

static void laguna_ikv_handle_irq(struct laguna_mailbox *mbox)
{
	struct edgetpu_laguna_dev *etdev = mbox->etdev;
	struct laguna_ikv *ikv = &etdev->ikv;
	struct laguna_vii_litebuf_resp *ring = ikv->rspq_mem.vaddr;
	u32 size = mbox->resp_queue_size;
	u32 tail = mbox_read32(mbox, LAGUNA_MBOX_RSPQ_TAIL);
	unsigned long flags;

	dma_rmb();

	spin_lock_irqsave(&ikv->rspq_lock, flags);
	while (mbox->resp_queue_head != tail) {
		u32 idx = laguna_circ_real_idx(mbox->resp_queue_head);
		struct laguna_vii_litebuf_resp wire = ring[idx];
		u32 pasid = wire.client_id & 0xffffU;
		struct laguna_ikv_inflight *inf, *matched = NULL;
		struct laguna_ikv_ready_resp *rdy = NULL;
		struct laguna_client *client = NULL;

		mbox->resp_queue_head = laguna_circ_inc(mbox->resp_queue_head, 1, size);

		spin_lock(&ikv->inflight_lock);
		list_for_each_entry(inf, &ikv->inflight, node) {
			if (inf->hw_seq == wire.seq && inf->pasid == pasid) {
				list_del(&inf->node);
				matched = inf;
				break;
			}
		}
		spin_unlock(&ikv->inflight_lock);

		if (pasid < LAGUNA_NUM_PASIDS) {
			spin_lock(&etdev->clients_lock);
			client = etdev->pasid_clients[pasid];
			if (client)
				edgetpu_laguna_client_get(client);
			spin_unlock(&etdev->clients_lock);
		}

		if (matched && client) {
			rdy = kzalloc(sizeof(*rdy), GFP_ATOMIC);
			if (rdy) {
				rdy->seq = matched->client_seq;
				rdy->code = wire.code;
				memcpy(rdy->payload, wire.runtime_response,
				       LAGUNA_VII_RESP_PAYLOAD_BYTES);
				rdy->large_cmd_mem = matched->large_cmd_mem;
				matched->large_cmd_mem.vaddr = NULL;

				spin_lock(&client->resp_lock);
				list_add_tail(&rdy->node, &client->resp_ready);
				if (client->inflight_count)
					client->inflight_count--;
				spin_unlock(&client->resp_lock);

				if (client->resp_eventfd)
					eventfd_signal(client->resp_eventfd);
			}
		}

		if (matched && matched->large_cmd_mem.vaddr)
			edgetpu_laguna_iremap_free(etdev, &matched->large_cmd_mem);
		kfree(matched);
		if (client)
			edgetpu_laguna_client_put(client);
	}
	mbox_write32(mbox, LAGUNA_MBOX_RSPQ_HEAD, mbox->resp_queue_head);
	spin_unlock_irqrestore(&ikv->rspq_lock, flags);
}

int edgetpu_laguna_ikv_init(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_ikv *ikv = &etdev->ikv;
	size_t cmdq_bytes = LAGUNA_IKV_QUEUE_SIZE * sizeof(struct laguna_vii_litebuf_cmd);
	size_t rspq_bytes = LAGUNA_IKV_QUEUE_SIZE * sizeof(struct laguna_vii_litebuf_resp);
	int ret;

	ikv->etdev = etdev;
	mutex_init(&ikv->cmdq_lock);
	spin_lock_init(&ikv->rspq_lock);
	spin_lock_init(&ikv->inflight_lock);
	INIT_LIST_HEAD(&ikv->inflight);
	atomic_set(&ikv->seq, 0);

	ikv->mbox = laguna_mbox_create(etdev, LAGUNA_IKV_MBOX_INDEX,
				       laguna_ikv_handle_irq);
	if (IS_ERR(ikv->mbox)) {
		ret = PTR_ERR(ikv->mbox);
		ikv->mbox = NULL;
		return ret;
	}

	ret = edgetpu_laguna_iremap_alloc(etdev, cmdq_bytes, &ikv->cmdq_mem);
	if (ret)
		goto err_mbox;

	ret = edgetpu_laguna_iremap_alloc(etdev, rspq_bytes, &ikv->rspq_mem);
	if (ret)
		goto err_cmdq;

	ret = edgetpu_laguna_ikv_reinit(etdev);
	if (ret)
		goto err_rspq;

	return 0;

err_rspq:
	edgetpu_laguna_iremap_free(etdev, &ikv->rspq_mem);
err_cmdq:
	edgetpu_laguna_iremap_free(etdev, &ikv->cmdq_mem);
err_mbox:
	laguna_mbox_destroy(ikv->mbox);
	ikv->mbox = NULL;
	return ret;
}

int edgetpu_laguna_ikv_reinit(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_ikv *ikv = &etdev->ikv;

	if (!ikv->mbox)
		return -ENODEV;

	memset(ikv->cmdq_mem.vaddr, 0, ikv->cmdq_mem.size);
	memset(ikv->rspq_mem.vaddr, 0, ikv->rspq_mem.size);
	dma_wmb();

	return laguna_mbox_program_queues(ikv->mbox,
					  ikv->cmdq_mem.daddr, LAGUNA_IKV_QUEUE_SIZE,
					  ikv->rspq_mem.daddr, LAGUNA_IKV_QUEUE_SIZE);
}

void edgetpu_laguna_ikv_exit(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_ikv *ikv = &etdev->ikv;
	struct laguna_ikv_inflight *inf, *tmp;
	LIST_HEAD(leftover);
	unsigned long flags;

	if (ikv->mbox) {
		laguna_mbox_destroy(ikv->mbox);
		ikv->mbox = NULL;
	}

	spin_lock_irqsave(&ikv->inflight_lock, flags);
	list_splice_init(&ikv->inflight, &leftover);
	spin_unlock_irqrestore(&ikv->inflight_lock, flags);

	list_for_each_entry_safe(inf, tmp, &leftover, node) {
		list_del(&inf->node);
		edgetpu_laguna_iremap_free(etdev, &inf->large_cmd_mem);
		kfree(inf);
	}

	edgetpu_laguna_iremap_free(etdev, &ikv->rspq_mem);
	edgetpu_laguna_iremap_free(etdev, &ikv->cmdq_mem);
}

int edgetpu_laguna_ikv_submit(struct laguna_client *client,
			      const struct edgetpu_vii_litebuf_command_ioctl *req)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_ikv *ikv = &etdev->ikv;
	struct laguna_mailbox *mbox = ikv->mbox;
	struct laguna_vii_litebuf_cmd wire = { };
	struct laguna_vii_litebuf_cmd *ring;
	struct laguna_ikv_inflight *inf;
	unsigned long flags;
	u32 head, tail, idx;
	int ret;

	BUILD_BUG_ON(sizeof(struct laguna_vii_litebuf_cmd) != 128);
	BUILD_BUG_ON(sizeof(struct laguna_vii_litebuf_resp) != 64);

	if (!mbox || !client->etdom || !client->wakelock_held)
		return -EINVAL;
	if (req->seq >> 32)
		return -EINVAL;
	if (req->in_fence_count || req->out_fence_count)
		return -EOPNOTSUPP;
	if (req->litebuf_size && !req->litebuf_address)
		return -EINVAL;

	spin_lock_irqsave(&client->resp_lock, flags);
	if (client->inflight_count >= LAGUNA_NUM_VII_CREDITS) {
		spin_unlock_irqrestore(&client->resp_lock, flags);
		return -EBUSY;
	}
	client->inflight_count++;
	spin_unlock_irqrestore(&client->resp_lock, flags);

	inf = kzalloc(sizeof(*inf), GFP_KERNEL);
	if (!inf) {
		ret = -ENOMEM;
		goto err_dec_credit;
	}

	inf->pasid = client->etdom->pasid;
	inf->client_seq = (u32)req->seq;
	inf->hw_seq = (u32)atomic_inc_return(&ikv->seq);

	wire.seq = inf->hw_seq;
	wire.client_id = inf->pasid;

	if (!req->litebuf_size) {
		wire.type = LAGUNA_VII_CMD_NULL;
	} else if (req->litebuf_size <= LAGUNA_VII_CMD_PAYLOAD_BYTES) {
		wire.type = LAGUNA_VII_CMD_RUNTIME;
		if (copy_from_user(wire.runtime_command,
				   u64_to_user_ptr(req->litebuf_address),
				   req->litebuf_size)) {
			ret = -EFAULT;
			goto err_free_inf;
		}
	} else {
		ret = edgetpu_laguna_iremap_alloc(etdev, req->litebuf_size,
						  &inf->large_cmd_mem);
		if (ret)
			goto err_free_inf;

		if (copy_from_user(inf->large_cmd_mem.vaddr,
				   u64_to_user_ptr(req->litebuf_address),
				   req->litebuf_size)) {
			ret = -EFAULT;
			goto err_free_large;
		}
		dma_wmb();
		wire.type = LAGUNA_VII_CMD_LARGE_RUNTIME;
		wire.large_runtime_command.address = (u32)inf->large_cmd_mem.daddr;
		wire.large_runtime_command.size_bytes = req->litebuf_size;
	}

	mutex_lock(&ikv->cmdq_lock);
	head = mbox_read32(mbox, LAGUNA_MBOX_CMDQ_HEAD);
	tail = mbox->cmd_queue_tail;
	if (head == (tail ^ LAGUNA_QUEUE_WRAP_BIT)) {
		mutex_unlock(&ikv->cmdq_lock);
		ret = -EBUSY;
		goto err_free_large;
	}

	spin_lock_irqsave(&ikv->inflight_lock, flags);
	list_add_tail(&inf->node, &ikv->inflight);
	spin_unlock_irqrestore(&ikv->inflight_lock, flags);

	ring = ikv->cmdq_mem.vaddr;
	idx = laguna_circ_real_idx(tail);
	ring[idx] = wire;
	dma_wmb();

	mbox->cmd_queue_tail = laguna_circ_inc(tail, 1, mbox->cmd_queue_size);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_TAIL, mbox->cmd_queue_tail);
	mbox_write32(mbox, LAGUNA_MBOX_CMDQ_DB_SET, 1);
	mutex_unlock(&ikv->cmdq_lock);

	return 0;

err_free_large:
	edgetpu_laguna_iremap_free(etdev, &inf->large_cmd_mem);
err_free_inf:
	kfree(inf);
err_dec_credit:
	spin_lock_irqsave(&client->resp_lock, flags);
	if (client->inflight_count)
		client->inflight_count--;
	spin_unlock_irqrestore(&client->resp_lock, flags);
	return ret;
}

int edgetpu_laguna_ikv_pop_resp(struct laguna_client *client,
				struct edgetpu_vii_litebuf_response_ioctl *req)
{
	struct laguna_ikv_ready_resp *rdy = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&client->resp_lock, flags);
	if (!list_empty(&client->resp_ready)) {
		rdy = list_first_entry(&client->resp_ready,
				       struct laguna_ikv_ready_resp, node);
		list_del(&rdy->node);
	}
	spin_unlock_irqrestore(&client->resp_lock, flags);

	if (!rdy)
		return -ENOENT;

	edgetpu_laguna_iremap_free(client->etdev, &rdy->large_cmd_mem);

	if (req->litebuf_address &&
	    copy_to_user(u64_to_user_ptr(req->litebuf_address),
			 rdy->payload, LAGUNA_VII_RESP_PAYLOAD_BYTES)) {
		ret = -EFAULT;
		goto out_free;
	}

	req->seq = rdy->seq;
	req->code = rdy->code;
out_free:
	kfree(rdy);
	return ret;
}

void edgetpu_laguna_ikv_cancel_client(struct laguna_client *client)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_ikv *ikv = &etdev->ikv;
	struct laguna_ikv_inflight *inf, *inf_tmp;
	struct laguna_ikv_ready_resp *rdy, *rdy_tmp;
	LIST_HEAD(drop_inf);
	LIST_HEAD(drop_rdy);
	unsigned long flags;

	if (client->etdom) {
		u32 pasid = client->etdom->pasid;

		spin_lock_irqsave(&ikv->inflight_lock, flags);
		list_for_each_entry_safe(inf, inf_tmp, &ikv->inflight, node) {
			if (inf->pasid == pasid) {
				list_del(&inf->node);
				list_add_tail(&inf->node, &drop_inf);
			}
		}
		spin_unlock_irqrestore(&ikv->inflight_lock, flags);

		list_for_each_entry_safe(inf, inf_tmp, &drop_inf, node) {
			list_del(&inf->node);
			edgetpu_laguna_iremap_free(etdev, &inf->large_cmd_mem);
			kfree(inf);
		}
	}

	spin_lock_irqsave(&client->resp_lock, flags);
	list_splice_init(&client->resp_ready, &drop_rdy);
	client->inflight_count = 0;
	spin_unlock_irqrestore(&client->resp_lock, flags);

	list_for_each_entry_safe(rdy, rdy_tmp, &drop_rdy, node) {
		list_del(&rdy->node);
		edgetpu_laguna_iremap_free(etdev, &rdy->large_cmd_mem);
		kfree(rdy);
	}
}
