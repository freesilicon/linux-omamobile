// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room BlueZ HCI transport driver over Google Tensor G5 (Laguna)
 * Always-on Compute (AoC) channelized IPC ("com.google.bt").
 *
 * Bridges standard Linux Bluetooth HCI H4 frames onto Laguna AoC's
 * multiplexed channel IPC service exposed by google,lga-aoc, registering
 * a standard BlueZ HCI_PRIMARY controller (hciX).
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/soc/google/lga-aoc.h>
#include <linux/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btbcm.h"

#define LGA_AOCC_MAX_MSG_SIZE		1024U
#define LGA_AOCC_HDR_SIZE		sizeof(__le32)
#define LGA_AOCC_CHAN_MASK		GENMASK(30, 0)
#define LGA_AOCC_MAX_CHAN_ID		((1U << 30) - 1U)

#define LGA_AOCC_TX_TIMEOUT_MS		1000U
#define LGA_AOCC_RX_POLL_MS		250U

enum lga_aocc_cmd_code {
	LGA_AOCC_CMD_OPEN_CHANNEL	= 0,
	LGA_AOCC_CMD_CLOSE_CHANNEL	= 1,
	LGA_AOCC_CMD_BLOCK_CHANNEL	= 2,
	LGA_AOCC_CMD_UNBLOCK_CHANNEL	= 3,
	LGA_AOCC_CMD_SUSPEND_PREPARE	= 4,
	LGA_AOCC_CMD_WAKEUP_COMPLETE	= 5,
};

struct lga_aocc_ctrl_msg {
	__le32 channel_index;
	__le32 command_code;
	__le32 channel_to_modify;
} __packed;

struct hci_lga_aoc {
	struct device *aoc_dev;
	struct hci_dev *hdev;
	struct lga_aoc_service_info svc;

	struct task_struct *rx_task;
	struct mutex tx_lock; /* serializes TX frames on the AoC channel */
	struct sk_buff *rx_skb;
	size_t rx_hdr_len;
	size_t rx_expect_len;

	struct gpio_desc *bt_shutdown_gpio;
	struct gpio_desc *bt_ble_dbo_gpio;

	u32 channel_id;
	size_t max_msg_len;
	size_t max_payload;
	u8 *tx_buf;
};

static char *service_name = "com.google.bt";
module_param(service_name, charp, 0444);
MODULE_PARM_DESC(service_name, "AoC IPC service name (default: com.google.bt)");

static bool aocc_framing = true;
module_param(aocc_framing, bool, 0644);
MODULE_PARM_DESC(aocc_framing,
		 "Use 4-byte AoC channel mux header and open/close control messages");

static bool toggle_bt_power = true;
module_param(toggle_bt_power, bool, 0644);
MODULE_PARM_DESC(toggle_bt_power,
		 "Assert optional bt-ble-dbo/bt-shutdown GPIOs on hci open/close");

static bool run_btbcm_setup;
module_param(run_btbcm_setup, bool, 0644);
MODULE_PARM_DESC(run_btbcm_setup,
		 "Run btbcm_setup_patchram() during HCI setup (default: false)");

static bool debug_rx;
module_param(debug_rx, bool, 0644);
MODULE_PARM_DESC(debug_rx, "Log incoming com.google.bt payload bytes");

static DEFINE_MUTEX(hci_lga_aoc_lock);
static struct hci_lga_aoc *hci_lga_aoc_inst;
static atomic_t hci_lga_aoc_chan_seq = ATOMIC_INIT(1);

static int hci_lga_aoc_send_ctrl(struct hci_lga_aoc *bt,
				 enum lga_aocc_cmd_code cmd, u32 chan_id)
{
	struct lga_aocc_ctrl_msg msg = {
		.channel_index = cpu_to_le32(0),
		.command_code = cpu_to_le32(cmd),
		.channel_to_modify = cpu_to_le32(chan_id),
	};
	ssize_t ret;

	mutex_lock(&bt->tx_lock);
	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->svc.index,
					&msg, sizeof(msg),
					msecs_to_jiffies(LGA_AOCC_TX_TIMEOUT_MS));
	mutex_unlock(&bt->tx_lock);

	return ret < 0 ? (int)ret : 0;
}

static int hci_lga_aoc_send_chunk(struct hci_lga_aoc *bt, const u8 *data,
				  size_t len)
{
	size_t frame_len;
	ssize_t ret;

	if (len > bt->max_payload)
		return -EMSGSIZE;

	mutex_lock(&bt->tx_lock);
	if (aocc_framing) {
		__le32 hdr = cpu_to_le32(bt->channel_id & LGA_AOCC_CHAN_MASK);

		memcpy(bt->tx_buf, &hdr, LGA_AOCC_HDR_SIZE);
		memcpy(bt->tx_buf + LGA_AOCC_HDR_SIZE, data, len);
		frame_len = LGA_AOCC_HDR_SIZE + len;
	} else {
		memcpy(bt->tx_buf, data, len);
		frame_len = len;
	}

	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->svc.index,
					bt->tx_buf, frame_len,
					msecs_to_jiffies(LGA_AOCC_TX_TIMEOUT_MS));
	mutex_unlock(&bt->tx_lock);

	return ret < 0 ? (int)ret : 0;
}

static void hci_lga_aoc_rx_reset(struct hci_lga_aoc *bt)
{
	kfree_skb(bt->rx_skb);
	bt->rx_skb = NULL;
	bt->rx_hdr_len = 0;
	bt->rx_expect_len = 0;
}

static size_t hci_lga_aoc_pkt_hdr_len(u8 pkt_type)
{
	switch (pkt_type) {
	case HCI_EVENT_PKT:
		return HCI_EVENT_HDR_SIZE;
	case HCI_ACLDATA_PKT:
		return HCI_ACL_HDR_SIZE;
	case HCI_SCODATA_PKT:
		return HCI_SCO_HDR_SIZE;
	case HCI_ISODATA_PKT:
		return HCI_ISO_HDR_SIZE;
	default:
		return 0;
	}
}

static size_t hci_lga_aoc_pkt_payload_len(u8 pkt_type, const u8 *hdr)
{
	switch (pkt_type) {
	case HCI_EVENT_PKT:
		return hdr[1];
	case HCI_ACLDATA_PKT:
		return get_unaligned_le16(hdr + 2);
	case HCI_SCODATA_PKT:
		return hdr[2];
	case HCI_ISODATA_PKT:
		return get_unaligned_le16(hdr + 2) & 0x3fffU;
	default:
		return 0;
	}
}

static void hci_lga_aoc_rx_bytes(struct hci_lga_aoc *bt, const u8 *data,
				 size_t count)
{
	struct hci_dev *hdev = bt->hdev;

	hdev->stat.byte_rx += count;

	while (count > 0) {
		size_t need, take;

		if (!bt->rx_skb) {
			u8 pkt_type = *data++;
			size_t hdr_len = hci_lga_aoc_pkt_hdr_len(pkt_type);

			count--;
			if (!hdr_len) {
				bt_dev_warn_ratelimited(hdev,
							"unknown H4 packet type %#02x",
							pkt_type);
				hdev->stat.err_rx++;
				continue;
			}

			bt->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE,
						  GFP_KERNEL);
			if (!bt->rx_skb) {
				hdev->stat.err_rx++;
				return;
			}

			hci_skb_pkt_type(bt->rx_skb) = pkt_type;
			bt->rx_hdr_len = hdr_len;
			bt->rx_expect_len = hdr_len;
		}

		need = bt->rx_expect_len - bt->rx_skb->len;
		take = min(count, need);
		if (bt->rx_skb->len + take > HCI_MAX_FRAME_SIZE) {
			bt_dev_err(hdev, "oversized H4 frame (%zu bytes)",
				   bt->rx_skb->len + take);
			hdev->stat.err_rx++;
			hci_lga_aoc_rx_reset(bt);
			return;
		}

		skb_put_data(bt->rx_skb, data, take);
		data += take;
		count -= take;

		if (bt->rx_skb->len == bt->rx_hdr_len &&
		    bt->rx_expect_len == bt->rx_hdr_len) {
			u8 pkt_type = hci_skb_pkt_type(bt->rx_skb);
			size_t plen;

			plen = hci_lga_aoc_pkt_payload_len(pkt_type,
							   bt->rx_skb->data);

			bt->rx_expect_len = bt->rx_hdr_len + plen;
			if (bt->rx_expect_len > HCI_MAX_FRAME_SIZE) {
				bt_dev_err(hdev, "invalid H4 payload length %zu",
					   plen);
				hdev->stat.err_rx++;
				hci_lga_aoc_rx_reset(bt);
				continue;
			}
		}

		if (bt->rx_skb->len == bt->rx_expect_len) {
			struct sk_buff *skb = bt->rx_skb;

			bt->rx_skb = NULL;
			bt->rx_hdr_len = 0;
			bt->rx_expect_len = 0;
			hci_recv_frame(hdev, skb);
		}
	}
}

static int hci_lga_aoc_rx_thread(void *arg)
{
	struct hci_lga_aoc *bt = arg;
	u8 *rx_buf;

	rx_buf = kmalloc(bt->max_msg_len, GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	while (!kthread_should_stop()) {
		ssize_t n;
		const u8 *payload;
		size_t plen;

		n = lga_aoc_ipc_read_timeout(bt->aoc_dev, bt->svc.index,
					     rx_buf, bt->max_msg_len,
					     msecs_to_jiffies(LGA_AOCC_RX_POLL_MS));
		if (n == -ETIMEDOUT || n == -ERESTARTSYS || n == -EAGAIN)
			continue;

		if (n < 0) {
			if (n == -ENODEV) {
				set_current_state(TASK_INTERRUPTIBLE);
				if (!kthread_should_stop())
					schedule();
				set_current_state(TASK_RUNNING);
				continue;
			}
			bt_dev_err_ratelimited(bt->hdev,
					       "AoC IPC read failed: %zd", n);
			msleep(20);
			continue;
		}

		if (aocc_framing) {
			u32 hdr, rx_chan;

			if ((size_t)n < LGA_AOCC_HDR_SIZE) {
				bt_dev_warn_ratelimited(bt->hdev,
							"short AoC channel frame (%zd B)",
							n);
				continue;
			}
			hdr = get_unaligned_le32(rx_buf);
			rx_chan = hdr & LGA_AOCC_CHAN_MASK;
			if (rx_chan != bt->channel_id) {
				bt_dev_dbg(bt->hdev,
					   "closing stale AoC BT channel %u",
					   rx_chan);
				hci_lga_aoc_send_ctrl(bt,
						      LGA_AOCC_CMD_CLOSE_CHANNEL,
						      rx_chan);
				continue;
			}
			payload = rx_buf + LGA_AOCC_HDR_SIZE;
			plen = (size_t)n - LGA_AOCC_HDR_SIZE;
		} else {
			payload = rx_buf;
			plen = (size_t)n;
		}

		if (!plen)
			continue;

		if (debug_rx)
			dev_info(bt->aoc_dev, "hci_lga_aoc rx %zu B: %*ph\n",
				 plen, (int)min_t(size_t, plen, 32U), payload);

		hci_lga_aoc_rx_bytes(bt, payload, plen);
	}

	hci_lga_aoc_rx_reset(bt);
	kfree(rx_buf);
	return 0;
}

static void hci_lga_aoc_power_on(struct hci_lga_aoc *bt)
{
	struct gpio_desc *gp;

	if (!toggle_bt_power)
		return;

	gp = gpiod_get_optional(bt->aoc_dev, "bt-ble-dbo", GPIOD_OUT_LOW);
	if (IS_ERR(gp)) {
		dev_info(bt->aoc_dev,
			 "bt-ble-dbo GPIO unavailable (%ld), skipping AP power toggle\n",
			 PTR_ERR(gp));
	} else {
		bt->bt_ble_dbo_gpio = gp;
	}

	gp = gpiod_get_optional(bt->aoc_dev, "bt-shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(gp)) {
		dev_info(bt->aoc_dev,
			 "bt-shutdown GPIO unavailable (%ld), skipping AP power toggle\n",
			 PTR_ERR(gp));
	} else {
		bt->bt_shutdown_gpio = gp;
	}

	if (bt->bt_ble_dbo_gpio)
		gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 1);

	if (bt->bt_shutdown_gpio) {
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 0);
		msleep(50);
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 1);
		msleep(100);
	}
}

static void hci_lga_aoc_power_off(struct hci_lga_aoc *bt)
{
	if (bt->bt_shutdown_gpio) {
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 0);
		gpiod_put(bt->bt_shutdown_gpio);
		bt->bt_shutdown_gpio = NULL;
	}
	if (bt->bt_ble_dbo_gpio) {
		gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 0);
		gpiod_put(bt->bt_ble_dbo_gpio);
		bt->bt_ble_dbo_gpio = NULL;
	}
}

static int hci_lga_aoc_open(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);
	u32 chan;
	int ret;

	if (!lga_aoc_is_online(bt->aoc_dev))
		return -ENODEV;

	chan = (u32)atomic_inc_return(&hci_lga_aoc_chan_seq);
	if (chan == 0 || chan >= LGA_AOCC_MAX_CHAN_ID) {
		atomic_set(&hci_lga_aoc_chan_seq, 1);
		chan = 1;
	}
	bt->channel_id = chan;

	hci_lga_aoc_power_on(bt);

	bt->rx_task = kthread_run(hci_lga_aoc_rx_thread, bt, "hci-lga-aoc");
	if (IS_ERR(bt->rx_task)) {
		ret = PTR_ERR(bt->rx_task);
		bt->rx_task = NULL;
		hci_lga_aoc_power_off(bt);
		return ret;
	}

	if (aocc_framing) {
		ret = hci_lga_aoc_send_ctrl(bt, LGA_AOCC_CMD_OPEN_CHANNEL,
					    bt->channel_id);
		if (ret) {
			bt_dev_err(hdev,
				   "failed to open AoC channel %u: %d",
				   bt->channel_id, ret);
			kthread_stop(bt->rx_task);
			bt->rx_task = NULL;
			hci_lga_aoc_power_off(bt);
			return ret;
		}
	}

	bt_dev_info(hdev, "opened AoC service '%s' (idx=%u chan=%u framing=%u)",
		    bt->svc.name, bt->svc.index, bt->channel_id, aocc_framing);
	return 0;
}

static int hci_lga_aoc_close(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);

	if (aocc_framing && bt->channel_id && lga_aoc_is_online(bt->aoc_dev))
		hci_lga_aoc_send_ctrl(bt, LGA_AOCC_CMD_CLOSE_CHANNEL,
				      bt->channel_id);

	if (bt->rx_task) {
		kthread_stop(bt->rx_task);
		bt->rx_task = NULL;
	}

	hci_lga_aoc_rx_reset(bt);
	hci_lga_aoc_power_off(bt);
	bt->channel_id = 0;
	return 0;
}

static int hci_lga_aoc_flush(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);

	hci_lga_aoc_rx_reset(bt);
	return 0;
}

static int hci_lga_aoc_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);
	u8 pkt_type = hci_skb_pkt_type(skb);
	const u8 *src;
	size_t remaining, sent = 0;
	int ret;

	if (!lga_aoc_is_online(bt->aoc_dev)) {
		hdev->stat.err_tx++;
		return -ENODEV;
	}

	switch (pkt_type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	case HCI_ISODATA_PKT:
		break;
	default:
		hdev->stat.err_tx++;
		return -EILSEQ;
	}

	/* Prepend H4 packet indicator byte before chunking onto AoC IPC */
	memcpy(skb_push(skb, 1), &pkt_type, 1);
	src = skb->data;
	remaining = skb->len;

	while (remaining > 0) {
		size_t chunk = min(remaining, bt->max_payload);

		ret = hci_lga_aoc_send_chunk(bt, src + sent, chunk);
		if (ret) {
			hdev->stat.err_tx++;
			skb_pull(skb, 1);
			return ret;
		}
		sent += chunk;
		remaining -= chunk;
	}

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);
	return 0;
}

static int hci_lga_aoc_setup(struct hci_dev *hdev)
{
	struct sk_buff *skb;
	int ret;

	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "HCI Reset over AoC failed: %ld",
			   PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	kfree_skb(skb);

	if (run_btbcm_setup) {
		ret = btbcm_setup_patchram(hdev);
		if (ret && ret != -ENOENT)
			return ret;
	}

	return 0;
}

static int hci_lga_aoc_attach_locked(void)
{
	struct device *aoc_dev;
	struct lga_aoc_service_info svc;
	struct hci_lga_aoc *bt;
	struct hci_dev *hdev;
	size_t max_msg, hdr_overhead;
	int ret;

	if (hci_lga_aoc_inst)
		return 0;

	aoc_dev = lga_aoc_get_device();
	if (!aoc_dev)
		return -ENODEV;

	if (!lga_aoc_is_online(aoc_dev)) {
		put_device(aoc_dev);
		return -EAGAIN;
	}

	ret = lga_aoc_find_service(aoc_dev, service_name, &svc);
	if (ret) {
		dev_err(aoc_dev, "AoC service '%s' not found: %d\n",
			service_name, ret);
		put_device(aoc_dev);
		return ret;
	}

	max_msg = LGA_AOCC_MAX_MSG_SIZE;
	if (svc.type == LGA_AOC_SVC_QUEUE) {
		size_t down_msg = svc.down_size > 8U ? svc.down_size - 8U : 0;
		size_t up_msg = svc.up_size > 8U ? svc.up_size - 8U : 0;

		if (down_msg)
			max_msg = min(max_msg, down_msg);
		if (up_msg)
			max_msg = min(max_msg, up_msg);
	}

	hdr_overhead = aocc_framing ? LGA_AOCC_HDR_SIZE : 0;
	if (max_msg <= hdr_overhead + 1U) {
		dev_err(aoc_dev, "AoC service '%s' message size %zu too small\n",
			svc.name, max_msg);
		put_device(aoc_dev);
		return -EINVAL;
	}

	bt = kzalloc(sizeof(*bt), GFP_KERNEL);
	if (!bt) {
		put_device(aoc_dev);
		return -ENOMEM;
	}

	bt->tx_buf = kmalloc(max_msg, GFP_KERNEL);
	if (!bt->tx_buf) {
		kfree(bt);
		put_device(aoc_dev);
		return -ENOMEM;
	}

	hdev = hci_alloc_dev();
	if (!hdev) {
		kfree(bt->tx_buf);
		kfree(bt);
		put_device(aoc_dev);
		return -ENOMEM;
	}

	mutex_init(&bt->tx_lock);
	bt->aoc_dev = aoc_dev;
	bt->hdev = hdev;
	bt->svc = svc;
	bt->max_msg_len = max_msg;
	bt->max_payload = max_msg - hdr_overhead;

	hci_set_drvdata(hdev, bt);
	SET_HCIDEV_DEV(hdev, aoc_dev);
	hdev->bus = HCI_SMD;
	hdev->open = hci_lga_aoc_open;
	hdev->close = hci_lga_aoc_close;
	hdev->flush = hci_lga_aoc_flush;
	hdev->send = hci_lga_aoc_send;
	hdev->setup = hci_lga_aoc_setup;
	hdev->set_bdaddr = btbcm_set_bdaddr;
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	hci_set_quirk(hdev, HCI_QUIRK_USE_BDADDR_PROPERTY);

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		dev_err(aoc_dev, "failed to register BlueZ HCI dev: %d\n", ret);
		hci_free_dev(hdev);
		kfree(bt->tx_buf);
		kfree(bt);
		put_device(aoc_dev);
		return ret;
	}

	hci_lga_aoc_inst = bt;
	dev_info(aoc_dev,
		 "registered %s over AoC service '%s' (idx=%u mbox=%u max_payload=%zu)\n",
		 hdev->name, svc.name, svc.index, svc.mbox_index,
		 bt->max_payload);
	return 0;
}

static void hci_lga_aoc_detach_locked(void)
{
	struct hci_lga_aoc *bt = hci_lga_aoc_inst;

	if (!bt)
		return;

	hci_lga_aoc_inst = NULL;
	hci_unregister_dev(bt->hdev);
	hci_free_dev(bt->hdev);
	kfree(bt->tx_buf);
	put_device(bt->aoc_dev);
	kfree(bt);
}

static int hci_lga_aoc_notify(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	mutex_lock(&hci_lga_aoc_lock);
	if (event == LGA_AOC_EVENT_ONLINE)
		hci_lga_aoc_attach_locked();
	else if (event == LGA_AOC_EVENT_OFFLINE)
		hci_lga_aoc_detach_locked();
	mutex_unlock(&hci_lga_aoc_lock);
	return NOTIFY_DONE;
}

static struct notifier_block hci_lga_aoc_nb = {
	.notifier_call = hci_lga_aoc_notify,
};

static int __init hci_lga_aoc_init(void)
{
	int ret;

	ret = lga_aoc_register_notifier(&hci_lga_aoc_nb);
	if (ret)
		return ret;

	mutex_lock(&hci_lga_aoc_lock);
	ret = hci_lga_aoc_attach_locked();
	mutex_unlock(&hci_lga_aoc_lock);

	if (ret == -ENODEV || ret == -EAGAIN)
		pr_info("hci_lga_aoc: waiting for Laguna AoC online ('%s')\n",
			service_name);

	return 0;
}

static void __exit hci_lga_aoc_exit(void)
{
	lga_aoc_unregister_notifier(&hci_lga_aoc_nb);

	mutex_lock(&hci_lga_aoc_lock);
	hci_lga_aoc_detach_locked();
	mutex_unlock(&hci_lga_aoc_lock);
}

module_init(hci_lga_aoc_init);
module_exit(hci_lga_aoc_exit);

MODULE_DESCRIPTION("Google Tensor G5 (Laguna) AoC Bluetooth HCI transport");
MODULE_LICENSE("GPL");
