// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room BlueZ HCI transport driver over Google Tensor G5 (Laguna)
 * Always-on Compute (AoC) IPC.
 *
 * Supports both:
 *  - Laguna's EFW AP offload transport over the split AoC services
 *    "chre_bt_offload_ctl" (queue), "chre_bt_offload_data_tx" (TX ring),
 *    and "chre_bt_offload_data_rx" (RX ring), including the control-queue
 *    CONNECT/CREDIT/DISCONNECT handshake, FlatBuffer hardware/power/UART
 *    bring-up messages, and 32-byte-aligned H4 framing; and
 *  - Legacy "com.google.bt" 4-byte channel-muxed AOCC framing (or raw H4).
 */

#include <linux/align.h>
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
#include <linux/string.h>
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

/* Laguna EFW AP transport constants (chre_bt_offload_{ctl,data_tx,data_rx}) */
#define LGA_EFW_CTL_SERVICE_DEFAULT	"chre_bt_offload_ctl"
#define LGA_EFW_TX_SERVICE_DEFAULT	"chre_bt_offload_data_tx"
#define LGA_EFW_RX_SERVICE_DEFAULT	"chre_bt_offload_data_rx"

#define LGA_EFW_CTL_MSG_SIZE		56U
#define LGA_EFW_PROTO_VER		2U
#define LGA_EFW_AP_HDR_SIZE		17U
#define LGA_EFW_AOC_HDR_SIZE_DEFAULT	30U
#define LGA_EFW_AOC_HDR_SIZE_MIN	30U
#define LGA_EFW_AOC_HDR_SIZE_MAX	64U
#define LGA_EFW_RING_ALIGN		32U
#define LGA_EFW_MAX_FRAME_SIZE		4096U
#define LGA_EFW_RX_STREAM_SIZE		8192U
#define LGA_EFW_CTL_TIMEOUT_MS		2000U
#define LGA_EFW_RX_POLL_MS		100U
#define LGA_EFW_INIT_BAUD		115200U
#define LGA_EFW_OPER_BAUD_DEFAULT	3000000U

#define LGA_BCM_OP_UPDATE_BAUDRATE	0xfc18
#define LGA_BCM_OP_WRITE_UART_CLOCK	0xfc45

#define LGA_EFW_FLAG_NORMAL		0U
#define LGA_EFW_FLAG_EXPECT_REPLY	1U

#define LGA_EFW_PKT_KIND_NORMAL		0U
#define LGA_EFW_PKT_KIND_REPLY		1U
#define LGA_EFW_PKT_KIND_ECHO		2U
#define LGA_EFW_PKT_KIND_STATS		3U

enum lga_efw_ctl_type {
	LGA_EFW_CTL_DISCONNECT		= 0,
	LGA_EFW_CTL_DISCONNECT_ACK	= 1,
	LGA_EFW_CTL_CONNECT		= 2,
	LGA_EFW_CTL_CONNECT_ACK		= 3,
	LGA_EFW_CTL_CREDIT		= 4,
	LGA_EFW_CTL_ERROR		= 5,
	LGA_EFW_CTL_SYNC		= 6,
	LGA_EFW_CTL_SYNC_ACK		= 7,
};

enum lga_efw_msg_type {
	LGA_EFW_MSG_HCI_TX_OR_EVT	= 8,
	LGA_EFW_MSG_HCI_ACL_RX		= 9,
	LGA_EFW_MSG_HCI_SCO_RX		= 10,
	LGA_EFW_MSG_HCI_ISO_RX		= 11,
	LGA_EFW_MSG_HCI_DIAG_RX		= 14,
	LGA_EFW_MSG_FLATBUFFER		= 39,
};

enum lga_efw_fb_union_type {
	LGA_EFW_FB_ENABLE_HW		= 2,
	LGA_EFW_FB_SET_POWER		= 3,
	LGA_EFW_FB_SET_LPM		= 4,
	LGA_EFW_FB_CONFIG_UART		= 5,
	LGA_EFW_FB_DUMP_DEBUG		= 6,
	LGA_EFW_FB_QUERY_STATE		= 7,
};

enum hci_lga_aoc_mode {
	HCI_LGA_AOC_MODE_RAW		= 0,
	HCI_LGA_AOC_MODE_AOCC		= 1,
	HCI_LGA_AOC_MODE_EFW		= 2,
};

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
	struct lga_aoc_service_info tx_svc;
	struct lga_aoc_service_info rx_svc;
	struct lga_aoc_service_info ctl_svc;
	bool has_ctl_svc;
	enum hci_lga_aoc_mode mode;

	struct task_struct *rx_task;
	struct mutex tx_lock; /* serializes TX frames on the AoC channel */
	struct mutex ctl_lock; /* serializes control-queue writes */
	struct mutex rx_lock; /* protects rx_skb reassembly against flush() */
	struct sk_buff *rx_skb;
	size_t rx_hdr_len;
	size_t rx_expect_len;

	struct gpio_desc *bt_shutdown_gpio;
	struct gpio_desc *bt_ble_dbo_gpio;
	struct gpio_desc *bt_dev_wake_gpio;
	struct gpio_desc *bt_wlan_reg_on_gpio;
	bool wlan_reg_on_owned;

	u32 channel_id;
	u32 efw_tx_seq;
	u32 efw_aoc_hdr_size;
	u32 efw_tx_ring_size;
	u32 efw_rx_ring_size;
	u32 efw_cur_baud;
	bool efw_connected;

	size_t tx_max_msg;
	size_t rx_max_msg;
	size_t max_payload;
	u8 *tx_buf;
};

static char *service_name = "com.google.bt";
module_param(service_name, charp, 0444);
MODULE_PARM_DESC(service_name,
		 "AoC IPC TX/bidirectional service name (auto-falls back to chre_bt_offload_data_tx)");

static char *rx_service_name = "";
module_param(rx_service_name, charp, 0444);
MODULE_PARM_DESC(rx_service_name,
		 "Optional AoC IPC RX service name for split TX/RX rings");

static char *ctl_service_name = "";
module_param(ctl_service_name, charp, 0444);
MODULE_PARM_DESC(ctl_service_name,
		 "Optional AoC IPC control queue service name (default: chre_bt_offload_ctl in EFW mode)");

static bool aocc_framing = true;
module_param(aocc_framing, bool, 0444);
MODULE_PARM_DESC(aocc_framing,
		 "Use 4-byte AoC channel mux framing when not using EFW transport");

static bool efw_transport = true;
module_param(efw_transport, bool, 0444);
MODULE_PARM_DESC(efw_transport,
		 "Enable EFW AP transport handshake on chre_bt_offload_{ctl,data_tx,data_rx}");

static bool efw_pmic_ctrl = true;
module_param(efw_pmic_ctrl, bool, 0644);
MODULE_PARM_DESC(efw_pmic_ctrl,
		 "Request AoC-side PMIC/CPM BT power sequence in FlatBuffer SetPower");

static bool efw_power_cycle = true;
module_param(efw_power_cycle, bool, 0644);
MODULE_PARM_DESC(efw_power_cycle,
		 "Issue SetPower(0) before SetPower(1) when efw_pmic_ctrl=1 for a clean VBR_ON reset edge");

static int efw_tx_flag = -1;
module_param(efw_tx_flag, int, 0644);
MODULE_PARM_DESC(efw_tx_flag,
		 "Leading control byte for EFW msg_type=8 TX (-1=auto: 1 for HCI cmd, 0 for data; 0/1=force)");

static int efw_lpm_timeout = -1;
module_param(efw_lpm_timeout, int, 0644);
MODULE_PARM_DESC(efw_lpm_timeout,
		 "Optional EFW SetLpm (-1=keep AoC default, 0=disable LPM, 1..255=enable LPM timeout sec)");

static unsigned int efw_uart_baud = LGA_EFW_OPER_BAUD_DEFAULT;
module_param(efw_uart_baud, uint, 0644);
MODULE_PARM_DESC(efw_uart_baud,
		 "Operational UART baud rate negotiated via Broadcom 0xfc18 + EFW ConfigUART (default: 3000000; 0 or 115200 keeps init baud)");

static int efw_uart_stop_bits = -1;
module_param(efw_uart_stop_bits, int, 0644);
MODULE_PARM_DESC(efw_uart_stop_bits,
		 "Optional EFW ConfigUART stop_bits field (-1=keep AoC default 115200 8N1, 1=1 stop bit, 2=2 stop bits)");

static bool efw_cts_probe;
module_param(efw_cts_probe, bool, 0644);
MODULE_PARM_DESC(efw_cts_probe,
		 "On HCI Reset timeout, send >64B HCI_Reset burst to trigger AoC UART TX mismatch / MSR.CTS check (default: false)");

static bool toggle_bt_power = true;
module_param(toggle_bt_power, bool, 0644);
MODULE_PARM_DESC(toggle_bt_power,
		 "Assert optional bt-ble-dbo/bt-shutdown GPIOs on hci open/close");

static bool assert_wlan_reg_on = true;
module_param(assert_wlan_reg_on, bool, 0644);
MODULE_PARM_DESC(assert_wlan_reg_on,
		 "Assert optional bt-wlan-reg-on (AP2WLAN_REG_ON_1V2) nonexclusively before BT power-on (default: true)");

static bool run_btbcm_setup = true;
module_param(run_btbcm_setup, bool, 0644);
MODULE_PARM_DESC(run_btbcm_setup,
		 "Run Broadcom btbcm patchram and finalize sequence during HCI setup (default: true)");

static bool debug_rx;
module_param(debug_rx, bool, 0644);
MODULE_PARM_DESC(debug_rx, "Log incoming AoC BT control and payload frames");

static DEFINE_MUTEX(hci_lga_aoc_lock);
static struct hci_lga_aoc *hci_lga_aoc_inst;
static atomic_t hci_lga_aoc_chan_seq = ATOMIC_INIT(1);

static const char *hci_lga_aoc_mode_str(enum hci_lga_aoc_mode mode)
{
	switch (mode) {
	case HCI_LGA_AOC_MODE_EFW:
		return "efw";
	case HCI_LGA_AOC_MODE_AOCC:
		return "aocc";
	default:
		return "raw";
	}
}

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
	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->tx_svc.index,
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
	if (bt->mode == HCI_LGA_AOC_MODE_AOCC) {
		__le32 hdr = cpu_to_le32(bt->channel_id & LGA_AOCC_CHAN_MASK);

		memcpy(bt->tx_buf, &hdr, LGA_AOCC_HDR_SIZE);
		memcpy(bt->tx_buf + LGA_AOCC_HDR_SIZE, data, len);
		frame_len = LGA_AOCC_HDR_SIZE + len;
	} else {
		memcpy(bt->tx_buf, data, len);
		frame_len = len;
	}

	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->tx_svc.index,
					bt->tx_buf, frame_len,
					msecs_to_jiffies(LGA_AOCC_TX_TIMEOUT_MS));
	mutex_unlock(&bt->tx_lock);

	return ret < 0 ? (int)ret : 0;
}

/* ---- Laguna EFW AP transport helpers ---- */

static int hci_lga_aoc_efw_ctl_write(struct hci_lga_aoc *bt,
				     const u8 msg[LGA_EFW_CTL_MSG_SIZE])
{
	ssize_t ret;

	mutex_lock(&bt->ctl_lock);
	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->ctl_svc.index,
					msg, LGA_EFW_CTL_MSG_SIZE,
					msecs_to_jiffies(LGA_EFW_CTL_TIMEOUT_MS));
	mutex_unlock(&bt->ctl_lock);

	return ret < 0 ? (int)ret : 0;
}

static int hci_lga_aoc_efw_send_credit(struct hci_lga_aoc *bt, u32 delta_bytes)
{
	u8 msg[LGA_EFW_CTL_MSG_SIZE] = {};

	if (!delta_bytes)
		return 0;

	msg[0] = LGA_EFW_CTL_CREDIT;
	put_unaligned_le32(delta_bytes, msg + 1);
	msg[5] = 0; /* primary RX ring */

	return hci_lga_aoc_efw_ctl_write(bt, msg);
}

static int hci_lga_aoc_efw_send_disconnect(struct hci_lga_aoc *bt)
{
	u8 msg[LGA_EFW_CTL_MSG_SIZE] = {};

	msg[0] = LGA_EFW_CTL_DISCONNECT;
	return hci_lga_aoc_efw_ctl_write(bt, msg);
}

static void hci_lga_aoc_efw_drain_ctl(struct hci_lga_aoc *bt)
{
	u8 rep[LGA_EFW_CTL_MSG_SIZE];
	ssize_t n;

	for (;;) {
		n = lga_aoc_ipc_read(bt->aoc_dev, bt->ctl_svc.index,
				     rep, sizeof(rep));
		if (n <= 0)
			break;

		if (rep[0] == LGA_EFW_CTL_ERROR)
			bt_dev_warn(bt->hdev,
				    "EFW ctl ERROR code=%u (%zd B)",
				    n > 1 ? rep[1] : 0, n);
		else if (debug_rx)
			dev_info(bt->aoc_dev,
				 "EFW ctl rx type=%u len=%zd: %*ph\n",
				 rep[0], n, (int)min_t(ssize_t, n, 24), rep);
	}
}

static int hci_lga_aoc_efw_wait_ctl(struct hci_lga_aoc *bt, u8 want_type,
				    u8 rep[LGA_EFW_CTL_MSG_SIZE],
				    unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);

	for (;;) {
		long rem = deadline - jiffies;
		ssize_t n;

		if (rem <= 0)
			return -ETIMEDOUT;

		n = lga_aoc_ipc_read_timeout(bt->aoc_dev, bt->ctl_svc.index,
					     rep, LGA_EFW_CTL_MSG_SIZE, rem);
		if (n == -ETIMEDOUT)
			return -ETIMEDOUT;
		if (n == -ERESTARTSYS || n == -EAGAIN)
			continue;
		if (n < 0)
			return (int)n;
		if (n == 0)
			continue;

		if (debug_rx)
			dev_info(bt->aoc_dev,
				 "EFW ctl rx type=%u len=%zd: %*ph\n",
				 rep[0], n, (int)min_t(ssize_t, n, 24), rep);

		if (rep[0] == want_type)
			return (int)n;
		if (rep[0] == LGA_EFW_CTL_ERROR) {
			bt_dev_warn(bt->hdev,
				    "EFW ctl returned ERROR code=%u while waiting for %u",
				    n > 1 ? rep[1] : 0, want_type);
			return -EPROTO;
		}
	}
}

static int hci_lga_aoc_efw_connect(struct hci_lga_aoc *bt)
{
	u8 req[LGA_EFW_CTL_MSG_SIZE] = {};
	u8 rep[LGA_EFW_CTL_MSG_SIZE] = {};
	u8 stale[256];
	int attempt, ret;

	/* Drain any stale messages left in the control queue or RX ring */
	hci_lga_aoc_efw_drain_ctl(bt);
	while (lga_aoc_ipc_read(bt->aoc_dev, bt->rx_svc.index,
				stale, sizeof(stale)) > 0)
		;

	req[0] = LGA_EFW_CTL_CONNECT;
	put_unaligned_le32(LGA_EFW_AP_HDR_SIZE, req + 1);
	req[5] = 1; /* required protocol mode byte */
	req[6] = LGA_EFW_PROTO_VER;
	req[7] = 0; /* single RX ring (primary chre_bt_offload_data_rx) */

	for (attempt = 0; attempt < 2; attempt++) {
		ret = hci_lga_aoc_efw_ctl_write(bt, req);
		if (ret)
			return ret;

		ret = hci_lga_aoc_efw_wait_ctl(bt, LGA_EFW_CTL_CONNECT_ACK,
					       rep, LGA_EFW_CTL_TIMEOUT_MS);
		if (ret >= 15) {
			u32 tx_sz = get_unaligned_le32(rep + 1);
			u32 rx_sz = get_unaligned_le32(rep + 5);
			u32 aoc_hdr = get_unaligned_le32(rep + 9);

			if (aoc_hdr < LGA_EFW_AOC_HDR_SIZE_MIN ||
			    aoc_hdr > LGA_EFW_AOC_HDR_SIZE_MAX)
				aoc_hdr = LGA_EFW_AOC_HDR_SIZE_DEFAULT;

			bt->efw_tx_ring_size = tx_sz;
			bt->efw_rx_ring_size = rx_sz;
			bt->efw_aoc_hdr_size = aoc_hdr;
			bt->efw_tx_seq = 0;
			bt->efw_connected = true;

			bt_dev_info(bt->hdev,
				    "EFW CONNECT_ACK: tx_ring=%u rx_ring=%u aoc_hdr=%u mode=%u ver=%u",
				    tx_sz, rx_sz, aoc_hdr, rep[13], rep[14]);
			return 0;
		}

		if (ret == -EPROTO && attempt == 0) {
			bt_dev_info(bt->hdev,
				    "EFW state reset via DISCONNECT before retrying CONNECT");
			hci_lga_aoc_efw_send_disconnect(bt);
			hci_lga_aoc_efw_wait_ctl(bt, LGA_EFW_CTL_DISCONNECT_ACK,
						 rep, 1000U);
			continue;
		}

		break;
	}

	return ret < 0 ? ret : -EIO;
}

static int hci_lga_aoc_efw_send_frame(struct hci_lga_aoc *bt, u32 msg_type,
				      u8 flags, const u8 *prefix,
				      size_t prefix_len, const u8 *body,
				      size_t body_len)
{
	size_t payload_len = sizeof(__le32) + prefix_len + body_len;
	size_t raw_len = LGA_EFW_AP_HDR_SIZE + payload_len;
	size_t frame_len = ALIGN(raw_len, LGA_EFW_RING_ALIGN);
	ssize_t ret;
	u8 *p;

	if (frame_len > bt->tx_max_msg)
		return -EMSGSIZE;

	mutex_lock(&bt->tx_lock);
	p = bt->tx_buf;
	put_unaligned_le64(0, p);
	put_unaligned_le32(payload_len, p + 8);
	put_unaligned_le32(bt->efw_tx_seq, p + 12);
	p[16] = flags;
	put_unaligned_le32(msg_type, p + LGA_EFW_AP_HDR_SIZE);

	p += LGA_EFW_AP_HDR_SIZE + sizeof(__le32);
	if (prefix_len) {
		memcpy(p, prefix, prefix_len);
		p += prefix_len;
	}
	if (body_len) {
		memcpy(p, body, body_len);
		p += body_len;
	}
	if (frame_len > raw_len)
		memset(p, 0, frame_len - raw_len);

	ret = lga_aoc_ipc_write_timeout(bt->aoc_dev, bt->tx_svc.index,
					bt->tx_buf, frame_len,
					msecs_to_jiffies(LGA_AOCC_TX_TIMEOUT_MS));
	if (ret >= 0)
		bt->efw_tx_seq++;
	mutex_unlock(&bt->tx_lock);

	return ret < 0 ? (int)ret : 0;
}

/*
 * Canonical 40-byte FlatBuffer for union tables with two u8/bool fields at
 * vtable offsets +4 and +6 (EnableHWInterface=2, SetPower=3, SetLpm=4).
 */
static int hci_lga_aoc_efw_send_fb_u8_2(struct hci_lga_aoc *bt, u8 union_type,
					u8 field4, u8 field6)
{
	u8 fb[40] = {};

	put_unaligned_le32(12, fb + 0);		/* root table offset */
	/* Root vtable at offset 4 */
	put_unaligned_le16(8, fb + 4);		/* vtable size */
	put_unaligned_le16(12, fb + 6);		/* table size */
	put_unaligned_le16(4, fb + 8);		/* union_type field offset */
	put_unaligned_le16(8, fb + 10);		/* union_table field offset */
	/* Root table at offset 12 */
	put_unaligned_le32(8, fb + 12);		/* soffset back to vtable (12-8=4) */
	fb[16] = union_type;
	put_unaligned_le32(12, fb + 20);	/* relative offset to subtable (20+12=32) */
	/* Subtable vtable at offset 24 */
	put_unaligned_le16(8, fb + 24);		/* vtable size */
	put_unaligned_le16(8, fb + 26);		/* table size */
	put_unaligned_le16(4, fb + 28);		/* field +4 offset */
	put_unaligned_le16(6, fb + 30);		/* field +6 offset */
	/* Subtable at offset 32 */
	put_unaligned_le32(8, fb + 32);		/* soffset back to vtable (32-8=24) */
	fb[36] = field4;
	fb[38] = field6;

	return hci_lga_aoc_efw_send_frame(bt, LGA_EFW_MSG_FLATBUFFER,
					  LGA_EFW_FLAG_NORMAL,
					  NULL, 0, fb, sizeof(fb));
}

/* Canonical 44-byte FlatBuffer for ConfigUART (union_type=5: u32 baud, u8 stop_bits) */
static int hci_lga_aoc_efw_send_fb_uart(struct hci_lga_aoc *bt, u32 baud,
					u8 stop_bits)
{
	u8 fb[44] = {};

	put_unaligned_le32(12, fb + 0);
	put_unaligned_le16(8, fb + 4);
	put_unaligned_le16(12, fb + 6);
	put_unaligned_le16(4, fb + 8);
	put_unaligned_le16(8, fb + 10);
	put_unaligned_le32(8, fb + 12);
	fb[16] = LGA_EFW_FB_CONFIG_UART;
	put_unaligned_le32(12, fb + 20);
	put_unaligned_le16(8, fb + 24);
	put_unaligned_le16(12, fb + 26);
	put_unaligned_le16(4, fb + 28);
	put_unaligned_le16(8, fb + 30);
	put_unaligned_le32(8, fb + 32);
	put_unaligned_le32(baud, fb + 36);
	fb[40] = stop_bits;

	return hci_lga_aoc_efw_send_frame(bt, LGA_EFW_MSG_FLATBUFFER,
					  LGA_EFW_FLAG_NORMAL,
					  NULL, 0, fb, sizeof(fb));
}

/* Canonical 32-byte FlatBuffer for empty-subtable commands (DumpDebug=6, QueryState=7) */
static int hci_lga_aoc_efw_send_fb_empty(struct hci_lga_aoc *bt, u8 union_type,
					 u8 flags)
{
	u8 fb[32] = {};

	put_unaligned_le32(12, fb + 0);
	put_unaligned_le16(8, fb + 4);
	put_unaligned_le16(12, fb + 6);
	put_unaligned_le16(4, fb + 8);
	put_unaligned_le16(8, fb + 10);
	put_unaligned_le32(8, fb + 12);
	fb[16] = union_type;
	put_unaligned_le32(8, fb + 20);		/* subtable at offset 28 (20+8=28) */
	put_unaligned_le16(4, fb + 24);		/* empty subtable vtable at 24 */
	put_unaligned_le16(4, fb + 26);
	put_unaligned_le32(4, fb + 28);		/* soffset back to 24 (28-4=24) */

	return hci_lga_aoc_efw_send_frame(bt, LGA_EFW_MSG_FLATBUFFER,
					  flags, NULL, 0, fb, sizeof(fb));
}

static const char *hci_lga_aoc_efw_state_str(u8 state)
{
	switch (state) {
	case 0:
		return "kUninitialized";
	case 1:
		return "kRecoverable";
	case 2:
		return "kUnrecoverable";
	default:
		return "unknown";
	}
}

static u8 hci_lga_aoc_efw_msg_to_h4(u32 msg_type)
{
	switch (msg_type) {
	case LGA_EFW_MSG_HCI_TX_OR_EVT:
		return HCI_EVENT_PKT;
	case LGA_EFW_MSG_HCI_ACL_RX:
		return HCI_ACLDATA_PKT;
	case LGA_EFW_MSG_HCI_SCO_RX:
		return HCI_SCODATA_PKT;
	case LGA_EFW_MSG_HCI_ISO_RX:
		return HCI_ISODATA_PKT;
	default:
		return 0;
	}
}

static bool hci_lga_aoc_efw_decode_reply(const u8 *fb, u32 len,
					 u8 *union_type, u8 *state)
{
	u32 root_off, tbl_rel, sub_off;
	s32 root_voff, sub_voff;
	u16 rv_sz, type_off, tbl_off, sv_sz, state_off;

	if (len < 12U)
		return false;

	root_off = get_unaligned_le32(fb);
	if (root_off > len - 4U)
		return false;

	root_voff = (s32)get_unaligned_le32(fb + root_off);
	if (root_voff <= 0 || (u32)root_voff > root_off)
		return false;

	root_voff = (s32)root_off - root_voff;
	if ((u32)root_voff + 8U > len)
		return false;

	rv_sz = get_unaligned_le16(fb + root_voff);
	if (rv_sz < 8U || (u32)root_voff + rv_sz > len)
		return false;

	type_off = get_unaligned_le16(fb + root_voff + 4);
	tbl_off = get_unaligned_le16(fb + root_voff + 6);
	if (!type_off || !tbl_off || root_off + type_off >= len ||
	    root_off + tbl_off + 4U > len)
		return false;

	*union_type = fb[root_off + type_off];
	*state = 0;

	tbl_rel = get_unaligned_le32(fb + root_off + tbl_off);
	if (tbl_rel > len - (root_off + tbl_off))
		return true;
	sub_off = root_off + tbl_off + tbl_rel;
	if (sub_off > len - 4U)
		return true;

	sub_voff = (s32)get_unaligned_le32(fb + sub_off);
	if (sub_voff <= 0 || (u32)sub_voff > sub_off)
		return true;

	sub_voff = (s32)sub_off - sub_voff;
	if ((u32)sub_voff + 4U > len)
		return true;

	sv_sz = get_unaligned_le16(fb + sub_voff);
	if (sv_sz >= 6U && (u32)sub_voff + sv_sz <= len) {
		state_off = get_unaligned_le16(fb + sub_voff + 4);
		if (state_off && sub_off + state_off < len)
			*state = fb[sub_off + state_off];
	}

	return true;
}

static const char *hci_lga_aoc_efw_pkt_tag_str(u8 tag)
{
	switch (tag) {
	case 0x1f:
		return "AP->Ctrl";
	case 0xef:
		return "Ctrl->AoC";
	case 0xfe:
		return "UART->Ctrl";
	default:
		return "pkt";
	}
}

static void hci_lga_aoc_efw_log_debug_dump(struct hci_lga_aoc *bt, u32 seq,
					   const u8 *payload, size_t body_len)
{
	size_t dump_len = min_t(size_t, body_len, 256U);
	size_t off;

	for (off = 0; off < dump_len; off += 32U) {
		size_t chunk = min_t(size_t, dump_len - off, 32U);

		dev_info(bt->aoc_dev,
			 "EFW debug_dump seq=%u [%zu..%zu/%zu]: %*ph\n",
			 seq, off, off + chunk - 1U, body_len,
			 (int)chunk, payload + off);
	}

	off = 6U;
	while (off + 5U <= body_len) {
		u32 ts_ms = get_unaligned_le32(payload + off);
		u8 tag = payload[off + 4U];

		switch (tag) {
		case 0x00:
			if (off + 6U > body_len)
				return;
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums power_state=%u (%s)\n",
				 ts_ms, payload[off + 5U],
				 payload[off + 5U] == 0 ? "ON" : "other");
			off += 6U;
			break;
		case 0x01:
			if (off + 6U > body_len)
				return;
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums dev_wake=%u\n",
				 ts_ms, payload[off + 5U]);
			off += 6U;
			break;
		case 0x02:
			if (off + 9U > body_len)
				return;
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums baud=%u\n",
				 ts_ms, get_unaligned_le32(payload + off + 5U));
			off += 9U;
			break;
		case 0x03:
			if (off + 7U > body_len)
				return;
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums lpm=%u->%u\n",
				 ts_ms, payload[off + 5U], payload[off + 6U]);
			off += 7U;
			break;
		case 0x05:
			if (off + 6U > body_len)
				return;
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums host_wake=%u\n",
				 ts_ms, payload[off + 5U]);
			off += 6U;
			break;
		default:
			if (tag >= 0x10 && off + 7U <= body_len) {
				u16 pkt_len = get_unaligned_le16(payload + off + 5U);
				size_t stored = min_t(size_t, pkt_len, 8U);

				if (off + 7U + stored <= body_len) {
					dev_info(bt->aoc_dev,
						 "EFW dbg t=%ums %s(tag=%#02x len=%u): %*ph\n",
						 ts_ms,
						 hci_lga_aoc_efw_pkt_tag_str(tag),
						 tag, pkt_len, (int)stored,
						 payload + off + 7U);
					off += 7U + stored;
					break;
				}
			}
			dev_info(bt->aoc_dev,
				 "EFW dbg t=%ums tag=%#02x tail(%zuB): %*ph\n",
				 ts_ms, tag, body_len - off,
				 (int)min_t(size_t, body_len - off, 32U),
				 payload + off);
			return;
		}
	}
}

static void hci_lga_aoc_efw_handle_frame(struct hci_lga_aoc *bt,
					 const u8 *frame, u32 payload_len,
					 u32 seq, u32 reply_seq, u8 pkt_kind)
{
	const u8 *payload = frame + bt->efw_aoc_hdr_size;
	struct hci_dev *hdev = bt->hdev;
	struct sk_buff *skb;
	u32 msg_type;
	u8 h4_type;
	size_t body_len;

	if (pkt_kind == LGA_EFW_PKT_KIND_REPLY) {
		u8 utype = 0, state = 0;

		if (hci_lga_aoc_efw_decode_reply(payload, payload_len,
						 &utype, &state))
			dev_info(bt->aoc_dev,
				 "EFW reply seq=%u reply_to=%u len=%u fb_type=%u chre_offload_state=%u (%s): %*ph\n",
				 seq, reply_seq, payload_len, utype, state,
				 utype == 1 ? hci_lga_aoc_efw_state_str(state) : "n/a",
				 (int)min_t(u32, payload_len, 32U), payload);
		else
			dev_info(bt->aoc_dev,
				 "EFW reply seq=%u reply_to=%u len=%u: %*ph\n",
				 seq, reply_seq, payload_len,
				 (int)min_t(u32, payload_len, 32U), payload);
		return;
	}

	if (pkt_kind != LGA_EFW_PKT_KIND_NORMAL) {
		if (debug_rx)
			dev_info(bt->aoc_dev,
				 "EFW rx kind=%u seq=%u len=%u: %*ph\n",
				 pkt_kind, seq, payload_len,
				 (int)min_t(u32, payload_len, 32U), payload);
		return;
	}

	if (payload_len < sizeof(__le32)) {
		bt_dev_warn_ratelimited(hdev,
					"short EFW normal payload (%u B)",
					payload_len);
		hdev->stat.err_rx++;
		return;
	}

	msg_type = get_unaligned_le32(payload);
	body_len = payload_len - sizeof(__le32);
	payload += sizeof(__le32);

	if (debug_rx) {
		if (msg_type == LGA_EFW_MSG_HCI_TX_OR_EVT && body_len >= 6U &&
		    payload[0] == 0xff && payload[2] == 0x58 &&
		    payload[3] == 0x14 && payload[4] == 0x5a)
			hci_lga_aoc_efw_log_debug_dump(bt, seq, payload,
						       body_len);
		else if (msg_type == LGA_EFW_MSG_HCI_TX_OR_EVT &&
			 body_len == 6U && payload[0] == 0xff &&
			 payload[1] == 0x04 && payload[2] == 0x58 &&
			 payload[3] == 0x05 && payload[4] == 0x00)
			dev_info(bt->aoc_dev,
				 "EFW hw_err seq=%u code=%#02x (%s)\n",
				 seq, payload[5],
				 payload[5] == 0xe2 ?
				 "UART TX mismatch / CTS stall" : "other");
		else
			dev_info(bt->aoc_dev,
				 "EFW rx msg_type=%u seq=%u len=%zu: %*ph\n",
				 msg_type, seq, body_len,
				 (int)min_t(size_t, body_len, 64U), payload);
	}

	h4_type = hci_lga_aoc_efw_msg_to_h4(msg_type);
	if (!h4_type) {
		if (msg_type != LGA_EFW_MSG_HCI_DIAG_RX)
			bt_dev_dbg(hdev, "ignoring EFW msg_type=%u (%zu B)",
				   msg_type, body_len);
		return;
	}

	if (!body_len || body_len > HCI_MAX_FRAME_SIZE) {
		bt_dev_err_ratelimited(hdev,
				       "invalid EFW HCI body length %zu (msg_type=%u)",
				       body_len, msg_type);
		hdev->stat.err_rx++;
		return;
	}

	skb = bt_skb_alloc(body_len, GFP_KERNEL);
	if (!skb) {
		hdev->stat.err_rx++;
		return;
	}

	hci_skb_pkt_type(skb) = h4_type;
	skb_put_data(skb, payload, body_len);
	hdev->stat.byte_rx += body_len + 1U;
	hci_recv_frame(hdev, skb);
}

static int hci_lga_aoc_efw_rx_thread(struct hci_lga_aoc *bt)
{
	u8 *stream;
	size_t buffered = 0;

	stream = kmalloc(LGA_EFW_RX_STREAM_SIZE, GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	while (!kthread_should_stop()) {
		size_t space = LGA_EFW_RX_STREAM_SIZE - buffered;
		size_t off = 0;
		u32 credit_delta = 0;
		ssize_t n;

		hci_lga_aoc_efw_drain_ctl(bt);

		if (!space) {
			bt_dev_err(bt->hdev, "EFW RX stream overflow, resetting");
			buffered = 0;
			space = LGA_EFW_RX_STREAM_SIZE;
		}

		n = lga_aoc_ipc_read_timeout(bt->aoc_dev, bt->rx_svc.index,
					     stream + buffered, space,
					     msecs_to_jiffies(LGA_EFW_RX_POLL_MS));
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
					       "EFW AoC RX read failed: %zd", n);
			msleep(20);
			continue;
		}

		buffered += (size_t)n;

		while (buffered - off >= bt->efw_aoc_hdr_size) {
			const u8 *f = stream + off;
			u32 payload_len = get_unaligned_le32(f + 16);
			u32 seq = get_unaligned_le32(f + 20);
			u32 reply_seq = get_unaligned_le32(f + 24);
			u8 pkt_kind = f[28];
			size_t raw_len, frame_len;

			if (payload_len > LGA_EFW_RX_STREAM_SIZE -
					  bt->efw_aoc_hdr_size) {
				bt_dev_err_ratelimited(bt->hdev,
						       "corrupt EFW RX header (payload_len=%u kind=%u), dropping %zu B",
						       payload_len, pkt_kind,
						       buffered - off);
				credit_delta += ALIGN_DOWN(buffered - off,
							   LGA_EFW_RING_ALIGN);
				off = buffered;
				break;
			}

			raw_len = bt->efw_aoc_hdr_size + payload_len;
			frame_len = ALIGN(raw_len, LGA_EFW_RING_ALIGN);
			if (frame_len > LGA_EFW_RX_STREAM_SIZE) {
				credit_delta += ALIGN_DOWN(buffered - off,
							   LGA_EFW_RING_ALIGN);
				off = buffered;
				break;
			}

			if (buffered - off < frame_len)
				break;

			hci_lga_aoc_efw_handle_frame(bt, f, payload_len, seq,
						     reply_seq, pkt_kind);
			off += frame_len;
			credit_delta += frame_len;
		}

		if (off) {
			if (off < buffered)
				memmove(stream, stream + off, buffered - off);
			buffered -= off;
		}

		if (credit_delta)
			hci_lga_aoc_efw_send_credit(bt, credit_delta);
	}

	kfree(stream);
	return 0;
}

/* ---- Legacy AOCC / raw H4 reassembly ---- */

static void hci_lga_aoc_rx_reset_locked(struct hci_lga_aoc *bt)
{
	kfree_skb(bt->rx_skb);
	bt->rx_skb = NULL;
	bt->rx_hdr_len = 0;
	bt->rx_expect_len = 0;
}

static void hci_lga_aoc_rx_reset(struct hci_lga_aoc *bt)
{
	mutex_lock(&bt->rx_lock);
	hci_lga_aoc_rx_reset_locked(bt);
	mutex_unlock(&bt->rx_lock);
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
		struct sk_buff *ready_skb = NULL;
		size_t need, take;

		mutex_lock(&bt->rx_lock);
		if (!bt->rx_skb) {
			u8 pkt_type = *data++;
			size_t hdr_len = hci_lga_aoc_pkt_hdr_len(pkt_type);

			count--;
			if (!hdr_len) {
				bt_dev_warn_ratelimited(hdev,
							"unknown H4 packet type %#02x",
							pkt_type);
				hdev->stat.err_rx++;
				mutex_unlock(&bt->rx_lock);
				continue;
			}

			bt->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE,
						  GFP_KERNEL);
			if (!bt->rx_skb) {
				hdev->stat.err_rx++;
				mutex_unlock(&bt->rx_lock);
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
			hci_lga_aoc_rx_reset_locked(bt);
			mutex_unlock(&bt->rx_lock);
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
				hci_lga_aoc_rx_reset_locked(bt);
				mutex_unlock(&bt->rx_lock);
				continue;
			}
		}

		if (bt->rx_skb->len == bt->rx_expect_len) {
			ready_skb = bt->rx_skb;
			bt->rx_skb = NULL;
			bt->rx_hdr_len = 0;
			bt->rx_expect_len = 0;
		}
		mutex_unlock(&bt->rx_lock);

		if (ready_skb)
			hci_recv_frame(hdev, ready_skb);
	}
}

static int hci_lga_aoc_rx_thread(void *arg)
{
	struct hci_lga_aoc *bt = arg;
	u8 *rx_buf;

	if (bt->mode == HCI_LGA_AOC_MODE_EFW)
		return hci_lga_aoc_efw_rx_thread(bt);

	rx_buf = kmalloc(bt->rx_max_msg, GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	while (!kthread_should_stop()) {
		ssize_t n;
		const u8 *payload;
		size_t plen;

		n = lga_aoc_ipc_read_timeout(bt->aoc_dev, bt->rx_svc.index,
					     rx_buf, bt->rx_max_msg,
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

		if (bt->mode == HCI_LGA_AOC_MODE_AOCC) {
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

static void hci_lga_aoc_latch_ble_dbo(struct hci_lga_aoc *bt)
{
	if (!bt->bt_ble_dbo_gpio)
		return;

	gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 0);
	usleep_range(10000, 12000);
	gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 1);
	usleep_range(10000, 12000);
	gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 0);
	usleep_range(10000, 12000);
}

static void hci_lga_aoc_power_on(struct hci_lga_aoc *bt)
{
	struct gpio_desc *gp;

	if (!toggle_bt_power)
		return;

	/*
	 * BCM4390/5 on Laguna requires AP2WLAN_REG_ON_1V2 (bt-wlan-reg-on)
	 * asserted for the combo PMU/BT core to exit reset. Request it
	 * nonexclusively with GPIOD_ASIS so an active Wi-Fi session holding
	 * the line HIGH is never glitched, and only lower the line on close
	 * if this driver raised it.
	 */
	if (assert_wlan_reg_on && !bt->bt_wlan_reg_on_gpio) {
		gp = gpiod_get_optional(bt->aoc_dev, "bt-wlan-reg-on",
					GPIOD_ASIS | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
		if (IS_ERR(gp)) {
			dev_info(bt->aoc_dev,
				 "bt-wlan-reg-on GPIO unavailable (%ld), continuing without WL_REG_ON\n",
				 PTR_ERR(gp));
		} else if (gp) {
			bt->bt_wlan_reg_on_gpio = gp;
			if (gpiod_get_direction(gp) == 0 &&
			    gpiod_get_value_cansleep(gp) > 0) {
				bt->wlan_reg_on_owned = false;
				dev_info(bt->aoc_dev,
					 "bt-wlan-reg-on already high (shared), preserving state\n");
			} else {
				int err = gpiod_direction_output(gp, 1);

				if (err) {
					dev_warn(bt->aoc_dev,
						 "bt-wlan-reg-on output high failed (%d)\n",
						 err);
					bt->wlan_reg_on_owned = false;
				} else {
					bt->wlan_reg_on_owned = true;
					msleep(200);
					dev_info(bt->aoc_dev,
						 "bt-wlan-reg-on asserted (val=%d)\n",
						 gpiod_get_value_cansleep(gp));
				}
			}
		}
	}

	/*
	 * In EFW mode with AoC PMIC control enabled, AoC's BT power controller
	 * drives VBR_ON_IN/VBR_ON_LE over CPM MBA and owns XAOC_GPIO1
	 * (BT_DEV_WAKE). Avoid claiming those lines on AP so AP does not fight
	 * AoC for pin muxing or double-toggle BT_REG_ON.
	 */
	if (bt->mode == HCI_LGA_AOC_MODE_EFW && efw_pmic_ctrl)
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

	/*
	 * Keep XAOC_GPIO1 (BT_DEV_WAKE) owned by AoC in EFW mode even when
	 * efw_pmic_ctrl=0; claiming it via pinctrl_aoc switches dmux to AP
	 * GPIO and breaks AoC's internal BT_DEV_WAKE LPM control.
	 */
	if (bt->mode != HCI_LGA_AOC_MODE_EFW) {
		gp = gpiod_get_optional(bt->aoc_dev, "bt-device-wakeup",
					GPIOD_OUT_LOW);
		if (IS_ERR(gp)) {
			dev_info(bt->aoc_dev,
				 "bt-device-wakeup GPIO unavailable (%ld), skipping AP power toggle\n",
				 PTR_ERR(gp));
		} else {
			bt->bt_dev_wake_gpio = gp;
		}
	}

	if (bt->bt_shutdown_gpio) {
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 0);
		usleep_range(10000, 12000);
		hci_lga_aoc_latch_ble_dbo(bt);
		msleep(30);
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 1);
		usleep_range(10000, 12000);
		hci_lga_aoc_latch_ble_dbo(bt);
		if (debug_rx)
			dev_info(bt->aoc_dev,
				 "AP VBR_ON latched: shutdown=%d ble_dbo=%d\n",
				 gpiod_get_value_cansleep(bt->bt_shutdown_gpio),
				 bt->bt_ble_dbo_gpio ?
				 gpiod_get_value_cansleep(bt->bt_ble_dbo_gpio) : -1);
	}
	if (bt->bt_dev_wake_gpio)
		gpiod_set_value_cansleep(bt->bt_dev_wake_gpio, 1);
	if (bt->bt_shutdown_gpio || bt->bt_dev_wake_gpio)
		msleep(150);
}

static void hci_lga_aoc_power_off(struct hci_lga_aoc *bt)
{
	if (bt->bt_dev_wake_gpio) {
		gpiod_set_value_cansleep(bt->bt_dev_wake_gpio, 0);
		gpiod_put(bt->bt_dev_wake_gpio);
		bt->bt_dev_wake_gpio = NULL;
	}
	if (bt->bt_shutdown_gpio) {
		gpiod_set_value_cansleep(bt->bt_shutdown_gpio, 0);
		hci_lga_aoc_latch_ble_dbo(bt);
		gpiod_put(bt->bt_shutdown_gpio);
		bt->bt_shutdown_gpio = NULL;
	}
	if (bt->bt_ble_dbo_gpio) {
		gpiod_set_value_cansleep(bt->bt_ble_dbo_gpio, 0);
		gpiod_put(bt->bt_ble_dbo_gpio);
		bt->bt_ble_dbo_gpio = NULL;
	}
	if (bt->bt_wlan_reg_on_gpio) {
		if (bt->wlan_reg_on_owned)
			gpiod_set_value_cansleep(bt->bt_wlan_reg_on_gpio, 0);
		gpiod_put(bt->bt_wlan_reg_on_gpio);
		bt->bt_wlan_reg_on_gpio = NULL;
		bt->wlan_reg_on_owned = false;
	}
}

static void hci_lga_aoc_efw_disconnect_sync(struct hci_lga_aoc *bt,
					    bool send_power_off)
{
	u8 rep[LGA_EFW_CTL_MSG_SIZE] = {};
	u8 stale[256];
	int ret;

	if (bt->efw_connected && lga_aoc_is_online(bt->aoc_dev)) {
		if (send_power_off)
			hci_lga_aoc_efw_send_fb_u8_2(bt, LGA_EFW_FB_SET_POWER,
						     0, efw_pmic_ctrl ? 1 : 0);
		/*
		 * Wait for AoC to consume all pending TX ring frames (including
		 * SetPower(off)) before sending DISCONNECT so no unread TX
		 * bytes remain when AoC transitions back to DISCONNECTED.
		 */
		ret = lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
					   msecs_to_jiffies(1000));
		if (ret && ret != -ENODEV)
			bt_dev_warn(bt->hdev,
				    "EFW TX ring flush before DISCONNECT returned %d",
				    ret);
	}

	/*
	 * Stop rx_task before sending DISCONNECT so it does not consume
	 * DISCONNECT_ACK out from under the synchronous close handshake.
	 */
	if (bt->rx_task) {
		kthread_stop(bt->rx_task);
		bt->rx_task = NULL;
	}

	if (bt->efw_connected && lga_aoc_is_online(bt->aoc_dev)) {
		hci_lga_aoc_efw_drain_ctl(bt);
		ret = hci_lga_aoc_efw_send_disconnect(bt);
		if (!ret) {
			ret = hci_lga_aoc_efw_wait_ctl(bt,
						       LGA_EFW_CTL_DISCONNECT_ACK,
						       rep, 1000U);
			if (ret < 0)
				bt_dev_warn(bt->hdev,
					    "EFW DISCONNECT_ACK wait failed: %d",
					    ret);
		}
		while (lga_aoc_ipc_read(bt->aoc_dev, bt->rx_svc.index,
					stale, sizeof(stale)) > 0)
			;
	}

	bt->efw_connected = false;
}

static int hci_lga_aoc_open(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);
	int ret;

	if (!lga_aoc_is_online(bt->aoc_dev))
		return -ENODEV;

	if (bt->mode != HCI_LGA_AOC_MODE_EFW)
		hci_lga_aoc_power_on(bt);

	if (bt->mode == HCI_LGA_AOC_MODE_EFW) {
		if (lga_aoc_ipc_tx_pending(bt->aoc_dev, bt->tx_svc.index))
			bt_dev_warn(hdev,
				    "EFW TX ring has unconsumed bytes before CONNECT");
		ret = hci_lga_aoc_efw_connect(bt);
		if (ret) {
			bt_dev_err(hdev, "EFW CONNECT handshake failed: %d",
				   ret);
			hci_lga_aoc_power_off(bt);
			return ret;
		}
	} else {
		u32 chan = (u32)atomic_inc_return(&hci_lga_aoc_chan_seq);

		if (chan == 0 || chan >= LGA_AOCC_MAX_CHAN_ID) {
			atomic_set(&hci_lga_aoc_chan_seq, 1);
			chan = 1;
		}
		bt->channel_id = chan;
	}

	bt->rx_task = kthread_run(hci_lga_aoc_rx_thread, bt, "hci-lga-aoc");
	if (IS_ERR(bt->rx_task)) {
		ret = PTR_ERR(bt->rx_task);
		bt->rx_task = NULL;
		if (bt->mode == HCI_LGA_AOC_MODE_EFW)
			hci_lga_aoc_efw_disconnect_sync(bt, false);
		hci_lga_aoc_power_off(bt);
		return ret;
	}

	if (bt->mode == HCI_LGA_AOC_MODE_EFW) {
		ret = hci_lga_aoc_efw_send_fb_u8_2(bt, LGA_EFW_FB_ENABLE_HW,
						   1, debug_rx ? 1 : 0);
		if (ret) {
			bt_dev_err(hdev, "EFW EnableHWInterface failed: %d",
				   ret);
			goto err_stop_efw;
		}

		if (efw_power_cycle) {
			ret = hci_lga_aoc_efw_send_fb_u8_2(bt,
							   LGA_EFW_FB_SET_POWER,
							   0, efw_pmic_ctrl ? 1 : 0);
			if (ret) {
				bt_dev_err(hdev,
					   "EFW SetPower(off) cycle failed: %d",
					   ret);
				goto err_stop_efw;
			}
			lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
					     msecs_to_jiffies(500));
			msleep(30);
		}

		/*
		 * Assert optional bt-wlan-reg-on (when assert_wlan_reg_on=1)
		 * and, when AP drives VBR_ON (efw_pmic_ctrl=0), latch BT_REG_ON
		 * before SetPower(1) muxes XAOC_UART0 and asserts BT_DEV_WAKE,
		 * matching AoC's own SetPower(1, 1) ordering so UART/wake pads
		 * stay idle across the BT_REG_ON rising edge.
		 */
		hci_lga_aoc_power_on(bt);

		ret = hci_lga_aoc_efw_send_fb_u8_2(bt, LGA_EFW_FB_SET_POWER,
						   1, efw_pmic_ctrl ? 1 : 0);
		if (ret) {
			bt_dev_err(hdev, "EFW SetPower(on) failed: %d", ret);
			goto err_stop_efw;
		}

		lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
				     msecs_to_jiffies(500));
		if (efw_pmic_ctrl)
			msleep(150);

		if (efw_lpm_timeout >= 0) {
			u8 lpm_en = efw_lpm_timeout > 0 ? 1 : 0;
			u8 lpm_to = (u8)min(efw_lpm_timeout, 255);

			ret = hci_lga_aoc_efw_send_fb_u8_2(bt,
							   LGA_EFW_FB_SET_LPM,
							   lpm_en, lpm_to);
			if (ret)
				bt_dev_warn(hdev,
					    "EFW SetLpm(%u, %u) failed: %d",
					    lpm_en, lpm_to, ret);
		}

		bt->efw_cur_baud = LGA_EFW_INIT_BAUD;
		if (efw_uart_stop_bits >= 0) {
			u8 stop_bits = (u8)min(efw_uart_stop_bits, 2);

			ret = hci_lga_aoc_efw_send_fb_uart(bt,
							   LGA_EFW_INIT_BAUD,
							   stop_bits);
			if (ret)
				bt_dev_warn(hdev,
					    "EFW ConfigUART(%u, stop_bits=%u) failed: %d",
					    LGA_EFW_INIT_BAUD, stop_bits, ret);
		}

		ret = hci_lga_aoc_efw_send_fb_empty(bt, LGA_EFW_FB_QUERY_STATE,
						    LGA_EFW_FLAG_EXPECT_REPLY);
		if (ret)
			bt_dev_warn(hdev, "EFW QueryState failed: %d", ret);
	} else if (bt->mode == HCI_LGA_AOC_MODE_AOCC) {
		ret = hci_lga_aoc_send_ctrl(bt, LGA_AOCC_CMD_OPEN_CHANNEL,
					    bt->channel_id);
		if (ret) {
			bt_dev_err(hdev, "failed to open AoC channel %u: %d",
				   bt->channel_id, ret);
			kthread_stop(bt->rx_task);
			bt->rx_task = NULL;
			hci_lga_aoc_power_off(bt);
			return ret;
		}
	}

	bt_dev_info(hdev,
		    "opened AoC TX '%s' (idx=%u) / RX '%s' (idx=%u) mode=%s",
		    bt->tx_svc.name, bt->tx_svc.index,
		    bt->rx_svc.name, bt->rx_svc.index,
		    hci_lga_aoc_mode_str(bt->mode));
	return 0;

err_stop_efw:
	hci_lga_aoc_efw_disconnect_sync(bt, false);
	hci_lga_aoc_power_off(bt);
	return ret;
}

static int hci_lga_aoc_close(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);

	if (bt->mode == HCI_LGA_AOC_MODE_EFW) {
		hci_lga_aoc_efw_disconnect_sync(bt, true);
	} else {
		if (bt->mode == HCI_LGA_AOC_MODE_AOCC && bt->channel_id &&
		    lga_aoc_is_online(bt->aoc_dev))
			hci_lga_aoc_send_ctrl(bt, LGA_AOCC_CMD_CLOSE_CHANNEL,
					      bt->channel_id);
		if (bt->rx_task) {
			kthread_stop(bt->rx_task);
			bt->rx_task = NULL;
		}
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

	if (bt->mode == HCI_LGA_AOC_MODE_EFW) {
		u8 tx_flag = efw_tx_flag >= 0 ? (efw_tx_flag ? 1U : 0U) :
			     (pkt_type == HCI_COMMAND_PKT ? 1U : 0U);
		u8 prefix[2] = { tx_flag, pkt_type };

		ret = hci_lga_aoc_efw_send_frame(bt, LGA_EFW_MSG_HCI_TX_OR_EVT,
						 LGA_EFW_FLAG_NORMAL,
						 prefix, sizeof(prefix),
						 skb->data, skb->len);
		if (ret) {
			hdev->stat.err_tx++;
			return ret;
		}
		hdev->stat.byte_tx += skb->len + 1U;
		kfree_skb(skb);
		return 0;
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

static int hci_lga_aoc_efw_set_host_baudrate(struct hci_lga_aoc *bt, u32 speed)
{
	u8 stop_bits = efw_uart_stop_bits >= 0 ?
		       (u8)min(efw_uart_stop_bits, 2) : 1U;
	int ret;

	if (bt->mode != HCI_LGA_AOC_MODE_EFW)
		return 0;

	ret = hci_lga_aoc_efw_send_fb_uart(bt, speed, stop_bits);
	if (ret) {
		bt_dev_err(bt->hdev, "EFW ConfigUART(%u, stop_bits=%u) failed: %d",
			   speed, stop_bits, ret);
		return ret;
	}

	ret = lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
				   msecs_to_jiffies(500));
	if (ret && ret != -ENODEV)
		bt_dev_warn(bt->hdev, "EFW TX flush after ConfigUART(%u): %d",
			    speed, ret);

	msleep(20);
	bt->efw_cur_baud = speed;
	return 0;
}

static int hci_lga_aoc_set_baudrate(struct hci_dev *hdev, unsigned int speed)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);
	struct bcm_update_uart_baud_rate param;
	struct sk_buff *skb;
	int err;

	if (bt->mode != HCI_LGA_AOC_MODE_EFW || !speed)
		return 0;

	if (speed > 3000000U) {
		struct bcm_write_uart_clock_setting clock = {
			.type = BCM_UART_CLOCK_48MHZ,
		};

		skb = __hci_cmd_sync(hdev, LGA_BCM_OP_WRITE_UART_CLOCK,
				     sizeof(clock), &clock, HCI_INIT_TIMEOUT);
		if (IS_ERR(skb)) {
			err = PTR_ERR(skb);
			bt_dev_err(hdev,
				   "BCM: failed to write UART clock (%d)", err);
			return err;
		}
		kfree_skb(skb);
	}

	param.zero = cpu_to_le16(0);
	param.baud_rate = cpu_to_le32(speed);

	skb = __hci_cmd_sync(hdev, LGA_BCM_OP_UPDATE_BAUDRATE,
			     sizeof(param), &param, HCI_INIT_TIMEOUT);
	if (PTR_ERR_OR_ZERO(skb) == -ETIMEDOUT) {
		bt_dev_warn(hdev,
			    "BCM: baud rate change to %u timed out, retrying after reset",
			    speed);
		skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL,
				     HCI_INIT_TIMEOUT);
		if (!IS_ERR(skb)) {
			kfree_skb(skb);
			msleep(100);
			skb = __hci_cmd_sync(hdev, LGA_BCM_OP_UPDATE_BAUDRATE,
					     sizeof(param), &param,
					     HCI_INIT_TIMEOUT);
		}
	}
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		bt_dev_err(hdev,
			   "BCM: failed to write update baudrate %u (%d)",
			   speed, err);
		return err;
	}
	kfree_skb(skb);

	lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
			     msecs_to_jiffies(500));
	msleep(20);

	err = hci_lga_aoc_efw_set_host_baudrate(bt, speed);
	if (err)
		return err;

	bt_dev_info(hdev, "EFW UART baudrate switched to %u", speed);
	return 0;
}

static int hci_lga_aoc_setup(struct hci_dev *hdev)
{
	struct hci_lga_aoc *bt = hci_get_drvdata(hdev);
	bool fw_load_done = false;
	struct sk_buff *skb;
	int ret;

	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "HCI Reset over AoC failed: %ld",
			   PTR_ERR(skb));
		if (bt->mode == HCI_LGA_AOC_MODE_EFW && debug_rx &&
		    bt->efw_connected && lga_aoc_is_online(bt->aoc_dev)) {
			if (efw_cts_probe) {
				static const u8 hci_rst[3] = { 0x03, 0x0c, 0x00 };
				u8 prefix[2] = { 1U, HCI_COMMAND_PKT };
				unsigned int i;

				/*
				 * Exceed the 64-byte DW-APB TX FIFO so AoC's
				 * UART write helper hits its 2s timeout and
				 * emits vendor error 0xe2 if MSR.CTS == 0.
				 */
				for (i = 0; i < 18U; i++)
					hci_lga_aoc_efw_send_frame(bt,
								   LGA_EFW_MSG_HCI_TX_OR_EVT,
								   LGA_EFW_FLAG_NORMAL,
								   prefix, sizeof(prefix),
								   hci_rst, sizeof(hci_rst));
				lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
						     msecs_to_jiffies(2500));
				msleep(2200);
			}
			hci_lga_aoc_efw_send_fb_empty(bt, LGA_EFW_FB_DUMP_DEBUG,
						      LGA_EFW_FLAG_NORMAL);
			lga_aoc_ipc_flush_tx(bt->aoc_dev, bt->tx_svc.index,
					     msecs_to_jiffies(250));
			msleep(50);
		}
		return PTR_ERR(skb);
	}
	kfree_skb(skb);

	/*
	 * Step up to operational baudrate (default 3 Mbaud) before patchram
	 * download so ~820 KB BCM.hcd streams in ~3 s instead of >75 s.
	 */
	if (bt->mode == HCI_LGA_AOC_MODE_EFW && efw_uart_baud &&
	    efw_uart_baud != LGA_EFW_INIT_BAUD) {
		ret = hci_lga_aoc_set_baudrate(hdev, efw_uart_baud);
		if (ret)
			bt_dev_warn(hdev,
				    "pre-patchram baudrate %u failed (%d), continuing at %u",
				    efw_uart_baud, ret, bt->efw_cur_baud);
	}

	if (!run_btbcm_setup)
		return 0;

	/*
	 * btbcm_initialize() queries root DT compatible[0] ("google,lga-mustang"
	 * or "google,lga-rango") and requests brcm/BCM.<board>.hcd with fallback
	 * to brcm/BCM.hcd. Missing firmware (-ENOENT) is non-fatal (returns 0
	 * with fw_load_done=false).
	 */
	ret = btbcm_initialize(hdev, &fw_load_done, false);
	if (ret)
		return ret;

	/*
	 * Launch RAM (0xfc4e) resets BCM4390's UART back to 115200 baud.
	 * Resynchronize AoC XAOC_UART0 to 115200, then step the newly loaded
	 * RAM firmware back up to efw_uart_baud before btbcm_finalize().
	 */
	if (fw_load_done && bt->mode == HCI_LGA_AOC_MODE_EFW) {
		if (bt->efw_cur_baud != LGA_EFW_INIT_BAUD) {
			ret = hci_lga_aoc_efw_set_host_baudrate(bt,
								LGA_EFW_INIT_BAUD);
			if (ret)
				return ret;
		}
		if (efw_uart_baud && efw_uart_baud != LGA_EFW_INIT_BAUD) {
			ret = hci_lga_aoc_set_baudrate(hdev, efw_uart_baud);
			if (ret)
				bt_dev_warn(hdev,
					    "post-patchram baudrate %u failed (%d), continuing at %u",
					    efw_uart_baud, ret, bt->efw_cur_baud);
		}
	}

	ret = btbcm_finalize(hdev, &fw_load_done, false);
	if (ret)
		return ret;

	if (hci_test_quirk(hdev, HCI_QUIRK_INVALID_BDADDR))
		hci_set_quirk(hdev, HCI_QUIRK_USE_BDADDR_PROPERTY);

	return 0;
}

static int hci_lga_aoc_calc_msg_len(const struct lga_aoc_service_info *svc,
				    bool is_tx, size_t limit, size_t *out_len)
{
	u32 slots = is_tx ? svc->down_slots : svc->up_slots;
	u32 size = is_tx ? svc->down_size : svc->up_size;
	size_t max_msg = limit;

	if (svc->type == LGA_AOC_SVC_BUFFER)
		return -EOPNOTSUPP;
	if (!slots || !size)
		return -EINVAL;

	if (svc->type == LGA_AOC_SVC_QUEUE) {
		if (size <= 8U)
			return -EINVAL;
		max_msg = min_t(size_t, max_msg, size - 8U);
	} else if (svc->type == LGA_AOC_SVC_RING) {
		max_msg = min_t(size_t, max_msg, size);
	} else {
		return -EOPNOTSUPP;
	}

	*out_len = max_msg;
	return 0;
}

static int hci_lga_aoc_attach_locked(void)
{
	struct device *aoc_dev;
	struct lga_aoc_service_info tx_svc, rx_svc, ctl_svc = {};
	const char *tx_name = service_name;
	const char *rx_name = rx_service_name;
	const char *ctl_name = ctl_service_name;
	struct hci_lga_aoc *bt;
	struct hci_dev *hdev;
	enum hci_lga_aoc_mode mode;
	bool has_ctl = false;
	size_t tx_limit, tx_max_msg, rx_max_msg, hdr_overhead;
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

	ret = lga_aoc_find_service(aoc_dev, tx_name, &tx_svc);
	if (ret && !strcmp(tx_name, "com.google.bt") && efw_transport) {
		ret = lga_aoc_find_service(aoc_dev, LGA_EFW_TX_SERVICE_DEFAULT,
					   &tx_svc);
		if (!ret) {
			tx_name = LGA_EFW_TX_SERVICE_DEFAULT;
			if (!rx_name || !*rx_name)
				rx_name = LGA_EFW_RX_SERVICE_DEFAULT;
			if (!ctl_name || !*ctl_name)
				ctl_name = LGA_EFW_CTL_SERVICE_DEFAULT;
		}
	}
	if (ret) {
		dev_err(aoc_dev, "AoC service '%s' not found: %d\n",
			tx_name, ret);
		put_device(aoc_dev);
		return ret;
	}

	if ((!rx_name || !*rx_name) &&
	    !strcmp(tx_name, LGA_EFW_TX_SERVICE_DEFAULT))
		rx_name = LGA_EFW_RX_SERVICE_DEFAULT;
	if (!rx_name || !*rx_name)
		rx_name = tx_name;

	if (!strcmp(rx_name, tx_name)) {
		rx_svc = tx_svc;
	} else {
		ret = lga_aoc_find_service(aoc_dev, rx_name, &rx_svc);
		if (ret) {
			dev_err(aoc_dev, "AoC RX service '%s' not found: %d\n",
				rx_name, ret);
			put_device(aoc_dev);
			return ret;
		}
	}

	if (efw_transport && (!ctl_name || !*ctl_name) &&
	    !strcmp(tx_name, LGA_EFW_TX_SERVICE_DEFAULT))
		ctl_name = LGA_EFW_CTL_SERVICE_DEFAULT;

	if (efw_transport && ctl_name && *ctl_name) {
		ret = lga_aoc_find_service(aoc_dev, ctl_name, &ctl_svc);
		if (ret) {
			dev_err(aoc_dev, "AoC ctl service '%s' not found: %d\n",
				ctl_name, ret);
			put_device(aoc_dev);
			return ret;
		}
		if (ctl_svc.type != LGA_AOC_SVC_QUEUE ||
		    ctl_svc.down_size < LGA_EFW_CTL_MSG_SIZE + 8U ||
		    ctl_svc.up_size < LGA_EFW_CTL_MSG_SIZE + 8U) {
			dev_err(aoc_dev,
				"AoC ctl service '%s' incompatible (type=%u down=%u up=%u)\n",
				ctl_svc.name, ctl_svc.type,
				ctl_svc.down_size, ctl_svc.up_size);
			put_device(aoc_dev);
			return -EINVAL;
		}
		has_ctl = true;
		mode = HCI_LGA_AOC_MODE_EFW;
	} else {
		mode = aocc_framing ? HCI_LGA_AOC_MODE_AOCC :
				      HCI_LGA_AOC_MODE_RAW;
	}

	tx_limit = mode == HCI_LGA_AOC_MODE_EFW ? LGA_EFW_MAX_FRAME_SIZE :
						  LGA_AOCC_MAX_MSG_SIZE;

	ret = hci_lga_aoc_calc_msg_len(&tx_svc, true, tx_limit, &tx_max_msg);
	if (ret) {
		dev_err(aoc_dev, "AoC TX service '%s' unsupported/invalid: %d\n",
			tx_svc.name, ret);
		put_device(aoc_dev);
		return ret;
	}

	ret = hci_lga_aoc_calc_msg_len(&rx_svc, false, LGA_AOCC_MAX_MSG_SIZE,
				       &rx_max_msg);
	if (ret) {
		dev_err(aoc_dev, "AoC RX service '%s' unsupported/invalid: %d\n",
			rx_svc.name, ret);
		put_device(aoc_dev);
		return ret;
	}

	if (mode == HCI_LGA_AOC_MODE_EFW)
		hdr_overhead = LGA_EFW_AP_HDR_SIZE + sizeof(__le32) + 2U;
	else if (mode == HCI_LGA_AOC_MODE_AOCC)
		hdr_overhead = LGA_AOCC_HDR_SIZE;
	else
		hdr_overhead = 0;

	if (tx_max_msg <= hdr_overhead + 1U || rx_max_msg <= 1U) {
		dev_err(aoc_dev,
			"AoC service '%s'/'%s' message size (%zu/%zu) too small\n",
			tx_svc.name, rx_svc.name, tx_max_msg, rx_max_msg);
		put_device(aoc_dev);
		return -EINVAL;
	}

	bt = kzalloc(sizeof(*bt), GFP_KERNEL);
	if (!bt) {
		put_device(aoc_dev);
		return -ENOMEM;
	}

	bt->tx_buf = kmalloc(tx_max_msg, GFP_KERNEL);
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
	mutex_init(&bt->ctl_lock);
	mutex_init(&bt->rx_lock);
	bt->aoc_dev = aoc_dev;
	bt->hdev = hdev;
	bt->tx_svc = tx_svc;
	bt->rx_svc = rx_svc;
	bt->ctl_svc = ctl_svc;
	bt->has_ctl_svc = has_ctl;
	bt->mode = mode;
	bt->efw_aoc_hdr_size = LGA_EFW_AOC_HDR_SIZE_DEFAULT;
	bt->tx_max_msg = tx_max_msg;
	bt->rx_max_msg = rx_max_msg;
	bt->max_payload = tx_max_msg - hdr_overhead;

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
		 "registered %s over AoC mode=%s TX '%s' (idx=%u mbox=%u) / RX '%s' (idx=%u)%s%s%s\n",
		 hdev->name, hci_lga_aoc_mode_str(mode),
		 tx_svc.name, tx_svc.index, tx_svc.mbox_index,
		 rx_svc.name, rx_svc.index,
		 has_ctl ? " / CTL '" : "",
		 has_ctl ? ctl_svc.name : "",
		 has_ctl ? "'" : "");
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
