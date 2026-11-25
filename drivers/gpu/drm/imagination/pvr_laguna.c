// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Clean-room Tensor G5 (Laguna) platform glue for PowerVR Volcanic
 * (DXT-48-1536, BVNC 71.2.2448.1212).
 */

#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_laguna.h"
#include "pvr_power.h"
#include "pvr_volcanic_defs.h"

#include <drm/drm_print.h>

#include <linux/align.h>
#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_IPC)
#include <linux/trusty/trusty_ipc.h>
#endif

/* Core-logic power-on/off CSRs (verified in Stage G0 probe on mustang/rango). */
#define PVR_LAGUNA_GPU_CTRL_POWER_OFF_ACK	BIT(0)
#define PVR_LAGUNA_GPU_CTRL_POWER_GATE_ACK	BIT(1)
#define PVR_LAGUNA_GPU_CTRL_ACK_MASK		(PVR_LAGUNA_GPU_CTRL_POWER_OFF_ACK | \
						 PVR_LAGUNA_GPU_CTRL_POWER_GATE_ACK)
#define PVR_LAGUNA_GPU_CTRL_POWER_ON_VAL	0x0U
#define PVR_LAGUNA_GPU_CTRL_POWER_OFF_VAL	0x8U

#define PVR_LAGUNA_GPU_DTS_PWRSWITCH_CTRL	0x00U
#define PVR_LAGUNA_GPU_DTS_ISO_EN		0x04U
#define PVR_LAGUNA_GPU_DTS_PWRSTATE		0x08U

#define PVR_LAGUNA_GPU_PVTC_CLK_SYNTH		0x10U
#define PVR_LAGUNA_GPU_PVTC_SDIF_DISABLE	0x20U
#define PVR_LAGUNA_GPU_PVTC_SDIF_STATUS		0x24U
#define PVR_LAGUNA_GPU_PVTC_SDIF		0x28U
#define PVR_LAGUNA_GPU_PVTC_PWR_STATUS		0x23CU

#define PVR_LAGUNA_GPU_PVTC_PWROFF_REQ		0x00U
#define PVR_LAGUNA_GPU_PVTC_PWROFF_ACK		0x04U

/* SLC PBHA LUT per-OSID register geometry (OSID 1 only; 32-bit writes only). */
#define PVR_LAGUNA_REQ_PBHA_OSID_STRIDE		0x200U
#define PVR_LAGUNA_REQ_PBHA_OSID_ENTRIES	16U
#define PVR_LAGUNA_REQ_PBHA_OSID1_DEFAULT	0x00000001U

#define PVR_LAGUNA_RA_LUT_OSID_STRIDE		0x40U
#define PVR_LAGUNA_RA_LUT_OSID_ENTRIES		4U
#define PVR_LAGUNA_RA_LUT_OSID1_DEFAULT		0x00000001U

/* PFSM CSR offsets (0x34813000, size 0xc). */
#define PVR_LAGUNA_PFSM_CTRL_OFF		0x00U
#define PVR_LAGUNA_PFSM_STATE_OFF		0x04U
#define PVR_LAGUNA_PFSM_STATUS_OFF		0x08U

/* Trusty IPC timeouts (ms). */
#define PVR_LAGUNA_TZ_CON_TIMEOUT_MS		30000U
#define PVR_LAGUNA_TZ_BUF_TIMEOUT_MS		30000U
#define PVR_LAGUNA_TZ_MSG_TIMEOUT_MS		30000U

/* Firmware trailer constants (DDK 25.x layout table at body_end - 4 KiB). */
#define PVR_LAGUNA_FW_BLOCK_SIZE		SZ_4K
#define PVR_LAGUNA_FW_LAYOUT_MAX_ENTRIES	16U
#define PVR_LAGUNA_FW_BUILD_ID_LEN		24U

enum pvr_laguna_fw_sec_type {
	PVR_LAGUNA_FW_SEC_NONE = 0,
	PVR_LAGUNA_FW_SEC_CODE = 1,
	PVR_LAGUNA_FW_SEC_RWDATA = 2,
	PVR_LAGUNA_FW_SEC_RODATA = 3,
	PVR_LAGUNA_FW_SEC_COREMEM_CODE = 4,
	PVR_LAGUNA_FW_SEC_COREMEM_DATA = 5,
};

struct pvr_laguna_fw_footer_hdr {
	__le32 fwabi_version;
	__le32 magic;
	__le32 footer_version;
	__le32 pa_range_count;
	char fw_build_id[PVR_LAGUNA_FW_BUILD_ID_LEN];
} __packed;

struct pvr_laguna_fw_info_hdr {
	__le32 info_version;
	__le32 header_len;
	__le32 layout_entry_num;
	__le32 layout_entry_size;
	__le64 bvnc;
	__le32 fw_page_size;
	__le32 flags;
} __packed;

struct pvr_laguna_fw_layout_entry {
	__le32 id;
	__le32 type;
	__le32 base_addr;
	__le32 max_size;
	__le32 alloc_size;
	__le32 alloc_offset;
} __packed;

struct pvr_laguna_tz_req_base {
	u32 command;
	u32 version;
};

struct pvr_laguna_tz_rsp_base {
	u32 command;
	u32 result;
};

struct pvr_laguna_tz_mem {
	u64 pa;
	u32 fw_va;
	u32 size;
};

struct pvr_laguna_tz_req_fw {
	struct pvr_laguna_tz_req_base base;
	struct pvr_laguna_tz_mem image_body;
	struct pvr_laguna_tz_mem image_header;
	struct pvr_laguna_tz_mem section[PVR_LAGUNA_GPU_SECURE_SEC_COUNT];
	u32 use_tf_a;
	u32 host_page_shift;
	u8 reserved[384];
} __packed __aligned(8);

static_assert(sizeof(struct pvr_laguna_tz_req_fw) ==
	      PVR_LAGUNA_GPU_SECURE_MAX_MSG_SIZE / 2);
static_assert(sizeof(struct pvr_laguna_tz_rsp_base) == 8);
static_assert(PVR_LAGUNA_FW_CODE_HEAP_SIZE +
	      PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE +
	      PVR_LAGUNA_FW_RODATA_HEAP_SIZE ==
	      PVR_LAGUNA_SECURE_FW_MEM_SIZE);
static_assert(PVR_LAGUNA_FW_PRIV_DATA_HEAP_OFFSET ==
	      PVR_LAGUNA_FW_CODE_HEAP_SIZE);
static_assert(PVR_LAGUNA_FW_RODATA_HEAP_OFFSET ==
	      PVR_LAGUNA_FW_CODE_HEAP_SIZE + PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE);

static int pvr_laguna_pvtc_sdif_cmd(struct pvr_laguna_data *laguna,
				    u8 addr, u32 data)
{
	u32 status;
	u32 cmd = BIT(31) | BIT(27) | ((u32)(addr & 0x7U) << 24) |
		  (data & 0xffffffU);

	writel(cmd, laguna->pvtc + PVR_LAGUNA_GPU_PVTC_SDIF);
	return readl_poll_timeout(laguna->pvtc + PVR_LAGUNA_GPU_PVTC_SDIF_STATUS,
				  status, (status & BIT(0)) == 0, 0, 1000);
}

static int pvr_laguna_set_pvtc_pwroff(struct pvr_laguna_data *laguna,
				      bool power_on)
{
	u32 req = power_on ? 0 : 1;
	u32 ack;

	writel(req, laguna->pvtc_pwroff + PVR_LAGUNA_GPU_PVTC_PWROFF_REQ);
	return readl_poll_timeout(laguna->pvtc_pwroff +
				  PVR_LAGUNA_GPU_PVTC_PWROFF_ACK,
				  ack, ack == req, 0, 1000);
}

static int pvr_laguna_core_logic_power_on(struct pvr_laguna_data *laguna)
{
	u32 ack, pwr_status;
	int ret;

	if (!laguna->gpu_control)
		return 0;

	writel(PVR_LAGUNA_GPU_CTRL_POWER_ON_VAL, laguna->gpu_control);
	ret = readl_poll_timeout(laguna->gpu_control, ack,
				 ack & PVR_LAGUNA_GPU_CTRL_ACK_MASK,
				 0, 10000);
	if (ret)
		return ret;

	if (laguna->dts) {
		writel(0x1, laguna->dts + PVR_LAGUNA_GPU_DTS_PWRSWITCH_CTRL);
		writel(0x0, laguna->dts + PVR_LAGUNA_GPU_DTS_ISO_EN);
	}

	if ((ack & PVR_LAGUNA_GPU_CTRL_POWER_OFF_ACK) && laguna->pvtc) {
		writel(BIT(24) | (1U << 16) | (1U << 8) | 1U,
		       laguna->pvtc + PVR_LAGUNA_GPU_PVTC_CLK_SYNTH);
		writel(0x7E, laguna->pvtc + PVR_LAGUNA_GPU_PVTC_SDIF_DISABLE);
		ret = readl_poll_timeout(laguna->pvtc +
					 PVR_LAGUNA_GPU_PVTC_PWR_STATUS,
					 pwr_status, pwr_status == 1, 10, 1000);
		if (ret)
			return ret;

		ret = pvr_laguna_pvtc_sdif_cmd(laguna, 0x1, 0xF0000);
		if (ret)
			return ret;
		ret = pvr_laguna_pvtc_sdif_cmd(laguna, 0x4, 0x7);
		if (ret)
			return ret;
		ret = pvr_laguna_pvtc_sdif_cmd(laguna, 0x5, 0x200);
		if (ret)
			return ret;
		ret = pvr_laguna_pvtc_sdif_cmd(laguna, 0x0, 0x508);
		if (ret)
			return ret;
	} else if ((ack & PVR_LAGUNA_GPU_CTRL_POWER_GATE_ACK) &&
		   laguna->pvtc_pwroff) {
		ret = pvr_laguna_set_pvtc_pwroff(laguna, true);
		if (ret)
			return ret;
	}

	return 0;
}

static void pvr_laguna_core_logic_power_off(struct pvr_laguna_data *laguna)
{
	u32 ack;

	if (!laguna->gpu_control)
		return;

	if (laguna->pvtc_pwroff)
		pvr_laguna_set_pvtc_pwroff(laguna, false);
	if (laguna->dts)
		writel(0x1, laguna->dts + PVR_LAGUNA_GPU_DTS_PWRSTATE);

	writel(PVR_LAGUNA_GPU_CTRL_POWER_OFF_VAL, laguna->gpu_control);
	readl_poll_timeout(laguna->gpu_control, ack,
			   ack & PVR_LAGUNA_GPU_CTRL_ACK_MASK, 0, 10000);
}

/**
 * pvr_laguna_slc_program_lut() - Program Laguna SLC PBHA & read-allocate LUTs
 * @pvr_dev: Target PowerVR device.
 *
 * Programs req_pbha_lut (0x34e08000, strictly 32-bit writes) and ra_lut_{0..2}
 * (0x3490{0,1,2}200) ONLY for OSID 1 (Non-Secure Linux). OSID 0 (FW) and
 * OSID 7 (TrustZone) are explicitly untouched so Trusty PTEs are never
 * polluted. Must be called after GPU power-on.
 */
void pvr_laguna_slc_program_lut(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	u32 osid1_req_base = PVR_LAGUNA_OSID_LINUX *
			     PVR_LAGUNA_REQ_PBHA_OSID_STRIDE;
	u32 osid1_ra_base = PVR_LAGUNA_OSID_LINUX *
			    PVR_LAGUNA_RA_LUT_OSID_STRIDE;

	if (!laguna)
		return;

	BUILD_BUG_ON(PVR_LAGUNA_OSID_LINUX == PVR_LAGUNA_OSID_FW);
	BUILD_BUG_ON(PVR_LAGUNA_OSID_LINUX == PVR_LAGUNA_OSID_TZ);

	if (laguna->req_pbha_lut) {
		for (u32 i = 0; i < PVR_LAGUNA_REQ_PBHA_OSID_ENTRIES; i++) {
			writel(PVR_LAGUNA_REQ_PBHA_OSID1_DEFAULT,
			       laguna->req_pbha_lut + osid1_req_base + i * 4U);
		}
	}

	for (u32 bank = 0; bank < PVR_LAGUNA_RA_LUT_BANKS; bank++) {
		if (!laguna->ra_lut[bank])
			continue;

		for (u32 i = 0; i < PVR_LAGUNA_RA_LUT_OSID_ENTRIES; i++) {
			writel(PVR_LAGUNA_RA_LUT_OSID1_DEFAULT,
			       laguna->ra_lut[bank] + osid1_ra_base + i * 4U);
		}
	}
}

static void pvr_laguna_mba_clear_irq(struct pvr_laguna_data *laguna)
{
	u32 itr;

	if (!laguna->mba_glue)
		return;

	itr = readl(laguna->mba_glue + PVR_LAGUNA_MBA_GLUE_INTR_ITR_OFF);
	if (itr)
		writel(itr, laguna->mba_glue + PVR_LAGUNA_MBA_GLUE_INTR_ITR_OFF);
}

/**
 * pvr_laguna_mba_send_msg() - Write a sideband message to a Laguna GPU MBA client
 * @pvr_dev: Target PowerVR device.
 * @client_idx: Client slot index (0..3, 0x1000 stride).
 * @msg: 32-bit COMMON_MSG payload.
 *
 * Returns:
 *  * 0 on success,
 *  * -%ENODEV if MBA client MMIO is unavailable, or
 *  * -%EINVAL if @client_idx is out of range.
 */
int pvr_laguna_mba_send_msg(struct pvr_device *pvr_dev, u32 client_idx, u32 msg)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	u32 offset;

	if (!laguna || !laguna->mba_client)
		return -ENODEV;

	if (client_idx >= PVR_LAGUNA_MBA_CLIENT_MAX)
		return -EINVAL;

	offset = client_idx * PVR_LAGUNA_MBA_CLIENT_STRIDE +
		 PVR_LAGUNA_MBA_CLIENT_COMMON_MSG_OFF;
	writel(msg, laguna->mba_client + offset);

	return 0;
}

unsigned long pvr_laguna_get_core_clk_rate(const struct pvr_device *pvr_dev)
{
	const struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (laguna && laguna->pf_state_clk) {
		unsigned long rate = clk_get_rate(laguna->pf_state_clk);

		if (rate)
			return rate;
	}

	if (laguna && laguna->core_freq_hz)
		return laguna->core_freq_hz;

	return PVR_LAGUNA_GPU_INIT_FREQ_HZ;
}

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_IPC)
static struct tipc_msg_buf *pvr_laguna_tz_handle_msg(void *data,
						     struct tipc_msg_buf *rxbuf)
{
	struct pvr_laguna_tz *tz = data;
	size_t len = mb_avail_data(rxbuf);

	mutex_lock(&tz->rsp_lock);
	if (!tz->rsp_buf) {
		dev_err(tz->dev, "pixel_gpu TZ: unexpected message, dropped\n");
	} else if (len > tz->rsp_size) {
		dev_err(tz->dev, "pixel_gpu TZ: reply too large (%zu > %zu)\n",
			len, tz->rsp_size);
		tz->rsp_res = -EMSGSIZE;
	} else {
		memcpy(tz->rsp_buf, mb_get_data(rxbuf, len), len);
		tz->rsp_res = (int)len;
	}
	mutex_unlock(&tz->rsp_lock);
	complete(&tz->reply);

	return rxbuf;
}

static void pvr_laguna_tz_handle_event(void *data, int event)
{
	struct pvr_laguna_tz *tz = data;

	WRITE_ONCE(tz->last_event, event);
	complete(&tz->reply);
}

static const struct tipc_chan_ops pvr_laguna_tz_ops = {
	.handle_msg = pvr_laguna_tz_handle_msg,
	.handle_event = pvr_laguna_tz_handle_event,
};

static void pvr_laguna_tz_close(struct pvr_laguna_tz *tz)
{
	if (tz->chan) {
		tipc_chan_shutdown(tz->chan);
		tipc_chan_destroy(tz->chan);
		tz->chan = NULL;
	}
}

static int pvr_laguna_tz_connect(struct pvr_laguna_tz *tz)
{
	struct tipc_chan *chan;
	int ret;

	chan = tipc_create_channel(NULL, &pvr_laguna_tz_ops, tz);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	WRITE_ONCE(tz->last_event, 0);
	reinit_completion(&tz->reply);
	ret = tipc_chan_connect(chan, tz->port);
	if (ret < 0)
		goto err_destroy;

	if (!wait_for_completion_timeout(&tz->reply,
					 msecs_to_jiffies(PVR_LAGUNA_TZ_CON_TIMEOUT_MS))) {
		ret = -ETIMEDOUT;
		tipc_chan_shutdown(chan);
		goto err_destroy;
	}

	if (READ_ONCE(tz->last_event) != TIPC_CHANNEL_CONNECTED) {
		ret = -ECONNREFUSED;
		tipc_chan_shutdown(chan);
		goto err_destroy;
	}

	tz->chan = chan;
	return 0;

err_destroy:
	tipc_chan_destroy(chan);
	return ret;
}

static int pvr_laguna_tz_xchg(struct pvr_laguna_tz *tz,
			      const void *req, size_t req_len,
			      void *rsp, size_t rsp_size)
{
	struct tipc_msg_buf *txbuf;
	bool retried = false;
	int ret;

	mutex_lock(&tz->req_lock);
again:
	if (!tz->chan) {
		ret = pvr_laguna_tz_connect(tz);
		if (ret)
			goto out_unlock;
	}

	txbuf = tipc_chan_get_txbuf_timeout(tz->chan,
					    PVR_LAGUNA_TZ_BUF_TIMEOUT_MS);
	if (IS_ERR(txbuf)) {
		ret = PTR_ERR(txbuf);
		goto out_unlock;
	}
	if (mb_avail_space(txbuf) < req_len) {
		dev_err(tz->dev,
			"pixel_gpu TZ: txbuf too small (%zu < %zu)\n",
			mb_avail_space(txbuf), req_len);
		tipc_chan_put_txbuf(tz->chan, txbuf);
		ret = -EMSGSIZE;
		goto out_unlock;
	}
	memcpy(mb_put_data(txbuf, req_len), req, req_len);

	mutex_lock(&tz->rsp_lock);
	tz->rsp_buf = rsp;
	tz->rsp_size = rsp_size;
	tz->rsp_res = -ENOMSG;
	mutex_unlock(&tz->rsp_lock);

	reinit_completion(&tz->reply);
	ret = tipc_chan_queue_msg(tz->chan, txbuf);
	if (ret < 0) {
		tipc_chan_put_txbuf(tz->chan, txbuf);
		pvr_laguna_tz_close(tz);
		if (!retried) {
			retried = true;
			goto again;
		}
		goto detach;
	}

	if (!wait_for_completion_timeout(&tz->reply,
					 msecs_to_jiffies(PVR_LAGUNA_TZ_MSG_TIMEOUT_MS))) {
		pvr_laguna_tz_close(tz);
		ret = -ETIMEDOUT;
		goto detach;
	}

	ret = 0;
detach:
	mutex_lock(&tz->rsp_lock);
	if (!ret)
		ret = tz->rsp_res;
	tz->rsp_buf = NULL;
	tz->rsp_size = 0;
	mutex_unlock(&tz->rsp_lock);
	if (READ_ONCE(tz->last_event) != TIPC_CHANNEL_CONNECTED) {
		pvr_laguna_tz_close(tz);
		if (ret == -ENOMSG)
			ret = -ECONNRESET;
	}
out_unlock:
	mutex_unlock(&tz->req_lock);
	return ret;
}
#else
static void pvr_laguna_tz_close(struct pvr_laguna_tz *tz)
{
}

static int pvr_laguna_tz_xchg(struct pvr_laguna_tz *tz,
			      const void *req, size_t req_len,
			      void *rsp, size_t rsp_size)
{
	return -EOPNOTSUPP;
}
#endif

static void pvr_laguna_tz_init(struct pvr_laguna_tz *tz, struct device *dev)
{
	tz->dev = dev;
	tz->port = PVR_LAGUNA_TRUSTY_GPU_PORT;
	mutex_init(&tz->req_lock);
	mutex_init(&tz->rsp_lock);
	init_completion(&tz->reply);
}

static int pvr_laguna_smc_gpu_control(struct pvr_device *pvr_dev,
				      unsigned long subcmd)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct arm_smccc_1_2_regs args = {
		.a0 = PVR_LAGUNA_SMC_OEM_GPU_CONTROL,
		.a1 = subcmd,
	};
	struct arm_smccc_1_2_regs res = {
		.a0 = 0xFFFFFFFFUL,
	};

	arm_smccc_1_2_smc(&args, &res);

	if (res.a0) {
		drm_err(drm_dev,
			"TF-A SMC_OEM_GPU_CONTROL subcmd=%lu failed (%#lx)\n",
			subcmd, res.a0);
		return -EIO;
	}

	return 0;
}

struct pvr_laguna_fw_parsed {
	u32 hdr_file_off;
	u32 hdr_size;
	u32 hdr_magic_off;
	u32 body_file_off;
	u32 body_size;
	u32 abi_version;
	u32 code_alloc_size;
	u32 data_alloc_size;
	u32 rodata_alloc_size;
	u64 body_carveout_off;
	u64 hdr_carveout_off;
	u64 bvnc;
};

static int pvr_laguna_detect_signed_hdr(const u8 *entry, u32 entry_len,
					u32 *hdr_size_out, u32 *magic_off_out)
{
	__le32 magic, version;

	if (entry_len < PVR_LAGUNA_SIGNED_FW_WHI_HDR_SIZE)
		return -EINVAL;

	memcpy(&magic, entry + PVR_LAGUNA_SIGNED_FW_WHI_MAGIC_OFF, sizeof(magic));
	if (le32_to_cpu(magic) == PVR_LAGUNA_SIGNED_FW_MAGIC) {
		*hdr_size_out = PVR_LAGUNA_SIGNED_FW_WHI_HDR_SIZE;
		*magic_off_out = PVR_LAGUNA_SIGNED_FW_WHI_MAGIC_OFF;
		return 0;
	}

	memcpy(&version, entry, sizeof(version));
	switch (le32_to_cpu(version)) {
	case PVR_LAGUNA_SIGNED_FW_LGA_VERSION:
		if (entry_len < PVR_LAGUNA_SIGNED_FW_LGA_HDR_SIZE)
			return -EINVAL;
		memcpy(&magic, entry + PVR_LAGUNA_SIGNED_FW_LGA_MAGIC_OFF,
		       sizeof(magic));
		if (le32_to_cpu(magic) != PVR_LAGUNA_SIGNED_FW_MAGIC)
			return -EINVAL;
		*hdr_size_out = PVR_LAGUNA_SIGNED_FW_LGA_HDR_SIZE;
		*magic_off_out = PVR_LAGUNA_SIGNED_FW_LGA_MAGIC_OFF;
		return 0;

	case PVR_LAGUNA_SIGNED_FW_MBU_VERSION:
		if (entry_len < PVR_LAGUNA_SIGNED_FW_MBU_HDR_SIZE)
			return -EINVAL;
		memcpy(&magic, entry + PVR_LAGUNA_SIGNED_FW_MBU_MAGIC_OFF,
		       sizeof(magic));
		if (le32_to_cpu(magic) != PVR_LAGUNA_SIGNED_FW_MAGIC)
			return -EINVAL;
		*hdr_size_out = PVR_LAGUNA_SIGNED_FW_MBU_HDR_SIZE;
		*magic_off_out = PVR_LAGUNA_SIGNED_FW_MBU_MAGIC_OFF;
		return 0;

	default:
		return -EINVAL;
	}
}

static int pvr_laguna_parse_fw_layout(const u8 *body, u32 body_size,
				      struct pvr_laguna_fw_parsed *out)
{
	struct pvr_laguna_fw_info_hdr info;
	u32 hdr_len, entry_num, entry_size;
	u64 code_sz = 0, data_sz = 0, rodata_sz = 0;
	u64 code_alloc, data_alloc, rodata_alloc;
	u64 body_off, hdr_off, end_off;
	const u8 *trailer;

	if (body_size <= PVR_LAGUNA_FW_BLOCK_SIZE)
		return -EINVAL;

	trailer = body + body_size - PVR_LAGUNA_FW_BLOCK_SIZE;
	memcpy(&info, trailer, sizeof(info));

	if (le32_to_cpu(info.info_version) == 0)
		return -EINVAL;

	hdr_len = le32_to_cpu(info.header_len);
	entry_num = le32_to_cpu(info.layout_entry_num);
	entry_size = le32_to_cpu(info.layout_entry_size);

	if (hdr_len < sizeof(info) || hdr_len > PVR_LAGUNA_FW_BLOCK_SIZE ||
	    entry_size < sizeof(struct pvr_laguna_fw_layout_entry) ||
	    entry_num == 0 || entry_num > PVR_LAGUNA_FW_LAYOUT_MAX_ENTRIES ||
	    (u64)hdr_len + (u64)entry_num * entry_size > PVR_LAGUNA_FW_BLOCK_SIZE)
		return -EINVAL;

	for (u32 i = 0; i < entry_num; i++) {
		struct pvr_laguna_fw_layout_entry ent;
		u32 alloc_sz;

		memcpy(&ent, trailer + hdr_len + i * entry_size, sizeof(ent));
		alloc_sz = le32_to_cpu(ent.alloc_size);
		if (!alloc_sz)
			continue;

		switch (le32_to_cpu(ent.type)) {
		case PVR_LAGUNA_FW_SEC_CODE:
			code_sz += alloc_sz;
			break;
		case PVR_LAGUNA_FW_SEC_RWDATA:
			data_sz += alloc_sz;
			break;
		case PVR_LAGUNA_FW_SEC_RODATA:
			rodata_sz += alloc_sz;
			break;
		case PVR_LAGUNA_FW_SEC_COREMEM_CODE:
		case PVR_LAGUNA_FW_SEC_COREMEM_DATA:
			/* Volcanic RISC-V has no META DMA coremem sections. */
			break;
		default:
			return -EINVAL;
		}
	}

	if (!code_sz || !data_sz)
		return -EINVAL;

	code_alloc = PAGE_ALIGN(code_sz);
	data_alloc = PAGE_ALIGN(data_sz);
	rodata_alloc = PAGE_ALIGN(rodata_sz);

	if (code_alloc > PVR_LAGUNA_FW_CODE_HEAP_SIZE ||
	    data_alloc > PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE ||
	    rodata_alloc > PVR_LAGUNA_FW_RODATA_HEAP_SIZE)
		return -EFBIG;

	/*
	 * Place image_body and image_header page-aligned inside FW_CODE_HEAP
	 * right after section[CODE] so GSA authentication pages never overlap
	 * any other firmware section.
	 */
	body_off = PVR_LAGUNA_FW_CODE_HEAP_OFFSET + code_alloc;
	hdr_off = body_off + PAGE_ALIGN((u64)body_size);
	end_off = hdr_off + PAGE_ALIGN((u64)out->hdr_size);
	if (end_off > PVR_LAGUNA_FW_CODE_HEAP_OFFSET + PVR_LAGUNA_FW_CODE_HEAP_SIZE)
		return -EFBIG;

	out->code_alloc_size = (u32)code_alloc;
	out->data_alloc_size = (u32)data_alloc;
	out->rodata_alloc_size = (u32)rodata_alloc;
	out->body_carveout_off = body_off;
	out->hdr_carveout_off = hdr_off;
	out->bvnc = le64_to_cpu(info.bvnc);

	return 0;
}

static int pvr_laguna_parse_signed_entry(const u8 *entry, u32 entry_len,
					 u32 entry_file_off, u32 abi_version,
					 struct pvr_laguna_fw_parsed *out)
{
	u32 hdr_size = 0, magic_off = 0;
	int err;

	err = pvr_laguna_detect_signed_hdr(entry, entry_len,
					   &hdr_size, &magic_off);
	if (err)
		return err;

	if (entry_len <= hdr_size)
		return -EINVAL;

	memset(out, 0, sizeof(*out));
	out->hdr_file_off = entry_file_off;
	out->hdr_size = hdr_size;
	out->hdr_magic_off = magic_off;
	out->body_file_off = entry_file_off + hdr_size;
	out->body_size = entry_len - hdr_size;
	out->abi_version = abi_version;

	return pvr_laguna_parse_fw_layout(entry + hdr_size,
					  entry_len - hdr_size, out);
}

static int pvr_laguna_validate_signed_fw(struct pvr_device *pvr_dev,
					 const struct firmware *fw,
					 struct pvr_laguna_fw_parsed *out)
{
	u64 target_bvnc = pvr_gpu_id_to_packed_bvnc(&pvr_dev->gpu_id);
	struct pvr_laguna_fw_parsed fallback = { 0 };
	bool have_fallback = false;
	size_t pos = 0;
	int err;

	if (!fw || !fw->data || fw->size == 0 || fw->size > SZ_8M)
		return -EINVAL;

	/*
	 * Production rgx.fw.signed is a catalogue of one or more length-prefixed
	 * signed firmware images (u32 LE entry_len followed by entry bytes with
	 * a 2 KiB Pixel footer at entry_end - 0x800).
	 */
	while (pos + sizeof(__le32) <= fw->size) {
		struct pvr_laguna_fw_footer_hdr footer;
		struct pvr_laguna_fw_parsed cand;
		const u8 *entry;
		__le32 raw_len;
		u32 entry_len;

		memcpy(&raw_len, fw->data + pos, sizeof(raw_len));
		entry_len = le32_to_cpu(raw_len);
		if (entry_len < PVR_LAGUNA_FW_FOOTER_OFF_FROM_END ||
		    !IS_ALIGNED(entry_len, sizeof(u32)) ||
		    entry_len > fw->size - (pos + sizeof(raw_len)))
			break;

		entry = fw->data + pos + sizeof(raw_len);
		memcpy(&footer,
		       entry + entry_len - PVR_LAGUNA_FW_FOOTER_OFF_FROM_END,
		       sizeof(footer));

		if (le32_to_cpu(footer.magic) != PVR_LAGUNA_FW_FOOTER_MAGIC ||
		    le32_to_cpu(footer.footer_version) != PVR_LAGUNA_FW_FOOTER_VERSION ||
		    le32_to_cpu(footer.pa_range_count) > PVR_LAGUNA_FW_FOOTER_MAX_PA_RANGES)
			break;

		err = pvr_laguna_parse_signed_entry(entry, entry_len,
						    (u32)(pos + sizeof(raw_len)),
						    le32_to_cpu(footer.fwabi_version),
						    &cand);
		if (!err) {
			if (cand.bvnc == target_bvnc) {
				*out = cand;
				return 0;
			}
			if (!have_fallback) {
				fallback = cand;
				have_fallback = true;
			}
		}

		pos += sizeof(raw_len) + entry_len;
	}

	if (have_fallback) {
		*out = fallback;
		return 0;
	}

	/* Fallback: standalone signed image without 4-byte catalogue prefix. */
	return pvr_laguna_parse_signed_entry(fw->data, (u32)fw->size, 0, 0, out);
}

static void pvr_laguna_release_firmware(void *data)
{
	struct pvr_device *pvr_dev = data;

	release_firmware(pvr_dev->fw_dev.firmware);
	pvr_dev->fw_dev.firmware = NULL;
}

/**
 * pvr_laguna_request_signed_firmware() - Load and validate runtime rgx.fw.signed
 * @pvr_dev: Target PowerVR device.
 *
 * Firmware is supplied offline at runtime under /lib/firmware/google/rgx.fw.signed
 * and is never embedded in kernel sources.
 *
 * Returns:
 *  * 0 on success, or negative errno on failure.
 */
int pvr_laguna_request_signed_firmware(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	struct pvr_laguna_fw_parsed parsed;
	const struct firmware *fw;
	int err;

	if (!laguna)
		return -ENODEV;

	err = request_firmware(&fw, PVR_LAGUNA_SIGNED_FW_NAME, drm_dev->dev);
	if (err) {
		drm_err(drm_dev, "failed to load signed firmware %s (err=%d)\n",
			PVR_LAGUNA_SIGNED_FW_NAME, err);
		return err;
	}

	err = pvr_laguna_validate_signed_fw(pvr_dev, fw, &parsed);
	if (err) {
		drm_err(drm_dev,
			"invalid signed firmware %s (size=%zu, err=%d)\n",
			PVR_LAGUNA_SIGNED_FW_NAME, fw->size, err);
		release_firmware(fw);
		return err;
	}

	laguna->fw_hdr_file_off = parsed.hdr_file_off;
	laguna->fw_hdr_size = parsed.hdr_size;
	laguna->fw_hdr_magic_off = parsed.hdr_magic_off;
	laguna->fw_body_file_off = parsed.body_file_off;
	laguna->fw_body_size = parsed.body_size;
	laguna->fw_abi_version = parsed.abi_version;
	laguna->fw_code_alloc_size = parsed.code_alloc_size;
	laguna->fw_data_alloc_size = parsed.data_alloc_size;
	laguna->fw_rodata_alloc_size = parsed.rodata_alloc_size;
	laguna->fw_body_carveout_off = parsed.body_carveout_off;
	laguna->fw_hdr_carveout_off = parsed.hdr_carveout_off;
	pvr_dev->fw_dev.firmware = fw;

	drm_info(drm_dev,
		 "validated signed Volcanic firmware %s (size=%zu, hdr=%#x@%#x, GPUF@%#x, body=%#x@%#x, abi=%#x, code=%#x, data=%#x, rodata=%#x)\n",
		 PVR_LAGUNA_SIGNED_FW_NAME, fw->size,
		 parsed.hdr_size, parsed.hdr_file_off,
		 parsed.hdr_file_off + parsed.hdr_magic_off,
		 parsed.body_size, parsed.body_file_off, parsed.abi_version,
		 parsed.code_alloc_size, parsed.data_alloc_size,
		 parsed.rodata_alloc_size);

	return devm_add_action_or_reset(drm_dev->dev,
					pvr_laguna_release_firmware, pvr_dev);
}

static int pvr_laguna_stage_signed_fw(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	const struct firmware *fw = pvr_dev->fw_dev.firmware;
	u8 *carveout;

	if (!laguna || !fw || !laguna->fw_body_size || !laguna->fw_hdr_size)
		return -EINVAL;

	if (laguna->fw_carveout_size < PVR_LAGUNA_SECURE_FW_MEM_SIZE ||
	    laguna->fw_hdr_carveout_off + PAGE_ALIGN((u64)laguna->fw_hdr_size) >
	    PVR_LAGUNA_SECURE_FW_MEM_SIZE)
		return -EFBIG;

	carveout = memremap(laguna->fw_carveout_phys,
			    PVR_LAGUNA_SECURE_FW_MEM_SIZE, MEMREMAP_WC);
	if (!carveout) {
		drm_err(drm_dev, "failed to remap gpu_fw secure carveout\n");
		return -ENOMEM;
	}

	memset(carveout, 0, PVR_LAGUNA_SECURE_FW_MEM_SIZE);
	memcpy(carveout + laguna->fw_body_carveout_off,
	       fw->data + laguna->fw_body_file_off, laguna->fw_body_size);
	memcpy(carveout + laguna->fw_hdr_carveout_off,
	       fw->data + laguna->fw_hdr_file_off, laguna->fw_hdr_size);

	/* Ensure staged firmware and zeroed sections reach DRAM before Trusty IPC. */
	wmb();
	memunmap(carveout);

	return 0;
}

static int pvr_laguna_trusty_send_fw(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	struct pvr_laguna_tz_req_fw req = { 0 };
	struct pvr_laguna_tz_rsp_base rsp = { 0 };
	u64 base = (u64)laguna->fw_carveout_phys;
	int ret;

	req.base.command = PVR_LAGUNA_GPU_SECURE_CMD_SEND_FW;
	req.base.version = PVR_LAGUNA_GPU_SECURE_FW_VER_25_2;

	req.image_body.pa = base + laguna->fw_body_carveout_off;
	req.image_body.fw_va = 0;
	req.image_body.size = laguna->fw_body_size;

	req.image_header.pa = base + laguna->fw_hdr_carveout_off;
	req.image_header.fw_va = 0;
	req.image_header.size = laguna->fw_hdr_size;

	req.section[PVR_LAGUNA_GPU_SECURE_SEC_CODE].pa =
		base + PVR_LAGUNA_FW_CODE_HEAP_OFFSET;
	req.section[PVR_LAGUNA_GPU_SECURE_SEC_CODE].fw_va = 0;
	req.section[PVR_LAGUNA_GPU_SECURE_SEC_CODE].size =
		laguna->fw_code_alloc_size;

	req.section[PVR_LAGUNA_GPU_SECURE_SEC_DATA].pa =
		base + PVR_LAGUNA_FW_PRIV_DATA_HEAP_OFFSET;
	req.section[PVR_LAGUNA_GPU_SECURE_SEC_DATA].fw_va = 0;
	req.section[PVR_LAGUNA_GPU_SECURE_SEC_DATA].size =
		laguna->fw_data_alloc_size;

	if (laguna->fw_rodata_alloc_size) {
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_RODATA].pa =
			base + PVR_LAGUNA_FW_RODATA_HEAP_OFFSET;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_RODATA].fw_va = 0;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_RODATA].size =
			laguna->fw_rodata_alloc_size;
	}

	req.use_tf_a = 1;
	req.host_page_shift = PAGE_SHIFT;

	ret = pvr_laguna_tz_xchg(&laguna->tz, &req, sizeof(req),
				 &rsp, sizeof(rsp));
	if (ret < 0) {
		drm_err(drm_dev, "Trusty %s SEND_FIRMWARE_IMAGE failed (%d)\n",
			PVR_LAGUNA_TRUSTY_GPU_PORT, ret);
		return ret;
	}

	if (ret != sizeof(rsp) || rsp.command != req.base.command ||
	    rsp.result != 0) {
		drm_err(drm_dev,
			"Trusty %s rejected FW (len=%d, cmd=%u, result=%u)\n",
			PVR_LAGUNA_TRUSTY_GPU_PORT, ret,
			rsp.command, rsp.result);
		return -EACCES;
	}

	laguna->fw_authenticated = true;
	drm_info(drm_dev, "Trusty %s authenticated Volcanic firmware\n",
		 PVR_LAGUNA_TRUSTY_GPU_PORT);
	return 0;
}

int pvr_laguna_fw_start(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	if (!laguna)
		return -ENODEV;

	/*
	 * Stage and authenticate once per boot: Trusty locks the 2 MiB secure
	 * carveout and invokes TF-A OEM_GPU_CONTROL_SETUP during
	 * SEND_FIRMWARE_IMAGE. Subsequent starts only invoke TF-A START.
	 */
	if (!laguna->fw_authenticated) {
		err = pvr_laguna_stage_signed_fw(pvr_dev);
		if (err)
			return err;

		err = pvr_laguna_trusty_send_fw(pvr_dev);
		if (err)
			return err;
	}

	err = pvr_laguna_smc_gpu_control(pvr_dev,
					 PVR_LAGUNA_OEM_GPU_CONTROL_START);
	if (err)
		return err;

	laguna->fw_running = true;
	return 0;
}

int pvr_laguna_fw_stop(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	if (!laguna || !laguna->fw_running)
		return 0;

	err = pvr_laguna_smc_gpu_control(pvr_dev,
					 PVR_LAGUNA_OEM_GPU_CONTROL_STOP);
	if (!err)
		laguna->fw_running = false;

	return err;
}

int pvr_laguna_fw_init(struct pvr_device *pvr_dev)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	int err;

	fw_dev->defs = &pvr_fw_defs_riscv;

	err = fw_dev->defs->init(pvr_dev);
	if (err)
		return err;

	err = pvr_laguna_fw_start(pvr_dev);
	if (err)
		return err;

	WRITE_ONCE(fw_dev->initialised, true);
	return 0;
}

void pvr_laguna_fw_fini(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	WRITE_ONCE(pvr_dev->fw_dev.initialised, false);

	if (!laguna)
		return;

	pvr_laguna_fw_stop(pvr_dev);
	pvr_laguna_tz_close(&laguna->tz);
}

static void __iomem *
pvr_laguna_ioremap_optional(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;

	return devm_ioremap_resource(&pdev->dev, res);
}

static int pvr_laguna_parse_fw_carveout(struct device *dev,
					struct pvr_laguna_data *laguna)
{
	struct device_node *rmem_np;
	struct resource rmem_res;
	resource_size_t size;
	int err;

	rmem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!rmem_np)
		return dev_err_probe(dev, -ENODEV,
				     "missing gpu_fw memory-region phandle\n");

	err = of_address_to_resource(rmem_np, 0, &rmem_res);
	of_node_put(rmem_np);
	if (err)
		return dev_err_probe(dev, err,
				     "failed to resolve gpu_fw reserved memory\n");

	size = resource_size(&rmem_res);
	if (size < PVR_LAGUNA_FW_CARVEOUT_MIN_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "gpu_fw carveout too small (%pap < %llu)\n",
				     &size, PVR_LAGUNA_FW_CARVEOUT_MIN_SIZE);

	laguna->fw_carveout_phys = rmem_res.start;
	laguna->fw_carveout_size = size;
	return 0;
}

static int pvr_laguna_power_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct device *dev = drm_dev->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct pvr_laguna_data *laguna;
	static const char *const ra_lut_names[PVR_LAGUNA_RA_LUT_BANKS] = {
		"ra_lut_0", "ra_lut_1", "ra_lut_2",
	};
	int err;

	laguna = devm_kzalloc(dev, sizeof(*laguna), GFP_KERNEL);
	if (!laguna)
		return -ENOMEM;

	laguna->core_freq_hz = PVR_LAGUNA_GPU_INIT_FREQ_HZ;
	laguna->pf_state_clk = pvr_dev->core_clk;

	laguna->pfsm = devm_platform_ioremap_resource_byname(pdev, "pfsm");
	if (IS_ERR(laguna->pfsm))
		return dev_err_probe(dev, PTR_ERR(laguna->pfsm),
				     "failed to map pfsm registers\n");

	laguna->req_pbha_lut =
		devm_platform_ioremap_resource_byname(pdev, "req_pbha_lut");
	if (IS_ERR(laguna->req_pbha_lut))
		return dev_err_probe(dev, PTR_ERR(laguna->req_pbha_lut),
				     "failed to map req_pbha_lut registers\n");

	for (u32 i = 0; i < PVR_LAGUNA_RA_LUT_BANKS; i++) {
		laguna->ra_lut[i] =
			devm_platform_ioremap_resource_byname(pdev, ra_lut_names[i]);
		if (IS_ERR(laguna->ra_lut[i]))
			return dev_err_probe(dev, PTR_ERR(laguna->ra_lut[i]),
					     "failed to map %s registers\n",
					     ra_lut_names[i]);
	}

	laguna->mba_glue =
		devm_platform_ioremap_resource_byname(pdev, "mba_glue");
	if (IS_ERR(laguna->mba_glue))
		return dev_err_probe(dev, PTR_ERR(laguna->mba_glue),
				     "failed to map mba_glue registers\n");

	laguna->mba_client =
		devm_platform_ioremap_resource_byname(pdev, "mba_client");
	if (IS_ERR(laguna->mba_client))
		return dev_err_probe(dev, PTR_ERR(laguna->mba_client),
				     "failed to map mba_client registers\n");

	laguna->gpu_control = pvr_laguna_ioremap_optional(pdev, "gpu_control");
	if (IS_ERR(laguna->gpu_control))
		return dev_err_probe(dev, PTR_ERR(laguna->gpu_control),
				     "failed to map gpu_control registers\n");

	laguna->pvtc = pvr_laguna_ioremap_optional(pdev, "pvtc");
	if (IS_ERR(laguna->pvtc))
		return dev_err_probe(dev, PTR_ERR(laguna->pvtc),
				     "failed to map pvtc registers\n");

	laguna->pvtc_pwroff = pvr_laguna_ioremap_optional(pdev, "pvtc_pwroff");
	if (IS_ERR(laguna->pvtc_pwroff))
		return dev_err_probe(dev, PTR_ERR(laguna->pvtc_pwroff),
				     "failed to map pvtc_pwroff registers\n");

	laguna->dts = pvr_laguna_ioremap_optional(pdev, "dts");
	if (IS_ERR(laguna->dts))
		return dev_err_probe(dev, PTR_ERR(laguna->dts),
				     "failed to map dts registers\n");

	err = pvr_laguna_parse_fw_carveout(dev, laguna);
	if (err)
		return err;

	pvr_laguna_tz_init(&laguna->tz, dev);
	pvr_dev->laguna = laguna;

	return 0;
}

static int pvr_laguna_power_on(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	if (!laguna)
		return -ENODEV;

	err = clk_prepare_enable(laguna->pf_state_clk);
	if (err)
		return err;

	err = pvr_laguna_core_logic_power_on(laguna);
	if (err) {
		clk_disable_unprepare(laguna->pf_state_clk);
		return err;
	}

	/* Read PFSM status to fence power-on before programming SLC/MBA. */
	(void)readl(laguna->pfsm + PVR_LAGUNA_PFSM_STATUS_OFF);

	/* Re-program OSID 1 SLC PBHA & read-allocate LUTs on every power-on. */
	pvr_laguna_slc_program_lut(pvr_dev);
	pvr_laguna_mba_clear_irq(laguna);

	return 0;
}

static int pvr_laguna_power_off(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna)
		return 0;

	pvr_laguna_mba_clear_irq(laguna);
	pvr_laguna_core_logic_power_off(laguna);
	clk_disable_unprepare(laguna->pf_state_clk);

	return 0;
}

const struct pvr_power_sequence_ops pvr_power_sequence_ops_laguna = {
	.init = pvr_laguna_power_init,
	.power_on = pvr_laguna_power_on,
	.power_off = pvr_laguna_power_off,
};
