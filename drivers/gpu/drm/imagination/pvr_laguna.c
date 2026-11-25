// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Clean-room Tensor G5 (Laguna) platform glue for PowerVR Volcanic
 * (DXT-48-1536, BVNC 71.2.2448.1212).
 */

#include "pvr_ccb.h"
#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_gem.h"
#include "pvr_laguna.h"
#include "pvr_power.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_fwif.h"
#include "pvr_rogue_fwif_common.h"
#include "pvr_rogue_heap_config.h"
#include "pvr_volcanic_defs.h"

#include <drm/drm_managed.h>
#include <drm/drm_mm.h>
#include <drm/drm_print.h>

#include <linux/align.h>
#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>

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
	u32 coremem_code_alloc_size;
	u32 coremem_data_alloc_size;
	u64 coremem_code_carveout_off;
	u64 coremem_data_carveout_off;
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
	u64 coremem_code_sz = 0, coremem_data_sz = 0;
	u64 code_alloc, data_alloc, rodata_alloc;
	u64 coremem_code_alloc, coremem_data_alloc;
	u64 coremem_code_off, coremem_data_off;
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
			coremem_code_sz += alloc_sz;
			break;
		case PVR_LAGUNA_FW_SEC_COREMEM_DATA:
			coremem_data_sz += alloc_sz;
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
	coremem_code_alloc = PAGE_ALIGN(coremem_code_sz);
	coremem_data_alloc = PAGE_ALIGN(coremem_data_sz);

	if (code_alloc + coremem_code_alloc > PVR_LAGUNA_FW_CODE_HEAP_SIZE ||
	    data_alloc + coremem_data_alloc > PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE ||
	    rodata_alloc > PVR_LAGUNA_FW_RODATA_HEAP_SIZE)
		return -EFBIG;

	/*
	 * Place COREMEM_CODE right after CODE in FW_CODE_HEAP, COREMEM_DATA
	 * right after DATA in FW_PRIV_DATA_HEAP, and stage image_body and
	 * image_header page-aligned after COREMEM_CODE in FW_CODE_HEAP so GSA
	 * authentication pages never overlap any firmware section.
	 */
	coremem_code_off = PVR_LAGUNA_FW_CODE_HEAP_OFFSET + code_alloc;
	coremem_data_off = PVR_LAGUNA_FW_PRIV_DATA_HEAP_OFFSET + data_alloc;
	body_off = coremem_code_off + coremem_code_alloc;
	hdr_off = body_off + PAGE_ALIGN((u64)body_size);
	end_off = hdr_off + PAGE_ALIGN((u64)out->hdr_size);
	if (end_off > PVR_LAGUNA_FW_CODE_HEAP_OFFSET + PVR_LAGUNA_FW_CODE_HEAP_SIZE)
		return -EFBIG;

	out->code_alloc_size = (u32)code_alloc;
	out->data_alloc_size = (u32)data_alloc;
	out->rodata_alloc_size = (u32)rodata_alloc;
	out->coremem_code_alloc_size = (u32)coremem_code_alloc;
	out->coremem_data_alloc_size = (u32)coremem_data_alloc;
	out->coremem_code_carveout_off = coremem_code_off;
	out->coremem_data_carveout_off = coremem_data_off;
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
	laguna->fw_coremem_code_alloc_size = parsed.coremem_code_alloc_size;
	laguna->fw_coremem_data_alloc_size = parsed.coremem_data_alloc_size;
	laguna->fw_coremem_code_carveout_off = parsed.coremem_code_carveout_off;
	laguna->fw_coremem_data_carveout_off = parsed.coremem_data_carveout_off;
	laguna->fw_body_carveout_off = parsed.body_carveout_off;
	laguna->fw_hdr_carveout_off = parsed.hdr_carveout_off;
	pvr_dev->fw_dev.firmware = fw;

	drm_info(drm_dev,
		 "validated signed Volcanic firmware %s (size=%zu, hdr=%#x@%#x, GPUF@%#x, body=%#x@%#x, abi=%#x, code=%#x, data=%#x, rodata=%#x, cm_code=%#x, cm_data=%#x)\n",
		 PVR_LAGUNA_SIGNED_FW_NAME, fw->size,
		 parsed.hdr_size, parsed.hdr_file_off,
		 parsed.hdr_file_off + parsed.hdr_magic_off,
		 parsed.body_size, parsed.body_file_off, parsed.abi_version,
		 parsed.code_alloc_size, parsed.data_alloc_size,
		 parsed.rodata_alloc_size, parsed.coremem_code_alloc_size,
		 parsed.coremem_data_alloc_size);

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

	if (laguna->fw_coremem_code_alloc_size) {
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_CODE].pa =
			base + laguna->fw_coremem_code_carveout_off;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_CODE].fw_va =
			PVR_LAGUNA_RISCV_FW_HEAP_BASE +
			(u32)laguna->fw_coremem_code_carveout_off;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_CODE].size =
			laguna->fw_coremem_code_alloc_size;
	}

	if (laguna->fw_coremem_data_alloc_size) {
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_DATA].pa =
			base + laguna->fw_coremem_data_carveout_off;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_DATA].fw_va =
			PVR_LAGUNA_RISCV_FW_HEAP_BASE +
			(u32)laguna->fw_coremem_data_carveout_off;
		req.section[PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_DATA].size =
			laguna->fw_coremem_data_alloc_size;
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

	if (!laguna->fw_running)
		drm_info(from_pvr_device(pvr_dev),
			 "TF-A SMC_OEM_GPU_CONTROL START succeeded\n");

	laguna->fw_running = true;
	return 0;
}

#define PVR_LAGUNA_FW_BOOT_TIMEOUT_US		500000U
#define PVR_LAGUNA_FW_HEALTH_TIMEOUT_US		100000U
#define PVR_LAGUNA_HCS_DEFAULT_DEADLINE_MS	5000U
#define PVR_LAGUNA_FAULT_PAGE_SIZE		SZ_4K
#define PVR_LAGUNA_FAULT_PAGE_POISON		0xdeadbee0U

void *pvr_laguna_fw_shared_vaddr(struct pvr_device *pvr_dev, u32 fw_offset)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna || !laguna->fw_shared_cpu ||
	    fw_offset < PVR_LAGUNA_FW_SHARED_HEAP_OFFSET ||
	    fw_offset >= PVR_LAGUNA_FW_SHARED_HEAP_OFFSET +
			 PVR_LAGUNA_FW_SHARED_HEAP_SIZE)
		return NULL;

	return (u8 *)laguna->fw_shared_cpu +
	       (fw_offset - PVR_LAGUNA_FW_SHARED_HEAP_OFFSET);
}

phys_addr_t pvr_laguna_fw_shared_paddr(struct pvr_device *pvr_dev,
				       u32 fw_offset)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna ||
	    fw_offset < PVR_LAGUNA_FW_SHARED_HEAP_OFFSET ||
	    fw_offset >= PVR_LAGUNA_FW_SHARED_HEAP_OFFSET +
			 PVR_LAGUNA_FW_SHARED_HEAP_SIZE)
		return 0;

	return laguna->fw_carveout_phys + fw_offset;
}

bool pvr_laguna_kccb_ready(const struct pvr_device *pvr_dev)
{
	return pvr_dev->laguna && READ_ONCE(pvr_dev->laguna->fw_kccb_ready);
}

bool pvr_laguna_power_is_idle(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna || !READ_ONCE(laguna->fw_kccb_ready) || !laguna->fwif_sysdata)
		return true;

	return READ_ONCE(laguna->fwif_sysdata->pow_state) == ROGUE_FWIF_POW_IDLE &&
	       pvr_laguna_kccb_is_idle(pvr_dev);
}

void pvr_laguna_clear_hwr_flags(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna || !laguna->fwif_sysdata)
		return;

	laguna->fwif_sysdata->hwr_state_flags &=
		~(ROGUE_FWIF_HWR_FW_FAULT | ROGUE_FWIF_HWR_RESTART_REQUESTED);
	wmb(); /* Ensure cleared HWR flags reach WC carveout before FW restart. */
}

u32 pvr_laguna_kccb_used_slot_count_locked(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	u32 wr_offset, rd_offset;

	lockdep_assert_held(&pvr_dev->kccb.ccb.lock);

	if (!laguna || !laguna->kccb_ctl)
		return 0;

	wr_offset = READ_ONCE(laguna->kccb_ctl->write_offset);
	rd_offset = READ_ONCE(laguna->kccb_ctl->read_offset);

	if (wr_offset >= PVR_LAGUNA_KCCB_NUMCMDS ||
	    rd_offset >= PVR_LAGUNA_KCCB_NUMCMDS) {
		drm_err_once(from_pvr_device(pvr_dev),
			     "Corrupt Laguna KCCB indices (rd=%u, wr=%u)\n",
			     rd_offset, wr_offset);
		return pvr_dev->kccb.slot_count - 1U;
	}

	if (wr_offset >= rd_offset)
		return wr_offset - rd_offset;

	return wr_offset + pvr_dev->kccb.slot_count - rd_offset;
}

bool pvr_laguna_kccb_is_idle(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	bool idle;

	if (!laguna || !laguna->kccb_ctl)
		return true;

	mutex_lock(&pvr_dev->kccb.ccb.lock);
	idle = READ_ONCE(laguna->kccb_ctl->write_offset) ==
	       READ_ONCE(laguna->kccb_ctl->read_offset);
	mutex_unlock(&pvr_dev->kccb.ccb.lock);

	return idle;
}

static int pvr_laguna_translate_kccb_cmd(const struct rogue_fwif_kccb_cmd *in,
					 struct pvr_laguna_fwif_kccb_cmd *out)
{
	memset(out, 0, sizeof(*out));
	out->kccb_flags = in->kccb_flags;

	switch (in->cmd_type) {
	case ROGUE_FWIF_KCCB_CMD_KICK: {
		const struct rogue_fwif_kccb_cmd_kick_data *src =
			&in->cmd_data.cmd_kick_data;
		u32 count = min_t(u32, src->num_cleanup_ctl,
				  ARRAY_SIZE(out->data.kick.cleanup_ctl));

		out->cmd_type = PVR_LAGUNA_KCCB_CMD_KICK;
		out->data.kick.context_fw_addr = src->context_fw_addr;
		out->data.kick.cwoff_update = src->client_woff_update;
		out->data.kick.cwrap_mask_update = src->client_wrap_mask_update;
		out->data.kick.num_cleanup_ctl = count;
		for (u32 i = 0; i < count; i++)
			out->data.kick.cleanup_ctl[i] = src->cleanup_ctl_fw_addr[i];
		return 0;
	}

	case ROGUE_FWIF_KCCB_CMD_MMUCACHE:
		out->cmd_type = PVR_LAGUNA_KCCB_CMD_MMUCACHE;
		out->data.mmucache.cache_flags = in->cmd_data.mmu_cache_data.cache_flags;
		out->data.mmucache.mmu_cache_sync_fw_addr =
			in->cmd_data.mmu_cache_data.mmu_cache_sync_fw_addr;
		out->data.mmucache.mmu_cache_sync_update_value =
			in->cmd_data.mmu_cache_data.mmu_cache_sync_update_value;
		return 0;

	case ROGUE_FWIF_KCCB_CMD_CLEANUP:
		out->cmd_type = PVR_LAGUNA_KCCB_CMD_CLEANUP;
		switch (in->cmd_data.cleanup_data.cleanup_type) {
		case ROGUE_FWIF_CLEANUP_FWCOMMONCONTEXT:
			out->data.cleanup.cleanup_type =
				PVR_LAGUNA_CLEANUP_FWCOMMONCONTEXT;
			out->data.cleanup.resource_fw_addr =
				in->cmd_data.cleanup_data.cleanup_data.context_fw_addr;
			return 0;
		case ROGUE_FWIF_CLEANUP_HWRTDATA:
			out->data.cleanup.cleanup_type = PVR_LAGUNA_CLEANUP_HWRTDATA;
			out->data.cleanup.resource_fw_addr =
				in->cmd_data.cleanup_data.cleanup_data.hwrt_data_fw_addr;
			return 0;
		case ROGUE_FWIF_CLEANUP_FREELIST:
			out->data.cleanup.cleanup_type = PVR_LAGUNA_CLEANUP_FREELIST;
			out->data.cleanup.resource_fw_addr =
				in->cmd_data.cleanup_data.cleanup_data.freelist_fw_addr;
			return 0;
		default:
			return -EINVAL;
		}

	case ROGUE_FWIF_KCCB_CMD_HEALTH_CHECK:
		out->cmd_type = PVR_LAGUNA_KCCB_CMD_HEALTH_CHECK;
		return 0;

	case ROGUE_FWIF_KCCB_CMD_POW:
		out->cmd_type = PVR_LAGUNA_KCCB_CMD_POW;
		out->data.pow.pow_type = in->cmd_data.pow_data.pow_type;
		switch (in->cmd_data.pow_data.pow_type) {
		case ROGUE_FWIF_POW_OFF_REQ:
			out->data.pow.arg0 =
				in->cmd_data.pow_data.power_req_data.forced ? 1U : 0U;
			return 0;
		case ROGUE_FWIF_POW_FORCED_IDLE_REQ:
			out->data.pow.arg0 =
				in->cmd_data.pow_data.power_req_data.pow_request_type;
			return 0;
		case ROGUE_FWIF_POW_NUM_UNITS_CHANGE:
			out->data.pow.arg0 =
				in->cmd_data.pow_data.power_req_data.num_of_dusts;
			return 0;
		default:
			return -EINVAL;
		}

	default:
		return -EOPNOTSUPP;
	}
}

void pvr_laguna_kccb_send_cmd_reserved_powered(struct pvr_device *pvr_dev,
					       struct rogue_fwif_kccb_cmd *cmd,
					       u32 *kccb_slot)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	struct pvr_laguna_fwif_ccb_ctl *ctrl;
	struct pvr_laguna_fwif_kccb_cmd fw_cmd;
	u32 old_woff, new_woff, rd_offset;
	int err;

	WARN_ON(pvr_dev->lost);

	mutex_lock(&pvr_dev->kccb.ccb.lock);

	if (WARN_ON(!pvr_dev->kccb.reserved_count))
		goto out_unlock;

	if (!laguna || !laguna->kccb_ctl || !laguna->kccb || !laguna->kccb_rtn) {
		pvr_dev->kccb.reserved_count--;
		goto out_unlock;
	}

	ctrl = laguna->kccb_ctl;
	old_woff = READ_ONCE(ctrl->write_offset);
	rd_offset = READ_ONCE(ctrl->read_offset);
	if (old_woff >= PVR_LAGUNA_KCCB_NUMCMDS ||
	    rd_offset >= PVR_LAGUNA_KCCB_NUMCMDS) {
		drm_err_once(from_pvr_device(pvr_dev),
			     "Corrupt Laguna KCCB indices on send (rd=%u, wr=%u)\n",
			     rd_offset, old_woff);
		if (kccb_slot)
			*kccb_slot = 0;
		pvr_dev->kccb.reserved_count--;
		goto out_unlock;
	}

	new_woff = (old_woff + 1U) & (PVR_LAGUNA_KCCB_NUMCMDS - 1U);

	if (WARN_ON(rd_offset == new_woff)) {
		pvr_dev->kccb.reserved_count--;
		goto out_unlock;
	}

	err = pvr_laguna_translate_kccb_cmd(cmd, &fw_cmd);
	if (err) {
		drm_warn_once(from_pvr_device(pvr_dev),
			      "Unsupported Laguna KCCB cmd_type=%#x (err=%d)\n",
			      cmd->cmd_type, err);
		if (kccb_slot) {
			*kccb_slot = old_woff;
			WRITE_ONCE(laguna->kccb_rtn[old_woff],
				   PVR_LAGUNA_KCCB_RTN_CMD_EXECUTED);
		}
		pvr_dev->kccb.reserved_count--;
		goto out_unlock;
	}

	memcpy(&laguna->kccb[old_woff], &fw_cmd, sizeof(fw_cmd));
	if (kccb_slot) {
		*kccb_slot = old_woff;
		WRITE_ONCE(laguna->kccb_rtn[old_woff],
			   PVR_LAGUNA_KCCB_RTN_NO_RESPONSE);
	}
	/* Ensure command payload and return slot reset reach DRAM before woff. */
	wmb();
	WRITE_ONCE(ctrl->write_offset, new_woff);
	pvr_dev->kccb.reserved_count--;

	pvr_fw_mts_schedule(pvr_dev,
			    PVR_FWIF_DM_GP & ~ROGUE_CR_MTS_SCHEDULE_DM_CLRMSK);

out_unlock:
	mutex_unlock(&pvr_dev->kccb.ccb.lock);
}

void pvr_laguna_fwccb_process(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	struct pvr_laguna_fwif_ccb_ctl *ctrl;

	if (!laguna || !laguna->fwccb_ctl || !laguna->fwccb)
		return;

	ctrl = laguna->fwccb_ctl;
	mutex_lock(&pvr_dev->fwccb.lock);

	for (u32 processed = 0; processed < PVR_LAGUNA_FWCCB_NUMCMDS; processed++) {
		u32 rd_offset = READ_ONCE(ctrl->read_offset);
		u32 wr_offset = READ_ONCE(ctrl->write_offset);
		struct pvr_laguna_fwif_fwccb_cmd cmd;

		if (rd_offset == wr_offset)
			break;

		if (rd_offset >= PVR_LAGUNA_FWCCB_NUMCMDS ||
		    wr_offset >= PVR_LAGUNA_FWCCB_NUMCMDS) {
			drm_err_once(from_pvr_device(pvr_dev),
				     "Corrupt Laguna FWCCB indices (rd=%u, wr=%u)\n",
				     rd_offset, wr_offset);
			break;
		}

		cmd = laguna->fwccb[rd_offset];

		WRITE_ONCE(ctrl->read_offset,
			   (rd_offset + 1U) & (PVR_LAGUNA_FWCCB_NUMCMDS - 1U));
		wmb(); /* Publish updated FWCCB read offset to FW. */

		mutex_unlock(&pvr_dev->fwccb.lock);

		drm_dbg(from_pvr_device(pvr_dev),
			"Laguna FWCCB command received (type=%#x, flags=%#x)\n",
			cmd.cmd_type, cmd.fwccb_flags);

		mutex_lock(&pvr_dev->fwccb.lock);
	}

	mutex_unlock(&pvr_dev->fwccb.lock);
}

static int pvr_laguna_fw_defs_init(struct pvr_device *pvr_dev)
{
	pvr_fw_heap_info_init(pvr_dev, PVR_LAGUNA_FW_HEAP_SHIFT, 0);
	return 0;
}

static int pvr_laguna_fw_vm_map(struct pvr_device *pvr_dev,
				struct pvr_fw_object *fw_obj)
{
	u64 off = fw_obj->fw_addr_offset;
	u64 end = off + fw_obj->carveout_size;

	if (off < PVR_LAGUNA_FW_SHARED_HEAP_OFFSET ||
	    end > PVR_LAGUNA_FW_SHARED_HEAP_OFFSET +
		  PVR_LAGUNA_FW_SHARED_HEAP_SIZE)
		return -ENOSPC;

	fw_obj->carveout_cpu = pvr_laguna_fw_shared_vaddr(pvr_dev, (u32)off);
	fw_obj->carveout_phys = pvr_laguna_fw_shared_paddr(pvr_dev, (u32)off);
	if (!fw_obj->carveout_cpu || !fw_obj->carveout_phys)
		return -ENOMEM;

	memset(fw_obj->carveout_cpu, 0, fw_obj->carveout_size);
	return 0;
}

static void pvr_laguna_fw_vm_unmap(struct pvr_device *pvr_dev,
				   struct pvr_fw_object *fw_obj)
{
	fw_obj->carveout_cpu = NULL;
	fw_obj->carveout_phys = 0;
}

static u32 pvr_laguna_get_fw_addr_with_offset(struct pvr_fw_object *fw_obj,
					      u32 offset)
{
	u32 fw_addr = fw_obj->fw_addr_offset + offset;

	if (fw_obj->carveout_flags & PVR_BO_FW_FLAGS_DEVICE_UNCACHED)
		fw_addr |= VOLCANIC_RISCVFW_SHARED_UNCACHED_DATA0_BASE;
	else
		fw_addr |= VOLCANIC_RISCVFW_SHARED_CACHED_DATA0_BASE;

	return fw_addr;
}

static bool pvr_laguna_irq_pending(struct pvr_device *pvr_dev)
{
	return pvr_cr_read32(pvr_dev, VOLCANIC_CR_IRQ_OS0_EVENT_STATUS) &
	       VOLCANIC_CR_IRQ_OS0_EVENT_STATUS_SOURCE_EN;
}

static void pvr_laguna_irq_clear(struct pvr_device *pvr_dev)
{
	pvr_cr_write32(pvr_dev, VOLCANIC_CR_IRQ_OS0_EVENT_CLEAR,
		       VOLCANIC_CR_IRQ_OS0_EVENT_CLEAR_SOURCE_EN);
}

static const struct pvr_fw_defs pvr_fw_defs_laguna = {
	.init = pvr_laguna_fw_defs_init,
	.vm_map = pvr_laguna_fw_vm_map,
	.vm_unmap = pvr_laguna_fw_vm_unmap,
	.get_fw_addr_with_offset = pvr_laguna_get_fw_addr_with_offset,
	.irq_pending = pvr_laguna_irq_pending,
	.irq_clear = pvr_laguna_irq_clear,
	.has_fixed_data_addr = false,
};

static void pvr_laguna_fault_page_init(void *cpu_ptr, void *priv)
{
	u32 *page = cpu_ptr;

	for (u32 i = 0; i < PVR_LAGUNA_FAULT_PAGE_SIZE / sizeof(*page); i++)
		page[i] = PVR_LAGUNA_FAULT_PAGE_POISON;
}

static void pvr_laguna_kccb_ctl_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_ccb_ctl *ctrl = cpu_ptr;

	ctrl->write_offset = 0;
	ctrl->read_offset = 0;
	ctrl->wrap_mask = PVR_LAGUNA_KCCB_NUMCMDS - 1U;
	ctrl->reserved = 0;
}

static void pvr_laguna_fwccb_ctl_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_ccb_ctl *ctrl = cpu_ptr;

	ctrl->write_offset = 0;
	ctrl->read_offset = 0;
	ctrl->wrap_mask = PVR_LAGUNA_FWCCB_NUMCMDS - 1U;
	ctrl->reserved = 0;
}

static void pvr_laguna_osdata_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_osdata *osdata = cpu_ptr;
	struct pvr_device *pvr_dev = priv;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	pvr_fw_object_get_fw_addr(laguna->fwif_power_sync_obj,
				  &osdata->power_sync);
}

static void pvr_laguna_runtime_cfg_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_runtime_cfg *cfg = cpu_ptr;
	struct pvr_device *pvr_dev = priv;

	cfg->active_pm_latency_ms = 0;
	cfg->active_pm_latency_persistant = 1;
	cfg->core_clock_speed = (u32)pvr_laguna_get_core_clk_rate(pvr_dev);
	cfg->soc_clock_speed = PVR_LAGUNA_GTC_FREQ_HZ;
	cfg->pow_units_state = PVR_LAGUNA_POW_UNITS_MASK;
	cfg->rac_units_state = PVR_LAGUNA_RAC_UNITS_MASK;
	cfg->hcs_deadline_ms = PVR_LAGUNA_HCS_DEFAULT_DEADLINE_MS;
	cfg->wdg_period_us = 0;
}

static void pvr_laguna_tracebuf_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_tracebuf *tb = cpu_ptr;
	struct pvr_device *pvr_dev = priv;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	tb->log_type = 0;
	pvr_fw_object_get_fw_addr(laguna->fwif_trace_data_obj,
				  &tb->tracebuf.trace_buffer_fw_addr);
	tb->tracebuf_size_in_dwords = PVR_LAGUNA_FWIF_TRACE_BUF_DWORDS;
	tb->tracebuf_flags = 0;
}

static void pvr_laguna_osinit_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_osinit *osinit = cpu_ptr;
	struct pvr_device *pvr_dev = priv;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	pvr_fw_object_get_fw_addr(laguna->kccb_ctl_obj, &osinit->kernel_ccb_ctl);
	pvr_fw_object_get_fw_addr(laguna->kccb_obj, &osinit->kernel_ccb);
	pvr_fw_object_get_fw_addr(laguna->kccb_rtn_obj,
				  &osinit->kernel_ccb_rtn_slots);
	pvr_fw_object_get_fw_addr(laguna->fwccb_ctl_obj,
				  &osinit->firmware_ccb_ctl);
	pvr_fw_object_get_fw_addr(laguna->fwccb_obj, &osinit->firmware_ccb);
	pvr_fw_object_get_fw_addr(laguna->fwif_hwrinfobuf_obj,
				  &osinit->hwr_info_buf_ctl);
	pvr_fw_object_get_fw_addr(laguna->fwif_osdata_obj, &osinit->fw_os_data);
	pvr_fw_object_get_fw_addr(laguna->fwif_hwperf_data_obj,
				  &osinit->hwperf_data);
	pvr_fw_object_get_fw_addr(laguna->fwif_hwperf_rtcfg_obj,
				  &osinit->hwperf_runtime_cfg);
	pvr_fw_object_get_fw_addr(laguna->fwif_gpu_util_obj,
				  &osinit->gpu_util_fw_ctl);
	pvr_fw_object_get_fw_addr(laguna->fwif_hwperf_ctl_obj,
				  &osinit->hwperf_ctl);
}

static void pvr_laguna_sysinit_init(void *cpu_ptr, void *priv)
{
	struct pvr_laguna_fwif_sysinit *sysinit = cpu_ptr;
	struct pvr_device *pvr_dev = priv;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	dma_addr_t fault_pa = 0;

	WARN_ON(pvr_fw_object_get_dma_addr(laguna->fwif_fault_page_obj, 0,
					   &fault_pa));
	sysinit->fault_phys_addr = (u64)fault_pa;
	sysinit->pds_exec_base = PVR_LAGUNA_PDS_EXEC_BASE;
	sysinit->usc_exec_base = PVR_LAGUNA_USC_EXEC_BASE;
	sysinit->fbcdc_state_table_base = PVR_LAGUNA_FBCDC_STATE_BASE;
	sysinit->fbcdc_large_state_table_base = PVR_LAGUNA_FBCDC_LARGE_STATE_BASE;
	sysinit->texture_heap_base = PVR_LAGUNA_TEXTURE_STATE_BASE;
	sysinit->pds_indirect_heap_base = PVR_LAGUNA_PDS_INDIRECT_BASE;

	pvr_fw_object_get_fw_addr(laguna->fwif_runtime_cfg_obj,
				  &sysinit->runtime_cfg);
	pvr_fw_object_get_fw_addr(laguna->fwif_tracebuf_obj,
				  &sysinit->trace_buf_ctl);
	pvr_fw_object_get_fw_addr(laguna->fwif_sysdata_obj,
				  &sysinit->fw_sys_data);

	sysinit->align_checks = 0;
	sysinit->initial_core_clock_speed =
		(u32)pvr_laguna_get_core_clk_rate(pvr_dev);
	sysinit->initial_soc_clock_speed = PVR_LAGUNA_GTC_FREQ_HZ;
	sysinit->initial_active_pm_latency_ms = 0;
	sysinit->firmware_started = 0;
	sysinit->marker_val = 1;
	sysinit->firmware_perf = FW_PERF_CONF_NONE;
	sysinit->gpio_validation_mode = ROGUE_FWIF_GPIO_VAL_OFF;
}

static void pvr_laguna_fwif_destroy_structures(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	if (!laguna)
		return;

	pvr_dev->fw_dev.power_sync = NULL;
	pvr_dev->fw_dev.mem.mmucache_sync_obj = NULL;
	pvr_dev->kccb.rtn = NULL;

	if (laguna->fwif_sysinit_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_sysinit_obj);
		laguna->fwif_sysinit_obj = NULL;
		laguna->fwif_sysinit = NULL;
	}
	if (laguna->fwif_osinit_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_osinit_obj);
		laguna->fwif_osinit_obj = NULL;
		laguna->fwif_osinit = NULL;
	}
	if (laguna->fwif_connection_ctl_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_connection_ctl_obj);
		laguna->fwif_connection_ctl_obj = NULL;
		laguna->fwif_connection_ctl = NULL;
	}
	if (laguna->fwccb_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwccb_obj);
		laguna->fwccb_obj = NULL;
		laguna->fwccb = NULL;
	}
	if (laguna->fwccb_ctl_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwccb_ctl_obj);
		laguna->fwccb_ctl_obj = NULL;
		laguna->fwccb_ctl = NULL;
	}
	if (laguna->kccb_rtn_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->kccb_rtn_obj);
		laguna->kccb_rtn_obj = NULL;
		laguna->kccb_rtn = NULL;
	}
	if (laguna->kccb_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->kccb_obj);
		laguna->kccb_obj = NULL;
		laguna->kccb = NULL;
	}
	if (laguna->kccb_ctl_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->kccb_ctl_obj);
		laguna->kccb_ctl_obj = NULL;
		laguna->kccb_ctl = NULL;
	}
	if (laguna->fwif_tracebuf_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_tracebuf_obj);
		laguna->fwif_tracebuf_obj = NULL;
		laguna->fwif_tracebuf = NULL;
	}
	if (laguna->fwif_trace_data_obj) {
		pvr_fw_object_destroy(laguna->fwif_trace_data_obj);
		laguna->fwif_trace_data_obj = NULL;
	}
	if (laguna->fwif_runtime_cfg_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_runtime_cfg_obj);
		laguna->fwif_runtime_cfg_obj = NULL;
		laguna->fwif_runtime_cfg = NULL;
	}
	if (laguna->fwif_sysdata_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_sysdata_obj);
		laguna->fwif_sysdata_obj = NULL;
		laguna->fwif_sysdata = NULL;
	}
	if (laguna->fwif_osdata_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_osdata_obj);
		laguna->fwif_osdata_obj = NULL;
		laguna->fwif_osdata = NULL;
	}
	if (laguna->fwif_hwperf_ctl_obj) {
		pvr_fw_object_destroy(laguna->fwif_hwperf_ctl_obj);
		laguna->fwif_hwperf_ctl_obj = NULL;
	}
	if (laguna->fwif_hwperf_rtcfg_obj) {
		pvr_fw_object_destroy(laguna->fwif_hwperf_rtcfg_obj);
		laguna->fwif_hwperf_rtcfg_obj = NULL;
	}
	if (laguna->fwif_hwperf_data_obj) {
		pvr_fw_object_destroy(laguna->fwif_hwperf_data_obj);
		laguna->fwif_hwperf_data_obj = NULL;
	}
	if (laguna->fwif_gpu_util_obj) {
		pvr_fw_object_destroy(laguna->fwif_gpu_util_obj);
		laguna->fwif_gpu_util_obj = NULL;
	}
	if (laguna->fwif_hwrinfobuf_obj) {
		pvr_fw_object_destroy(laguna->fwif_hwrinfobuf_obj);
		laguna->fwif_hwrinfobuf_obj = NULL;
	}
	if (laguna->fwif_fault_page_obj) {
		pvr_fw_object_destroy(laguna->fwif_fault_page_obj);
		laguna->fwif_fault_page_obj = NULL;
	}
	if (laguna->fwif_mmucache_sync_obj) {
		pvr_fw_object_destroy(laguna->fwif_mmucache_sync_obj);
		laguna->fwif_mmucache_sync_obj = NULL;
	}
	if (laguna->fwif_power_sync_obj) {
		pvr_fw_object_unmap_and_destroy(laguna->fwif_power_sync_obj);
		laguna->fwif_power_sync_obj = NULL;
		laguna->power_sync = NULL;
	}
}

static int pvr_laguna_fwif_create_structures(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	laguna->power_sync = pvr_fw_object_create_and_map(pvr_dev, sizeof(u32),
							  PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							  NULL, NULL,
							  &laguna->fwif_power_sync_obj);
	if (IS_ERR(laguna->power_sync)) {
		err = PTR_ERR(laguna->power_sync);
		laguna->power_sync = NULL;
		goto err_destroy;
	}

	err = pvr_fw_object_create(pvr_dev, sizeof(u32),
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_mmucache_sync_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FAULT_PAGE_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
				   pvr_laguna_fault_page_init, NULL,
				   &laguna->fwif_fault_page_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FWIF_HWRINFOBUF_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_hwrinfobuf_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FWIF_GPU_UTIL_FW_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_gpu_util_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FWIF_HWPERFDATA_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_hwperf_data_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FWIF_RUNTIME_HWPERFCFG_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_hwperf_rtcfg_obj);
	if (err)
		goto err_destroy;

	err = pvr_fw_object_create(pvr_dev, PVR_LAGUNA_FWIF_HWPERF_CTL_SIZE,
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_hwperf_ctl_obj);
	if (err)
		goto err_destroy;

	laguna->fwif_osdata = pvr_fw_object_create_and_map(pvr_dev,
							   sizeof(*laguna->fwif_osdata),
							   PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							   pvr_laguna_osdata_init, pvr_dev,
							   &laguna->fwif_osdata_obj);
	if (IS_ERR(laguna->fwif_osdata)) {
		err = PTR_ERR(laguna->fwif_osdata);
		laguna->fwif_osdata = NULL;
		goto err_destroy;
	}

	laguna->fwif_sysdata = pvr_fw_object_create_and_map(pvr_dev,
							    sizeof(*laguna->fwif_sysdata),
							    PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							    NULL, NULL,
							    &laguna->fwif_sysdata_obj);
	if (IS_ERR(laguna->fwif_sysdata)) {
		err = PTR_ERR(laguna->fwif_sysdata);
		laguna->fwif_sysdata = NULL;
		goto err_destroy;
	}

	laguna->fwif_runtime_cfg = pvr_fw_object_create_and_map(pvr_dev,
								sizeof(*laguna->fwif_runtime_cfg),
								PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
								pvr_laguna_runtime_cfg_init,
								pvr_dev,
								&laguna->fwif_runtime_cfg_obj);
	if (IS_ERR(laguna->fwif_runtime_cfg)) {
		err = PTR_ERR(laguna->fwif_runtime_cfg);
		laguna->fwif_runtime_cfg = NULL;
		goto err_destroy;
	}

	err = pvr_fw_object_create(pvr_dev,
				   PVR_LAGUNA_FWIF_TRACE_BUF_DWORDS * sizeof(u32),
				   PVR_BO_FW_FLAGS_DEVICE_UNCACHED, NULL, NULL,
				   &laguna->fwif_trace_data_obj);
	if (err)
		goto err_destroy;

	laguna->fwif_tracebuf = pvr_fw_object_create_and_map(pvr_dev,
							     sizeof(*laguna->fwif_tracebuf),
							     PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							     pvr_laguna_tracebuf_init,
							     pvr_dev,
							     &laguna->fwif_tracebuf_obj);
	if (IS_ERR(laguna->fwif_tracebuf)) {
		err = PTR_ERR(laguna->fwif_tracebuf);
		laguna->fwif_tracebuf = NULL;
		goto err_destroy;
	}

	laguna->kccb_ctl = pvr_fw_object_create_and_map(pvr_dev,
							sizeof(*laguna->kccb_ctl),
							PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							pvr_laguna_kccb_ctl_init, NULL,
							&laguna->kccb_ctl_obj);
	if (IS_ERR(laguna->kccb_ctl)) {
		err = PTR_ERR(laguna->kccb_ctl);
		laguna->kccb_ctl = NULL;
		goto err_destroy;
	}

	laguna->kccb = pvr_fw_object_create_and_map(pvr_dev,
						    PVR_LAGUNA_KCCB_NUMCMDS *
						    sizeof(*laguna->kccb),
						    PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
						    NULL, NULL, &laguna->kccb_obj);
	if (IS_ERR(laguna->kccb)) {
		err = PTR_ERR(laguna->kccb);
		laguna->kccb = NULL;
		goto err_destroy;
	}

	laguna->kccb_rtn = pvr_fw_object_create_and_map(pvr_dev,
							PVR_LAGUNA_KCCB_NUMCMDS *
							sizeof(*laguna->kccb_rtn),
							PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							NULL, NULL,
							&laguna->kccb_rtn_obj);
	if (IS_ERR(laguna->kccb_rtn)) {
		err = PTR_ERR(laguna->kccb_rtn);
		laguna->kccb_rtn = NULL;
		goto err_destroy;
	}

	laguna->fwccb_ctl = pvr_fw_object_create_and_map(pvr_dev,
							 sizeof(*laguna->fwccb_ctl),
							 PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							 pvr_laguna_fwccb_ctl_init, NULL,
							 &laguna->fwccb_ctl_obj);
	if (IS_ERR(laguna->fwccb_ctl)) {
		err = PTR_ERR(laguna->fwccb_ctl);
		laguna->fwccb_ctl = NULL;
		goto err_destroy;
	}

	laguna->fwccb = pvr_fw_object_create_and_map(pvr_dev,
						     PVR_LAGUNA_FWCCB_NUMCMDS *
						     sizeof(*laguna->fwccb),
						     PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
						     NULL, NULL, &laguna->fwccb_obj);
	if (IS_ERR(laguna->fwccb)) {
		err = PTR_ERR(laguna->fwccb);
		laguna->fwccb = NULL;
		goto err_destroy;
	}

	/* Fixed-offset FW_CONFIG_HEAP structures (top 192 KiB of 32 MiB FW heap). */
	laguna->fwif_connection_ctl =
		pvr_fw_object_create_and_map_offset(pvr_dev,
						    PVR_LAGUNA_FW_CONNECTION_CTL_OFF,
						    sizeof(*laguna->fwif_connection_ctl),
						    PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
						    NULL, NULL,
						    &laguna->fwif_connection_ctl_obj);
	if (IS_ERR(laguna->fwif_connection_ctl)) {
		err = PTR_ERR(laguna->fwif_connection_ctl);
		laguna->fwif_connection_ctl = NULL;
		goto err_destroy;
	}

	laguna->fwif_osinit =
		pvr_fw_object_create_and_map_offset(pvr_dev,
						    PVR_LAGUNA_FW_OSINIT_OFF,
						    sizeof(*laguna->fwif_osinit),
						    PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
						    pvr_laguna_osinit_init, pvr_dev,
						    &laguna->fwif_osinit_obj);
	if (IS_ERR(laguna->fwif_osinit)) {
		err = PTR_ERR(laguna->fwif_osinit);
		laguna->fwif_osinit = NULL;
		goto err_destroy;
	}

	laguna->fwif_sysinit =
		pvr_fw_object_create_and_map_offset(pvr_dev,
						    PVR_LAGUNA_FW_SYSINIT_OFF,
						    sizeof(*laguna->fwif_sysinit),
						    PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
						    pvr_laguna_sysinit_init, pvr_dev,
						    &laguna->fwif_sysinit_obj);
	if (IS_ERR(laguna->fwif_sysinit)) {
		err = PTR_ERR(laguna->fwif_sysinit);
		laguna->fwif_sysinit = NULL;
		goto err_destroy;
	}

	pvr_dev->fw_dev.power_sync = laguna->power_sync;
	pvr_dev->fw_dev.mem.mmucache_sync_obj = laguna->fwif_mmucache_sync_obj;
	pvr_dev->kccb.rtn = laguna->kccb_rtn;

	/* Flush all WC carveout FWIF structures to DRAM before FW boot. */
	wmb();
	return 0;

err_destroy:
	pvr_laguna_fwif_destroy_structures(pvr_dev);
	return err;
}

static void pvr_laguna_fw_health_check_poll(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	struct rogue_fwif_kccb_cmd cmd = {
		.cmd_type = ROGUE_FWIF_KCCB_CMD_HEALTH_CHECK,
	};
	ktime_t deadline;
	u32 slot = 0;
	u32 rtn = 0;
	int err;

	err = pvr_kccb_send_cmd_powered(pvr_dev, &cmd, &slot);
	if (err) {
		drm_warn(drm_dev, "Volcanic FW health check send failed (%d)\n",
			 err);
		return;
	}

	deadline = ktime_add_us(ktime_get(), PVR_LAGUNA_FW_HEALTH_TIMEOUT_US);
	while (ktime_before(ktime_get(), deadline)) {
		rtn = READ_ONCE(laguna->kccb_rtn[slot]);
		if (rtn & PVR_LAGUNA_KCCB_RTN_CMD_EXECUTED)
			break;
		usleep_range(200, 500);
	}

	if (!pvr_dev->irq && pvr_laguna_irq_pending(pvr_dev))
		pvr_laguna_irq_clear(pvr_dev);

	if (rtn & PVR_LAGUNA_KCCB_RTN_CMD_EXECUTED) {
		drm_info(drm_dev,
			 "Volcanic FW KCCB HEALTH_CHECK passed (slot=%u, rtn=%#x, executed=%u)\n",
			 slot, rtn,
			 READ_ONCE(laguna->fwif_osdata->kccb_cmds_executed));
	} else {
		drm_warn(drm_dev,
			 "Volcanic FW KCCB HEALTH_CHECK timed out (slot=%u, woff=%u, roff=%u, executed=%u)\n",
			 slot, READ_ONCE(laguna->kccb_ctl->write_offset),
			 READ_ONCE(laguna->kccb_ctl->read_offset),
			 READ_ONCE(laguna->fwif_osdata->kccb_cmds_executed));
	}
}

int pvr_laguna_fw_wait_boot(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	ktime_t deadline;
	bool started = false;
	u32 stage, fw_state, faults, irq_stat;

	if (!laguna || !laguna->fwif_sysinit)
		return 0;

	deadline = ktime_add_us(ktime_get(), PVR_LAGUNA_FW_BOOT_TIMEOUT_US);
	while (ktime_before(ktime_get(), deadline)) {
		if (READ_ONCE(laguna->fwif_sysinit->firmware_started)) {
			started = true;
			break;
		}
		usleep_range(500, 1000);
	}

	stage = pvr_cr_read32(pvr_dev, VOLCANIC_CR_SCRATCH14);
	fw_state = laguna->fwif_connection_ctl ?
		   READ_ONCE(laguna->fwif_connection_ctl->connection_fw_state) : 0;
	faults = laguna->fwif_sysdata ?
		 READ_ONCE(laguna->fwif_sysdata->fw_faults) : 0;
	irq_stat = pvr_cr_read32(pvr_dev, VOLCANIC_CR_IRQ_OS0_EVENT_STATUS);

	WRITE_ONCE(laguna->fw_kccb_ready, started);

	if (started) {
		drm_info(drm_dev,
			 "Volcanic FW boot handshake succeeded (stage=%u, fw_state=%u, faults=%u, ts=%u)\n",
			 stage, fw_state, faults,
			 READ_ONCE(laguna->fwif_sysinit->firmware_started_timestamp));
		pvr_laguna_fw_health_check_poll(pvr_dev);
	} else {
		drm_warn(drm_dev,
			 "Volcanic FW boot handshake timed out (stage=%u, fw_state=%u, faults=%u, irq=%#x); keeping Stage G2 UAPI/carveout allocator active\n",
			 stage, fw_state, faults, irq_stat);
	}

	return 0;
}

int pvr_laguna_fw_stop(struct pvr_device *pvr_dev)
{
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	if (!laguna || !laguna->fw_running)
		return 0;

	WRITE_ONCE(laguna->fw_kccb_ready, false);

	err = pvr_laguna_smc_gpu_control(pvr_dev,
					 PVR_LAGUNA_OEM_GPU_CONTROL_STOP);
	if (!err) {
		laguna->fw_running = false;
		if (laguna->fwif_sysinit)
			WRITE_ONCE(laguna->fwif_sysinit->firmware_started, 0);
		if (laguna->kccb_ctl) {
			WRITE_ONCE(laguna->kccb_ctl->write_offset, 0);
			WRITE_ONCE(laguna->kccb_ctl->read_offset, 0);
		}
		if (laguna->fwccb_ctl) {
			WRITE_ONCE(laguna->fwccb_ctl->write_offset, 0);
			WRITE_ONCE(laguna->fwccb_ctl->read_offset, 0);
		}
		wmb(); /* Ensure reset FWIF state reaches DRAM before restart. */
	}

	return err;
}

int pvr_laguna_fw_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;
	int err;

	if (!laguna)
		return -ENODEV;

	fw_dev->defs = &pvr_fw_defs_laguna;

	err = fw_dev->defs->init(pvr_dev);
	if (err)
		return err;

	laguna->fw_shared_cpu = memremap(laguna->fw_carveout_phys +
					 PVR_LAGUNA_FW_SHARED_HEAP_OFFSET,
					 PVR_LAGUNA_FW_SHARED_HEAP_SIZE,
					 MEMREMAP_WC);
	if (!laguna->fw_shared_cpu) {
		drm_err(drm_dev, "failed to remap gpu_fw shared heap\n");
		return -ENOMEM;
	}
	memset(laguna->fw_shared_cpu, 0, PVR_LAGUNA_FW_SHARED_HEAP_SIZE);

	drm_mm_init(&fw_dev->fw_mm, ROGUE_FW_HEAP_BASE,
		    fw_dev->fw_heap_info.raw_size);
	fw_dev->fw_mm_base = ROGUE_FW_HEAP_BASE;
	spin_lock_init(&fw_dev->fw_mm_lock);

	INIT_LIST_HEAD(&fw_dev->fw_objs.list);
	err = drmm_mutex_init(drm_dev, &fw_dev->fw_objs.lock);
	if (err)
		goto err_mm_takedown;

	pvr_dev->kccb.slot_count = PVR_LAGUNA_KCCB_NUMCMDS;
	init_waitqueue_head(&pvr_dev->kccb.rtn_q);
	INIT_LIST_HEAD(&pvr_dev->kccb.waiters);
	pvr_dev->kccb.fence_ctx.id = dma_fence_context_alloc(1);
	spin_lock_init(&pvr_dev->kccb.fence_ctx.lock);

	err = drmm_mutex_init(drm_dev, &pvr_dev->kccb.ccb.lock);
	if (err)
		goto err_mm_takedown;

	err = drmm_mutex_init(drm_dev, &pvr_dev->fwccb.lock);
	if (err)
		goto err_mm_takedown;

	/* Reserve the Trusty-locked 2 MiB secure FW region at heap offset 0. */
	laguna->fw_secure_rsvd.start = ROGUE_FW_HEAP_BASE;
	laguna->fw_secure_rsvd.size = PVR_LAGUNA_SECURE_FW_MEM_SIZE;
	err = drm_mm_reserve_node(&fw_dev->fw_mm, &laguna->fw_secure_rsvd);
	if (err)
		goto err_mm_takedown;

	err = pvr_laguna_fwif_create_structures(pvr_dev);
	if (err)
		goto err_remove_secure_rsvd;

	err = pvr_laguna_fw_start(pvr_dev);
	if (err)
		goto err_destroy_fwif;

	pvr_laguna_fw_wait_boot(pvr_dev);

	WRITE_ONCE(fw_dev->initialised, true);
	return 0;

err_destroy_fwif:
	pvr_laguna_fwif_destroy_structures(pvr_dev);
err_remove_secure_rsvd:
	drm_mm_remove_node(&laguna->fw_secure_rsvd);
err_mm_takedown:
	drm_mm_takedown(&fw_dev->fw_mm);
	memunmap(laguna->fw_shared_cpu);
	laguna->fw_shared_cpu = NULL;
	return err;
}

void pvr_laguna_fw_fini(struct pvr_device *pvr_dev)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct pvr_laguna_data *laguna = pvr_dev->laguna;

	WRITE_ONCE(fw_dev->initialised, false);

	if (!laguna)
		return;

	pvr_laguna_fw_stop(pvr_dev);
	pvr_laguna_fwif_destroy_structures(pvr_dev);

	if (drm_mm_node_allocated(&laguna->fw_secure_rsvd))
		drm_mm_remove_node(&laguna->fw_secure_rsvd);

	if (laguna->fw_shared_cpu) {
		drm_mm_takedown(&fw_dev->fw_mm);
		memunmap(laguna->fw_shared_cpu);
		laguna->fw_shared_cpu = NULL;
	}

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
