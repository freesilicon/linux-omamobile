/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Clean-room Tensor G5 (Laguna) platform glue for PowerVR Volcanic
 * (DXT-48-1536, BVNC 71.2.2448.1212):
 *  - SLC PBHA LUT programming (req_pbha_lut + ra_lut_{0..2}, OSID 1 only)
 *  - PFSM frequency/clock state wrapper (pfsm)
 *  - GPU MBA host glue & client CSR binding (mba_glue + mba_client)
 *  - Core logic power-on/off sequence (gpu_control / pvtc / dts)
 *  - Trusty IPC ("com.android.trusty.pixel_gpu") + TF-A SMC_OEM_GPU_CONTROL
 *    (0xC300000C) signed rgx.fw.signed loader using gpu_fw@b9f00000 carveout
 */

#ifndef PVR_LAGUNA_H
#define PVR_LAGUNA_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct clk;
struct firmware;
struct pvr_device;
struct pvr_power_sequence_ops;
struct tipc_chan;

/* Laguna PowerVR firmware carveout layout (gpu_fw@b9f00000, 33 MiB no-map). */
#define PVR_LAGUNA_FW_CARVEOUT_MIN_SIZE		0x02100000ULL /* 33 MiB */
#define PVR_LAGUNA_SECURE_FW_MEM_SIZE		0x00200000ULL /* 2 MiB (Trusty isolated) */
#define PVR_LAGUNA_FW_CODE_HEAP_SIZE		0x00120000ULL
#define PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE	0x000C0000ULL
#define PVR_LAGUNA_FW_RODATA_HEAP_SIZE		0x00020000ULL
#define PVR_LAGUNA_FW_SHARED_HEAP_OFFSET	0x00200000ULL
#define PVR_LAGUNA_FW_SHARED_HEAP_SIZE		0x01E00000ULL /* 30 MiB */
#define PVR_LAGUNA_FW_PT_HEAP_OFFSET		0x02000000ULL
#define PVR_LAGUNA_FW_PT_HEAP_SIZE		0x00100000ULL /* 1 MiB */

/* Signed firmware ("rgx.fw.signed") header validation ("GPUF" little-endian). */
#define PVR_LAGUNA_SIGNED_FW_NAME		"google/rgx.fw.signed"
#define PVR_LAGUNA_SIGNED_FW_MAGIC		0x46555047U /* "GPUF" */
#define PVR_LAGUNA_SIGNED_FW_HDR_OFF_0		0x0400U
#define PVR_LAGUNA_SIGNED_FW_HDR_OFF_1		0x0864U

/* Trusty IPC port and TF-A EL3 SMC_OEM_GPU_CONTROL ABI. */
#define PVR_LAGUNA_TRUSTY_GPU_PORT		"com.android.trusty.pixel_gpu"
#define PVR_LAGUNA_GPU_SECURE_REQ_VERSION	1U
#define PVR_LAGUNA_GPU_SECURE_REQ_SEND_FW	1U

#define PVR_LAGUNA_SMC_OEM_GPU_CONTROL		0xC300000CUL
#define PVR_LAGUNA_OEM_GPU_CONTROL_SETUP	0UL
#define PVR_LAGUNA_OEM_GPU_CONTROL_START	1UL
#define PVR_LAGUNA_OEM_GPU_CONTROL_STOP		2UL

/* SLC PBHA LUT: OSID 1 (Non-Secure Linux) only; never touch OSID 0 (FW) or 7 (TZ). */
#define PVR_LAGUNA_OSID_FW			0U
#define PVR_LAGUNA_OSID_LINUX			1U
#define PVR_LAGUNA_OSID_TZ			7U
#define PVR_LAGUNA_RA_LUT_BANKS			3U

/* PFSM operating range (Hz) when standard CCF "core" clock is absent. */
#define PVR_LAGUNA_GPU_INIT_FREQ_HZ		300000000UL
#define PVR_LAGUNA_GPU_SOFT_MIN_FREQ_HZ		396000000UL
#define PVR_LAGUNA_GPU_SOFT_MAX_FREQ_HZ		1094000000UL

/* GPU MBA client stride and message register offsets. */
#define PVR_LAGUNA_MBA_GLUE_INTR_ITR_OFF	0x00U
#define PVR_LAGUNA_MBA_CLIENT_STRIDE		0x1000U
#define PVR_LAGUNA_MBA_CLIENT_MAX		4U
#define PVR_LAGUNA_MBA_CLIENT_COMMON_MSG_OFF	0x00U

/**
 * struct pvr_laguna_tz - Trusty IPC session state for pixel_gpu TA
 */
struct pvr_laguna_tz {
	struct device *dev;
	const char *port;
	struct tipc_chan *chan;
	struct mutex req_lock; /* serializes Trusty IPC requests */
	struct mutex rsp_lock; /* protects rsp_buf, rsp_size, and rsp_res */
	struct completion reply;
	void *rsp_buf;
	size_t rsp_size;
	int rsp_res;
	int last_event;
};

/**
 * struct pvr_laguna_data - Tensor G5 Laguna platform state for PowerVR Volcanic
 */
struct pvr_laguna_data {
	void __iomem *pfsm;
	void __iomem *req_pbha_lut;
	void __iomem *ra_lut[PVR_LAGUNA_RA_LUT_BANKS];
	void __iomem *mba_glue;
	void __iomem *mba_client;

	/* Optional core-logic power control CSRs (Stage G0 verified sequence). */
	void __iomem *gpu_control;
	void __iomem *pvtc;
	void __iomem *pvtc_pwroff;
	void __iomem *dts;

	/* Optional PFSM protocol clock ("gpu_pf_state"). */
	struct clk *pf_state_clk;
	unsigned long core_freq_hz;

	/* Reserved memory carveout (gpu_fw@b9f00000, no-map). */
	phys_addr_t fw_carveout_phys;
	size_t fw_carveout_size;

	/* Signed firmware header offset validated in rgx.fw.signed. */
	u32 signed_fw_hdr_offset;
	bool fw_authenticated;
	bool fw_running;

	struct pvr_laguna_tz tz;
};

extern const struct pvr_power_sequence_ops pvr_power_sequence_ops_laguna;

int pvr_laguna_request_signed_firmware(struct pvr_device *pvr_dev);
int pvr_laguna_fw_init(struct pvr_device *pvr_dev);
void pvr_laguna_fw_fini(struct pvr_device *pvr_dev);
int pvr_laguna_fw_start(struct pvr_device *pvr_dev);
int pvr_laguna_fw_stop(struct pvr_device *pvr_dev);
void pvr_laguna_slc_program_lut(struct pvr_device *pvr_dev);
int pvr_laguna_mba_send_msg(struct pvr_device *pvr_dev, u32 client_idx, u32 msg);
unsigned long pvr_laguna_get_core_clk_rate(const struct pvr_device *pvr_dev);

#endif /* PVR_LAGUNA_H */
