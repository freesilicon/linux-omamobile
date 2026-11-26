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

#include "pvr_laguna_fwif.h"

#include <drm/drm_mm.h>

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct clk;
struct firmware;
struct pvr_device;
struct pvr_fw_object;
struct pvr_job;
struct pvr_power_sequence_ops;
struct pvr_queue;
struct rogue_fwif_cdm_registers_cswitch;
struct rogue_fwif_kccb_cmd;
struct tipc_chan;

/* Laguna PowerVR firmware carveout layout (gpu_fw@b9f00000, 33 MiB no-map). */
#define PVR_LAGUNA_FW_CARVEOUT_MIN_SIZE		0x02100000ULL /* 33 MiB */
#define PVR_LAGUNA_SECURE_FW_MEM_SIZE		0x00200000ULL /* 2 MiB (Trusty isolated) */
#define PVR_LAGUNA_FW_CODE_HEAP_OFFSET		0x00000000ULL
#define PVR_LAGUNA_FW_CODE_HEAP_SIZE		0x00120000ULL
#define PVR_LAGUNA_FW_PRIV_DATA_HEAP_OFFSET	0x00120000ULL
#define PVR_LAGUNA_FW_PRIV_DATA_HEAP_SIZE	0x000C0000ULL
#define PVR_LAGUNA_FW_RODATA_HEAP_OFFSET	0x001E0000ULL
#define PVR_LAGUNA_FW_RODATA_HEAP_SIZE		0x00020000ULL
#define PVR_LAGUNA_FW_SHARED_HEAP_OFFSET	0x00200000ULL
#define PVR_LAGUNA_FW_SHARED_HEAP_SIZE		0x01E00000ULL /* 30 MiB */
#define PVR_LAGUNA_FW_PT_HEAP_OFFSET		0x02000000ULL
#define PVR_LAGUNA_FW_PT_HEAP_SIZE		0x00100000ULL /* 1 MiB */

/* Signed firmware ("rgx.fw.signed") catalogue, footer, and auth header ABI. */
#define PVR_LAGUNA_SIGNED_FW_NAME		"google/rgx.fw.signed"
#define PVR_LAGUNA_SIGNED_FW_MAGIC		0x46555047U /* "GPUF" */
#define PVR_LAGUNA_SIGNED_FW_WHI_HDR_SIZE	0x1000U
#define PVR_LAGUNA_SIGNED_FW_WHI_MAGIC_OFF	0x0400U
#define PVR_LAGUNA_SIGNED_FW_LGA_VERSION	1U
#define PVR_LAGUNA_SIGNED_FW_LGA_HDR_SIZE	0x1000U
#define PVR_LAGUNA_SIGNED_FW_LGA_MAGIC_OFF	0x0864U
#define PVR_LAGUNA_SIGNED_FW_MBU_VERSION	2U
#define PVR_LAGUNA_SIGNED_FW_MBU_HDR_SIZE	0x5000U
#define PVR_LAGUNA_SIGNED_FW_MBU_MAGIC_OFF	0x463CU

#define PVR_LAGUNA_FW_FOOTER_OFF_FROM_END	0x0800U
#define PVR_LAGUNA_FW_FOOTER_MAGIC		0x1BADD00DU
#define PVR_LAGUNA_FW_FOOTER_VERSION		0U
#define PVR_LAGUNA_FW_FOOTER_MAX_PA_RANGES	64U

/* Trusty IPC port and TF-A EL3 SMC_OEM_GPU_CONTROL ABI. */
#define PVR_LAGUNA_TRUSTY_GPU_PORT		"com.android.trusty.pixel_gpu"
#define PVR_LAGUNA_GPU_SECURE_MAX_MSG_SIZE	1024U
#define PVR_LAGUNA_GPU_SECURE_CMD_PING		0U
#define PVR_LAGUNA_GPU_SECURE_CMD_SEND_FW	1U
#define PVR_LAGUNA_GPU_SECURE_CMD_START		2U
#define PVR_LAGUNA_GPU_SECURE_CMD_STOP		3U
#define PVR_LAGUNA_GPU_SECURE_CMD_FAULT		4U

#define PVR_LAGUNA_GPU_SECURE_FW_VER_PRE_25_2	0U
#define PVR_LAGUNA_GPU_SECURE_FW_VER_25_2	1U

#define PVR_LAGUNA_GPU_SECURE_SEC_CODE		0U
#define PVR_LAGUNA_GPU_SECURE_SEC_DATA		1U
#define PVR_LAGUNA_GPU_SECURE_SEC_RODATA	2U
#define PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_CODE	3U
#define PVR_LAGUNA_GPU_SECURE_SEC_COREMEM_DATA	4U
#define PVR_LAGUNA_GPU_SECURE_SEC_COUNT		5U

/* RISC-V FW cached heap base (region 0xC << 28) for COREMEM fw_va mapping. */
#define PVR_LAGUNA_RISCV_FW_HEAP_BASE		0xC0000000U

#define PVR_LAGUNA_SMC_OEM_GPU_CONTROL		0xC300000CUL
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
	struct pvr_device *pvr_dev;
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

	/* Parsed signed firmware entry & carveout staging metadata. */
	u32 fw_hdr_file_off;
	u32 fw_hdr_size;
	u32 fw_hdr_magic_off;
	u32 fw_body_file_off;
	u32 fw_body_size;
	u32 fw_abi_version;
	u32 fw_code_alloc_size;
	u32 fw_data_alloc_size;
	u32 fw_rodata_alloc_size;
	u32 fw_coremem_code_alloc_size;
	u32 fw_coremem_data_alloc_size;
	u64 fw_coremem_code_carveout_off;
	u64 fw_coremem_data_carveout_off;
	u64 fw_body_carveout_off;
	u64 fw_hdr_carveout_off;
	bool fw_authenticated;
	bool fw_running;
	bool fw_kccb_ready;
	bool fw_wedged;

	/* Shared FW heap (gpu_fw@b9f00000 + 2 MiB, 30 MiB pre-mapped by Trusty). */
	void *fw_shared_cpu;
	struct drm_mm_node fw_secure_rsvd;

	/* DDK 25.3 FWIF objects allocated in the Trusty pre-mapped FW heap. */
	struct pvr_fw_object *fwif_connection_ctl_obj;
	struct pvr_fw_object *fwif_osinit_obj;
	struct pvr_fw_object *fwif_sysinit_obj;
	struct pvr_fw_object *fwif_osdata_obj;
	struct pvr_fw_object *fwif_sysdata_obj;
	struct pvr_fw_object *fwif_runtime_cfg_obj;
	struct pvr_fw_object *fwif_tracebuf_obj;
	struct pvr_fw_object *fwif_trace_data_obj;
	struct pvr_fw_object *fwif_hwrinfobuf_obj;
	struct pvr_fw_object *fwif_gpu_util_obj;
	struct pvr_fw_object *fwif_hwperf_data_obj;
	struct pvr_fw_object *fwif_hwperf_rtcfg_obj;
	struct pvr_fw_object *fwif_hwperf_ctl_obj;
	struct pvr_fw_object *fwif_fault_page_obj;
	struct pvr_fw_object *fwif_align_checks_obj;
	struct pvr_fw_object *fwif_power_sync_obj;
	struct pvr_fw_object *fwif_mmucache_sync_obj;
	struct pvr_fw_object *kccb_ctl_obj;
	struct pvr_fw_object *kccb_obj;
	struct pvr_fw_object *kccb_rtn_obj;
	struct pvr_fw_object *fwccb_ctl_obj;
	struct pvr_fw_object *fwccb_obj;

	struct pvr_laguna_fwif_connection_ctl *fwif_connection_ctl;
	struct pvr_laguna_fwif_osinit *fwif_osinit;
	struct pvr_laguna_fwif_sysinit *fwif_sysinit;
	struct pvr_laguna_fwif_osdata *fwif_osdata;
	struct pvr_laguna_fwif_sysdata *fwif_sysdata;
	struct pvr_laguna_fwif_runtime_cfg *fwif_runtime_cfg;
	struct pvr_laguna_fwif_tracebuf *fwif_tracebuf;
	u32 *fwif_trace_data;
	struct pvr_laguna_fwif_hwrinfobuf *fwif_hwrinfobuf;
	struct pvr_laguna_fwif_ccb_ctl *kccb_ctl;
	struct pvr_laguna_fwif_kccb_cmd *kccb;
	u32 *kccb_rtn;
	struct pvr_laguna_fwif_ccb_ctl *fwccb_ctl;
	struct pvr_laguna_fwif_fwccb_cmd *fwccb;
	u32 *power_sync;
	u32 *mmucache_sync;
	u32 mmucache_sync_seq;
	u64 last_cdm_cs_base;
	u64 last_cdm_ctx_state_base;

	struct work_struct restart_work;
	spinlock_t restart_lock; /* protects restart burst tracking */
	unsigned long restart_window_jiffies;
	u32 restart_burst_count;

	struct pvr_laguna_tz tz;
};

extern const struct pvr_power_sequence_ops pvr_power_sequence_ops_laguna;

int pvr_laguna_request_signed_firmware(struct pvr_device *pvr_dev);
int pvr_laguna_fw_init(struct pvr_device *pvr_dev);
void pvr_laguna_fw_fini(struct pvr_device *pvr_dev);
int pvr_laguna_fw_start(struct pvr_device *pvr_dev);
int pvr_laguna_fw_stop(struct pvr_device *pvr_dev);
int pvr_laguna_fw_wait_boot(struct pvr_device *pvr_dev);
void pvr_laguna_fw_health_check_irq(struct pvr_device *pvr_dev);
int pvr_laguna_wait_pow_off(struct pvr_device *pvr_dev);
void pvr_laguna_slc_program_lut(struct pvr_device *pvr_dev);
int pvr_laguna_mba_send_msg(struct pvr_device *pvr_dev, u32 client_idx, u32 msg);
unsigned long pvr_laguna_get_core_clk_rate(const struct pvr_device *pvr_dev);

void *pvr_laguna_fw_shared_vaddr(struct pvr_device *pvr_dev, u32 fw_offset);
phys_addr_t pvr_laguna_fw_shared_paddr(struct pvr_device *pvr_dev, u32 fw_offset);
bool pvr_laguna_fw_ready(const struct pvr_device *pvr_dev);
bool pvr_laguna_kccb_ready(const struct pvr_device *pvr_dev);
bool pvr_laguna_is_wedged(const struct pvr_device *pvr_dev);
bool pvr_laguna_power_is_idle(struct pvr_device *pvr_dev);
bool pvr_laguna_needs_reset(const struct pvr_device *pvr_dev);
void pvr_laguna_clear_hwr_flags(struct pvr_device *pvr_dev);
void pvr_laguna_schedule_restart(struct pvr_device *pvr_dev);
void pvr_laguna_dump_fw_state(struct pvr_device *pvr_dev, const char *reason);
u32 pvr_laguna_translate_cccb_cmd_type(u32 rogue_cmd_type);
u32 pvr_laguna_kccb_used_slot_count_locked(struct pvr_device *pvr_dev);
void pvr_laguna_kccb_send_cmd_reserved_powered(struct pvr_device *pvr_dev,
					       struct rogue_fwif_kccb_cmd *cmd,
					       u32 *kccb_slot);
bool pvr_laguna_kccb_is_idle(struct pvr_device *pvr_dev);
void pvr_laguna_fwccb_process(struct pvr_device *pvr_dev);
void pvr_laguna_init_fw_common_context(struct pvr_queue *queue, void *fw_ctx_map);
void pvr_laguna_copy_cdm_cswitch_regs(void *fw_ctx_map,
				      const struct rogue_fwif_cdm_registers_cswitch *src);
int pvr_laguna_translate_cmd_compute(struct pvr_job *job);

#endif /* PVR_LAGUNA_H */
