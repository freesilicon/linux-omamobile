/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Clean-room DDK 25.3 Volcanic FWIF ABI definitions for Tensor G5 (Laguna).
 *
 * All structure sizes and field offsets are verified against the GPL-2.0
 * Tensor G5 (lga/production, BVNC 71.2.2448.1212) kernel interface and locked
 * with compile-time static_assert() checks.
 */

#ifndef PVR_LAGUNA_FWIF_H
#define PVR_LAGUNA_FWIF_H

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/stddef.h>
#include <linux/types.h>

/* Laguna FW heap offsets inside the 32 MiB pre-mapped FW address space. */
#define PVR_LAGUNA_FW_HEAP_SHIFT		25U
#define PVR_LAGUNA_FW_HEAP_RAW_SIZE		((u32)BIT(PVR_LAGUNA_FW_HEAP_SHIFT))
#define PVR_LAGUNA_FW_CONFIG_GRANULARITY	0x10000U
#define PVR_LAGUNA_FW_CONFIG_HEAP_SIZE		(3U * PVR_LAGUNA_FW_CONFIG_GRANULARITY)
#define PVR_LAGUNA_FW_CONFIG_HEAP_OFFSET	(PVR_LAGUNA_FW_HEAP_RAW_SIZE - \
						 PVR_LAGUNA_FW_CONFIG_HEAP_SIZE)
#define PVR_LAGUNA_FW_CONNECTION_CTL_OFF	(PVR_LAGUNA_FW_CONFIG_HEAP_OFFSET + 0x00000U)
#define PVR_LAGUNA_FW_OSINIT_OFF		(PVR_LAGUNA_FW_CONFIG_HEAP_OFFSET + 0x10000U)
#define PVR_LAGUNA_FW_SYSINIT_OFF		(PVR_LAGUNA_FW_CONFIG_HEAP_OFFSET + 0x20000U)

/* DDK 25.3 KCCB / FWCCB geometry and command encoding. */
#define PVR_LAGUNA_KCCB_NUMCMDS_LOG2		7U
#define PVR_LAGUNA_KCCB_NUMCMDS			((u32)BIT(PVR_LAGUNA_KCCB_NUMCMDS_LOG2))
#define PVR_LAGUNA_FWCCB_NUMCMDS_LOG2		5U
#define PVR_LAGUNA_FWCCB_NUMCMDS		((u32)BIT(PVR_LAGUNA_FWCCB_NUMCMDS_LOG2))

#define PVR_LAGUNA_CMD_MAGIC_SHIFTED		0x2abc0000U
#define PVR_LAGUNA_KCCB_CMD_KICK		(101U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_KCCB_CMD_MMUCACHE		(102U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_KCCB_CMD_CLEANUP		(105U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_KCCB_CMD_HEALTH_CHECK	(111U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_KCCB_CMD_POW			(200U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)

#define PVR_LAGUNA_KCCB_RTN_NO_RESPONSE		0U
#define PVR_LAGUNA_KCCB_RTN_CMD_EXECUTED	BIT(0)
#define PVR_LAGUNA_KCCB_RTN_CLEANUP_BUSY	BIT(1)

/* DDK 25.3 FWCCB command types (RGXCheckFirmwareCCB jump table). */
#define PVR_LAGUNA_FWCCB_CMD_ZSBUFFER_BACKING		(101U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_ZSBUFFER_UNBACKING		(102U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_FREELIST_GROW		(103U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_FREELISTS_RECONSTRUCT	(104U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_CONTEXT_RESET		(105U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_DEBUG_DUMP			(106U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_UPDATE_STATS		(107U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_CORECLK_RATE_CHANGE	(108U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_REQUEST_GPU_RESTART	(109U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_FWCCB_CMD_FW_PAGEFAULT		(112U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)

/* DDK 25.3 Client CCB (cCCB) command types (DumpCCB / RGXCmdHelperAcquireCmdCCB). */
#define PVR_LAGUNA_CCB_CMD_DM_BIT		BIT(15)
#define PVR_LAGUNA_CCB_CMD_TYPE_TA		(201U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_TQ_3D		(202U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_3D		(203U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_3D_PR		(204U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_CDM		(205U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_TQ_TDM		(206U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_NULL		(210U | PVR_LAGUNA_CMD_MAGIC_SHIFTED | \
						 PVR_LAGUNA_CCB_CMD_DM_BIT)
#define PVR_LAGUNA_CCB_CMD_TYPE_FENCE		(212U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_CCB_CMD_TYPE_UPDATE		(213U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_CCB_CMD_TYPE_RMW_UPDATE	(214U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_CCB_CMD_TYPE_FENCE_PR	(215U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)
#define PVR_LAGUNA_CCB_CMD_TYPE_PADDING		(221U | PVR_LAGUNA_CMD_MAGIC_SHIFTED)

/* DDK 25.3 Client CCB command header (16 bytes; no Rogue workest trailer). */
struct pvr_laguna_fwif_ccb_cmd_header {
	u32 cmd_type;
	u32 cmd_size;
	u32 ext_job_ref;
	u32 int_job_ref;
};

static_assert(sizeof(struct pvr_laguna_fwif_ccb_cmd_header) == 16);

/* DDK 25.3 Volcanic RGXFWIF_CMDCDM flags (at cmd_cdm + 0x40). */
#define PVR_LAGUNA_CDM_FLAG_WGP_CHECKSUM	BIT(0)
#define PVR_LAGUNA_CDM_FLAG_PREVENT_ALL_OVERLAP	BIT(1)
#define PVR_LAGUNA_CDM_FLAG_USRM_RECONFIG	BIT(3)

/*
 * DDK 25.3 Volcanic RGXFWIF_CDM_REGISTERS (56 bytes, loaded at CMDCDM + 0x08).
 * Matches the live Volcanic firmware register-load order:
 *   +0x00: CR 0x17a8 (TPU_BORDER_COLOUR_TABLE_CDM)
 *   +0x08: CR 0x1878 (TPU_TAG_CDM_CTRL)
 *   +0x10: CR 0x04e8 (CDM_CTRL_STREAM_BASE)
 *   +0x18: CR 0x0598 (CDM_CB_QUEUE)
 *   +0x20: CDM_CONTEXT_LOAD_PDS1 (written via CR 0x04e0 on context load)
 *   +0x28: CR 0x0498 (CDM_CONTEXT_STATE_BASE_ADDR)
 *   +0x30: CR 0x04d0 (CDM_ITEM)
 */
struct pvr_laguna_fwif_cdm_regs {
	u64 tpu_border_colour_table;
	u64 tpu_tag_cdm_ctrl;
	u64 cdm_ctrl_stream_base;
	u64 cdm_cb_queue;
	u32 cdm_resume_pds1;
	u32 pad_24;
	u64 cdm_context_state_base_addr;
	u32 cdm_item;
	u32 pad_34;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_cdm_regs) == 56);
static_assert(offsetof(struct pvr_laguna_fwif_cdm_regs, cdm_ctrl_stream_base) == 0x10);
static_assert(offsetof(struct pvr_laguna_fwif_cdm_regs, cdm_resume_pds1) == 0x20);
static_assert(offsetof(struct pvr_laguna_fwif_cdm_regs, cdm_context_state_base_addr) == 0x28);
static_assert(offsetof(struct pvr_laguna_fwif_cdm_regs, cdm_item) == 0x30);

/*
 * DDK 25.3 Volcanic RGXFWIF_CMDCDM (120 bytes; 136 bytes / 0x88 with 16-byte
 * cCCB command header as DMA-loaded by the Volcanic firmware CDM kick path).
 */
struct pvr_laguna_fwif_cmd_cdm {
	u32 frame_num;
	u32 pad_04;
	struct pvr_laguna_fwif_cdm_regs regs;
	u32 flags;
	u32 num_shareds;
	u32 max_num_temp_regions;
	u32 usc_tasks_stride;
	u64 reserved_50;
	u32 wgp_checksum[8];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_cmd_cdm) == 120);
static_assert(sizeof(struct pvr_laguna_fwif_ccb_cmd_header) +
	      sizeof(struct pvr_laguna_fwif_cmd_cdm) == 136);
static_assert(offsetof(struct pvr_laguna_fwif_cmd_cdm, regs) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_cmd_cdm, flags) == 0x40);
static_assert(offsetof(struct pvr_laguna_fwif_cmd_cdm, max_num_temp_regions) == 0x48);
static_assert(offsetof(struct pvr_laguna_fwif_cmd_cdm, usc_tasks_stride) == 0x4c);
static_assert(offsetof(struct pvr_laguna_fwif_cmd_cdm, wgp_checksum) == 0x58);

/* DDK 25.3 Client CCB control structure size (32 bytes, RGXCreateCCB). */
#define PVR_LAGUNA_FWIF_CCCB_CTL_SIZE		32U

/*
 * DDK 25.3 Volcanic init config flags for Laguna (BVNC 71.2.2448.1212):
 *  - BIT(4):  POW_RASCALDUST (enable-firmware-managed-spu-power = 1)
 *  - BIT(6):  FBCDC_V3_1_EN (system FBCDC v3.1+ enabled)
 *  - BIT(10): pipelined datamasters init flag
 *  - 2 << 16: CTXSWITCH_PROFILE_MEDIUM
 * Note: BIT(2) (HWPERF_EN) stays clear while no HWPerf L1 buffer is allocated.
 */
#define PVR_LAGUNA_FWIF_INICFG_POW_RASCALDUST	BIT(4)
#define PVR_LAGUNA_FWIF_INICFG_FBCDC_V3_1_EN	BIT(6)
#define PVR_LAGUNA_FWIF_INICFG_PIPELINED_DM	BIT(10)
#define PVR_LAGUNA_FWIF_INICFG_DEFAULT		(PVR_LAGUNA_FWIF_INICFG_POW_RASCALDUST | \
						 PVR_LAGUNA_FWIF_INICFG_FBCDC_V3_1_EN | \
						 PVR_LAGUNA_FWIF_INICFG_PIPELINED_DM | \
						 (2U << 16))
#define PVR_LAGUNA_FWIF_OS_INICFG_DEFAULT	0x1fU
#define PVR_LAGUNA_ACTIVE_PM_LATENCY_MS		1U
#define PVR_LAGUNA_HWPERF_BVNC_FEATURE_FLAGS	0x0003c701U

/* DDK 25.3 OS-FW connection states in RGXFWIF_CONNECTION_CTL. */
#define PVR_LAGUNA_FW_CONN_OS_OFFLINE		0U
#define PVR_LAGUNA_FW_CONN_OS_READY		1U
#define PVR_LAGUNA_FW_CONN_OS_ACTIVE		2U

/* DDK 25.3 Volcanic MMU cache invalidation flags (_PrepareAndSubmitCacheCommand). */
#define PVR_LAGUNA_MMUCACHE_FLAGS_PT		0x1U
#define PVR_LAGUNA_MMUCACHE_FLAGS_PD		0x2U
#define PVR_LAGUNA_MMUCACHE_FLAGS_PC		0x4U
#define PVR_LAGUNA_MMUCACHE_FLAGS_INTERRUPT	0x4000000U

/* DDK 25.3 Volcanic Data Master IDs (matches RGXFWIF_DM). */
#define PVR_LAGUNA_DM_GP			0U
#define PVR_LAGUNA_DM_TDM			1U
#define PVR_LAGUNA_DM_GEOM			2U
#define PVR_LAGUNA_DM_3D			3U
#define PVR_LAGUNA_DM_CDM			4U
#define PVR_LAGUNA_DM_RAY			5U
#define PVR_LAGUNA_DM_MAX			9U

/* DDK 25.3 context compatibility flags. */
#define PVR_LAGUNA_CTX_COMPAT_HAS_DEFER_COUNT	BIT(1)

/* DDK 25.3 RGXFWIF_CLEANUP_TYPE (verified against RGXFWRequest*CleanUp). */
#define PVR_LAGUNA_CLEANUP_FWCOMMONCONTEXT	0U
#define PVR_LAGUNA_CLEANUP_HWRTDATA		1U
#define PVR_LAGUNA_CLEANUP_FREELIST		2U
#define PVR_LAGUNA_CLEANUP_ZSBUFFER		3U
#define PVR_LAGUNA_CLEANUP_HWRTDATA_AND_FL	4U

/* DDK 25.3 Volcanic 40-bit GPU execution heap bases passed in SYSINIT. */
#define PVR_LAGUNA_PDS_EXEC_BASE		0xDA00000000ULL
#define PVR_LAGUNA_USC_EXEC_BASE		0xE000000000ULL
#define PVR_LAGUNA_FBCDC_STATE_BASE		0xEC00000000ULL
#define PVR_LAGUNA_FBCDC_LARGE_STATE_BASE	0xEC40000000ULL
#define PVR_LAGUNA_PDS_INDIRECT_BASE		0xED00000000ULL
#define PVR_LAGUNA_TEXTURE_STATE_BASE		0xF000000000ULL

/* Laguna ARM arch-timer / GTC frequency (38.4 MHz). */
#define PVR_LAGUNA_GTC_FREQ_HZ			38400000U

/* Volcanic DXT-48-1536 SPU and RAC masks (2 SPUs, 2 RACs). */
#define PVR_LAGUNA_POW_UNITS_MASK		0x3U
#define PVR_LAGUNA_RAC_UNITS_MASK		0x3U

/* DDK 25.3 FW boot stage written to RGX_CR_SCRATCH14 (0x0870). */
#define PVR_LAGUNA_FW_BOOT_INIT_DONE		13U

/* Auxiliary FW structure sizes verified on ARM64 DDK 25.3 lga/production. */
#define PVR_LAGUNA_FWIF_HWRINFOBUF_SIZE		2464U
#define PVR_LAGUNA_FWIF_GPU_UTIL_FW_SIZE	10256U
#define PVR_LAGUNA_FWIF_HWPERFDATA_SIZE		40U
#define PVR_LAGUNA_FWIF_RUNTIME_HWPERFCFG_SIZE	40U
#define PVR_LAGUNA_FWIF_HWPERF_CTL_SIZE		4096U
#define PVR_LAGUNA_FWIF_TRACE_BUF_DWORDS	4096U

/* DDK 25.3 HWRINFOBUF counter offsets (2464 bytes total, 9 DMs). */
#define PVR_LAGUNA_HWRINFOBUF_LOCKUP_OFF	0x910U
#define PVR_LAGUNA_HWRINFOBUF_OVERRUN_OFF	0x934U
#define PVR_LAGUNA_HWRINFOBUF_REJECTED_OFF	0x958U
#define PVR_LAGUNA_HWRINFOBUF_WRITE_IDX_OFF	0x97cU

struct pvr_laguna_fwif_connection_ctl {
	u32 connection_fw_state;
	u32 connection_os_state;
	u32 alive_fw_token;
	u32 alive_os_token;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_connection_ctl) == 16);

struct pvr_laguna_fwif_ccb_ctl {
	u32 write_offset;
	u32 read_offset;
	u32 wrap_mask;
	u32 reserved;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_ccb_ctl) == 16);
static_assert(offsetof(struct pvr_laguna_fwif_ccb_ctl, read_offset) == 4);
static_assert(offsetof(struct pvr_laguna_fwif_ccb_ctl, wrap_mask) == 8);

struct pvr_laguna_fwif_osinit {
	u32 kernel_ccb_ctl;
	u32 kernel_ccb;
	u32 kernel_ccb_rtn_slots;
	u32 firmware_ccb_ctl;
	u32 firmware_ccb;
	u32 work_est_fw_ccb_ctl;
	u32 work_est_fw_ccb;
	u32 hwr_info_buf_ctl;
	u32 hwr_debug_dump_limit;
	u32 fw_os_data;
	u8 comp_checks[64];
	u8 multicore_info[8];
	u32 hwperf_data;
	u32 hwperf_runtime_cfg;
	u32 gpu_util_fw_ctl;
	u32 hwperf_ctl;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_osinit) == 128);
static_assert(offsetof(struct pvr_laguna_fwif_osinit, fw_os_data) == 0x24);
static_assert(offsetof(struct pvr_laguna_fwif_osinit, comp_checks) == 0x28);
static_assert(offsetof(struct pvr_laguna_fwif_osinit, hwperf_data) == 0x70);
static_assert(offsetof(struct pvr_laguna_fwif_osinit, hwperf_ctl) == 0x7c);

struct pvr_laguna_fwif_dma_addr {
	u64 dev_addr;
	u32 fw_addr;
	u32 pad_0c;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_dma_addr) == 16);
static_assert(offsetof(struct pvr_laguna_fwif_dma_addr, fw_addr) == 8);

/* DDK 25.3 FW HWPerf data header (40 bytes, RGXSetupFirmware). */
struct pvr_laguna_fwif_hwperf_data {
	u32 ridx;
	u32 widx;
	u32 wrap_count;
	u32 buf_size;
	u32 drop_count;
	u32 reserved_14[3];
	u64 filter;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_hwperf_data) == PVR_LAGUNA_FWIF_HWPERFDATA_SIZE);
static_assert(offsetof(struct pvr_laguna_fwif_hwperf_data, buf_size) == 0x0c);
static_assert(offsetof(struct pvr_laguna_fwif_hwperf_data, filter) == 0x20);

/* DDK 25.3 FW HWPerf runtime config (40 bytes, META DMA addr at +0x18). */
struct pvr_laguna_fwif_hwperf_rtcfg {
	u64 reserved_00;
	struct pvr_laguna_fwif_dma_addr l1_buf_dma_addr;
	struct pvr_laguna_fwif_dma_addr hwperf_ctl_dma_addr;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_hwperf_rtcfg) ==
	      PVR_LAGUNA_FWIF_RUNTIME_HWPERFCFG_SIZE);
static_assert(offsetof(struct pvr_laguna_fwif_hwperf_rtcfg, l1_buf_dma_addr) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_hwperf_rtcfg, hwperf_ctl_dma_addr) == 0x18);

struct pvr_laguna_fwif_sysinit {
	u64 fault_phys_addr;
	u64 pds_exec_base;
	u64 usc_exec_base;
	u64 fbcdc_state_table_base;
	u64 fbcdc_large_state_table_base;
	u64 texture_heap_base;
	u64 pds_indirect_heap_base;
	u64 customer_reg_base;
	u64 uvbrm_num_regions[2];
	u32 tpu_trilinear_frac_mask[5];
	u32 pad_64;
	u32 usrm_num_regions[5];
	u32 pad_7c;
	u64 clk_ctrl0;
	u64 clk_ctrl1;
	u32 clk_ctrl2;
	u32 filter_flags;
	u32 runtime_cfg;
	u32 trace_buf_ctl;
	u32 fw_sys_data;
	u32 reg_cfg;
	u32 counter_dump_buf;
	u32 counter_dump_size_dwords;
	u32 align_checks;
	u32 initial_core_clock_speed;
	u32 initial_soc_clock_speed;
	u32 initial_active_pm_latency_ms;
	u8 firmware_started;
	u8 pad_c1[3];
	u8 remap_tables[132];
	u32 marker_val;
	u32 firmware_started_timestamp;
	struct pvr_laguna_fwif_dma_addr coremem_data_store;
	u32 firmware_perf;
	u32 pad_164;
	u8 opp_info[384];
	u32 opp_reserved;
	u32 gpio_validation_mode;
	u64 pad_2f0;
	u8 bvnc_km_feature_flags[192];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_sysinit) == 952);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, clk_ctrl0) == 0x80);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, runtime_cfg) == 0x98);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, trace_buf_ctl) == 0x9c);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, fw_sys_data) == 0xa0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, align_checks) == 0xb0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, firmware_started) == 0xc0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, marker_val) == 0x148);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, coremem_data_store) == 0x150);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, firmware_perf) == 0x160);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, opp_info) == 0x168);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, gpio_validation_mode) == 0x2ec);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, bvnc_km_feature_flags) == 0x2f8);

struct pvr_laguna_fwif_osdata {
	u32 fw_os_config_flags;
	u8 reserved_04[0x22c];
	u32 interrupt_count;
	u32 kccb_cmds_executed;
	u32 power_sync;
	u8 reserved_23c[0x20c];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_osdata) == 1096);
static_assert(offsetof(struct pvr_laguna_fwif_osdata, interrupt_count) == 0x230);
static_assert(offsetof(struct pvr_laguna_fwif_osdata, kccb_cmds_executed) == 0x234);
static_assert(offsetof(struct pvr_laguna_fwif_osdata, power_sync) == 0x238);

#define PVR_LAGUNA_FWFAULTINFO_MAX		8U

struct pvr_laguna_fwif_fault_info {
	u64 cr_timer;
	u64 os_timer;
	u64 data;
	char assert_path[200];
	char assert_info[200];
	u32 line_num;
	u32 pad_1ac;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fault_info) == 0x1b0);
static_assert(offsetof(struct pvr_laguna_fwif_fault_info, assert_path) == 0x18);
static_assert(offsetof(struct pvr_laguna_fwif_fault_info, assert_info) == 0xe0);
static_assert(offsetof(struct pvr_laguna_fwif_fault_info, line_num) == 0x1a8);

struct pvr_laguna_fwif_sysdata {
	u32 config_flags;
	u32 config_flags_ext;
	u32 pow_state;
	u8 reserved_0c[0x24];
	struct pvr_laguna_fwif_fault_info fault_info[PVR_LAGUNA_FWFAULTINFO_MAX];
	u32 fw_faults;
	u32 cr_poll_addr;
	u32 cr_poll_mask;
	u8 reserved_dbc[0x2c];
	u32 hwr_state_flags;
	u8 reserved_dec[0x144];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_sysdata) == 3888);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, pow_state) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, fault_info) == 0x30);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, fw_faults) == 0xdb0);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, cr_poll_addr) == 0xdb4);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, cr_poll_mask) == 0xdb8);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, hwr_state_flags) == 0xde8);

struct pvr_laguna_fwif_runtime_cfg {
	u32 active_pm_latency_ms;
	u32 reserved_04;
	u8 active_pm_latency_persistant;
	u8 pad_09[3];
	u32 core_clock_speed;
	u32 soc_clock_speed;
	u32 pow_units_state;
	u32 rac_units_state;
	u32 reserved_1c;
	u32 hcs_deadline_ms;
	u32 wdg_period_us;
	u8 reserved_28[0x70];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_runtime_cfg) == 152);
static_assert(offsetof(struct pvr_laguna_fwif_runtime_cfg, core_clock_speed) == 0x0c);
static_assert(offsetof(struct pvr_laguna_fwif_runtime_cfg, pow_units_state) == 0x14);
static_assert(offsetof(struct pvr_laguna_fwif_runtime_cfg, hcs_deadline_ms) == 0x20);

struct pvr_laguna_fwif_tracebuf_space {
	u32 trace_pointer;
	u32 wrap_count;
	u32 trace_buffer_fw_addr;
	u32 pad_0c;
	char assert_path[200];
	char assert_info[200];
	u32 assert_line;
	u32 assert_pad;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_tracebuf_space) == 424);

struct pvr_laguna_fwif_hwrinfobuf {
	u8 hwr_info[PVR_LAGUNA_HWRINFOBUF_LOCKUP_OFF];
	u32 dm_lockup_cnt[PVR_LAGUNA_DM_MAX];
	u32 dm_overrun_cnt[PVR_LAGUNA_DM_MAX];
	u32 dm_rejected_cnt[PVR_LAGUNA_DM_MAX];
	u32 write_index[PVR_LAGUNA_DM_MAX];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_hwrinfobuf) == PVR_LAGUNA_FWIF_HWRINFOBUF_SIZE);
static_assert(offsetof(struct pvr_laguna_fwif_hwrinfobuf, dm_lockup_cnt) ==
	      PVR_LAGUNA_HWRINFOBUF_LOCKUP_OFF);
static_assert(offsetof(struct pvr_laguna_fwif_hwrinfobuf, dm_overrun_cnt) ==
	      PVR_LAGUNA_HWRINFOBUF_OVERRUN_OFF);
static_assert(offsetof(struct pvr_laguna_fwif_hwrinfobuf, dm_rejected_cnt) ==
	      PVR_LAGUNA_HWRINFOBUF_REJECTED_OFF);
static_assert(offsetof(struct pvr_laguna_fwif_hwrinfobuf, write_index) ==
	      PVR_LAGUNA_HWRINFOBUF_WRITE_IDX_OFF);

/* DDK 25.3 Volcanic RGXFWIF_FREELIST (72 bytes, verified against RGXCreateFreeList). */
struct pvr_laguna_fwif_freelist {
	u64 freelist_dev_addr;
	u64 current_dev_addr;
	u64 current_stack_top;
	u32 max_pages;
	u32 grow_pages;
	u32 current_pages;
	u32 allocated_page_count;
	u32 freelist_id;
	u32 allocated_mmu_page_count;
	u8 grow_pending;
	u8 pad_31[3];
	u32 ready_pages;
	u32 freelist_flags;
	u8 pm_global_pb;
	u8 pad_3d[3];
	u64 cleanup_state;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_freelist) == 72);
static_assert(offsetof(struct pvr_laguna_fwif_freelist, max_pages) == 0x18);
static_assert(offsetof(struct pvr_laguna_fwif_freelist, freelist_id) == 0x28);
static_assert(offsetof(struct pvr_laguna_fwif_freelist, ready_pages) == 0x34);
static_assert(offsetof(struct pvr_laguna_fwif_freelist, cleanup_state) == 0x40);

struct pvr_laguna_fwif_tracebuf {
	u32 log_type;
	u32 pad_04;
	struct pvr_laguna_fwif_tracebuf_space tracebuf;
	u32 tracebuf_size_in_dwords;
	u32 tracebuf_flags;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_tracebuf) == 440);
static_assert(offsetof(struct pvr_laguna_fwif_tracebuf, tracebuf) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_tracebuf, tracebuf_size_in_dwords) == 0x1b0);

struct pvr_laguna_kccb_cmd_kick {
	u32 context_fw_addr;
	u32 cwoff_update;
	u32 cwrap_mask_update;
	u32 num_cleanup_ctl;
	u32 cleanup_ctl[3];
};

static_assert(sizeof(struct pvr_laguna_kccb_cmd_kick) == 28);

struct pvr_laguna_kccb_cmd_mmucache {
	u32 cache_flags;
	u32 mmu_cache_sync_fw_addr;
	u32 mmu_cache_sync_update_value;
};

static_assert(sizeof(struct pvr_laguna_kccb_cmd_mmucache) == 12);

struct pvr_laguna_kccb_cmd_cleanup {
	u32 cleanup_type;
	u32 resource_fw_addr;
};

struct pvr_laguna_kccb_cmd_pow {
	u32 pow_type;
	u32 arg0;
	u32 arg1;
};

struct pvr_laguna_fwif_kccb_cmd {
	u32 cmd_type;
	u32 kccb_flags;
	union {
		struct pvr_laguna_kccb_cmd_kick kick;
		struct pvr_laguna_kccb_cmd_mmucache mmucache;
		struct pvr_laguna_kccb_cmd_cleanup cleanup;
		struct pvr_laguna_kccb_cmd_pow pow;
		u8 raw[104];
	} data __aligned(8);
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_kccb_cmd) == 112);
static_assert(offsetof(struct pvr_laguna_fwif_kccb_cmd, data) == 8);

struct pvr_laguna_fwif_fwccb_zsbuf {
	u32 zs_buffer_id;
} __aligned(8);

struct pvr_laguna_fwif_fwccb_fl_grow {
	u32 freelist_id;
	u32 pad_04;
} __aligned(8);

struct pvr_laguna_fwif_fwccb_fl_reconstruct {
	u32 freelist_count;
	u32 pad_04;
	u32 freelist_ids[24];
} __aligned(8);

struct pvr_laguna_fwif_fwccb_ctx_reset {
	u32 server_common_context_id;
	u32 reset_reason;
	u32 dm;
	u32 reset_job_ref;
	u32 flags;
	u32 pad_14;
	u64 pc_address;
	u64 mmu_status[2];
} __aligned(8);

struct pvr_laguna_fwif_fwccb_update_stats {
	u32 element_to_update;
	s32 adjustment_value;
	u32 pid_owner;
} __aligned(8);

struct pvr_laguna_fwif_fwccb_coreclk {
	u32 core_clk_rate;
} __aligned(8);

struct pvr_laguna_fwif_fwccb_fw_pf {
	u64 fw_fault_addr;
} __aligned(8);

struct pvr_laguna_fwif_fwccb_cmd {
	u32 cmd_type;
	u32 fwccb_flags;
	union {
		struct pvr_laguna_fwif_fwccb_zsbuf zsbuf;
		struct pvr_laguna_fwif_fwccb_fl_grow fl_grow;
		struct pvr_laguna_fwif_fwccb_fl_reconstruct fl_reconstruct;
		struct pvr_laguna_fwif_fwccb_ctx_reset ctx_reset;
		struct pvr_laguna_fwif_fwccb_update_stats update_stats;
		struct pvr_laguna_fwif_fwccb_coreclk coreclk;
		struct pvr_laguna_fwif_fwccb_fw_pf fw_pf;
		u8 raw[104];
	} data __aligned(8);
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwccb_cmd) == 112);
static_assert(offsetof(struct pvr_laguna_fwif_fwccb_cmd, data) == 8);

/* DDK 25.3 Volcanic RGXFWIF_FWMEMCONTEXT (DUAL_MEMCTX enabled on 71.2.2448.1212). */
struct pvr_laguna_fwif_fwmemcontext {
	u64 pc_dev_paddr;
	u32 page_cat_base_reg_set;
	u32 breakpoint_addr;
	u32 bp_handler_addr;
	u32 breakpoint_ctl;
	u64 fbc_state_id_mask;
	u64 spill_addr;
	u32 fw_mem_ctx_flags;
	u8 cross_dm_wait_sync_update[PVR_LAGUNA_DM_MAX];
	u8 pad_35[3];
	u64 pc_pm_dev_paddr;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwmemcontext) == 64);
static_assert(offsetof(struct pvr_laguna_fwif_fwmemcontext, page_cat_base_reg_set) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_fwmemcontext, fbc_state_id_mask) == 0x18);
static_assert(offsetof(struct pvr_laguna_fwif_fwmemcontext, fw_mem_ctx_flags) == 0x28);
static_assert(offsetof(struct pvr_laguna_fwif_fwmemcontext, pc_pm_dev_paddr) == 0x38);

/* DDK 25.3 Volcanic RGXFWIF_COMPUTECTX_STATE. */
struct pvr_laguna_fwif_computectx_state {
	u32 ctx_state_flags;
	u32 pad_04;
	u64 enabled_units_mask;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_computectx_state) == 16);
static_assert(offsetof(struct pvr_laguna_fwif_computectx_state, enabled_units_mask) == 0x08);

/* DDK 25.3 Volcanic RGXFWIF_CDM_REGISTERS_CSWITCH (SUPPORT_TRUSTED_DEVICE). */
struct pvr_laguna_fwif_cdm_regs_cswitch {
	u64 cdm_context_pds0;
	u64 cdm_context_pds1;
	u64 cdm_terminate_pds;
	u64 cdm_terminate_pds1;
	u64 cdm_resume_pds0;
	u64 cdm_context_pds0_b;
	u64 cdm_resume_pds0_b;
	u64 cdm_context_secure_pds0;
	u64 cdm_context_secure_pds0_b;
	u64 cdm_resume_secure_pds0;
	u64 cdm_resume_secure_pds0_b;
	u64 reserved_58;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_cdm_regs_cswitch) == 96);

/* DDK 25.3 Volcanic RGXFWIF_FWCOMMONCONTEXT (488 bytes on lga/production). */
struct pvr_laguna_fwif_fwcommoncontext {
	u32 cccb_ctl_fw_addr;
	u32 cccb_fw_addr;
	struct pvr_laguna_fwif_dma_addr ccb_meta_dma_addr;
	u32 context_state_fw_addr;
	u32 fw_com_ctx_flags;
	s32 priority;
	u32 priority_seq_num;
	u32 rf_cmd_fw_addr;
	u32 compat_flags;
	s32 stats_num_stores;
	u32 pad_34;
	u32 buf_stalled_node[2];
	u64 cbuf_queue_ctrl_addr;
	u64 robustness_address;
	u32 max_deadline_ms;
	u8 read_offset_needs_reset;
	u8 pad_55[3];
	u32 waiting_node[2];
	u32 run_node[2];
	u32 last_failed_ufo[2];
	u32 fw_mem_context_fw_addr;
	u32 server_common_context_id;
	u32 pid;
	u32 pipelined_kicks;
	char proc_name[16];
	u32 defer_count;
	u8 cancel_and_flags[0x44];
	u32 uid;
	u32 cross_dm_update_cmd_offset;
	u32 cross_dm_task_cmd_offset;
	u32 work_est_ccb_submitted;
	u32 dm;
	u32 unpause_dm;
	union {
		struct {
			struct pvr_laguna_fwif_cdm_regs_cswitch ctxswitch_regs;
			u32 wgp_checksum[8];
		} comp;
		u8 raw[248];
	} dm_specific __aligned(8);
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwcommoncontext) == 488);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, cccb_fw_addr) == 0x04);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, context_state_fw_addr) == 0x18);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, priority) == 0x20);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, rf_cmd_fw_addr) == 0x28);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, compat_flags) == 0x2c);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, robustness_address) == 0x48);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, max_deadline_ms) == 0x50);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, run_node) == 0x60);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, fw_mem_context_fw_addr) == 0x70);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, server_common_context_id) == 0x74);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, pid) == 0x78);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, proc_name) == 0x80);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, uid) == 0xd8);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, dm) == 0xe8);
static_assert(offsetof(struct pvr_laguna_fwif_fwcommoncontext, dm_specific.comp.ctxswitch_regs) ==
	      0xf0);

struct pvr_laguna_fwif_fwcomputecontext {
	struct pvr_laguna_fwif_fwcommoncontext cdm_context;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwcomputecontext) == 488);

struct pvr_laguna_fwif_fwrendercontext {
	struct pvr_laguna_fwif_fwcommoncontext geom_context;
	struct pvr_laguna_fwif_fwcommoncontext frag_context;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwrendercontext) == 976);
static_assert(offsetof(struct pvr_laguna_fwif_fwrendercontext, frag_context) == 488);

struct pvr_laguna_fwif_fwtransfercontext {
	struct pvr_laguna_fwif_fwcommoncontext tdm_context;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwtransfercontext) == 488);

/* DDK 25.3 KM alignment check table (28 KM values + UM placeholder). */
#define PVR_LAGUNA_ALIGN_CHECKS_KM_COUNT	28U
#define PVR_LAGUNA_ALIGN_CHECKS_UM_MAX		128U
#define PVR_LAGUNA_ALIGN_CHECKS_TOTAL_SIZE \
	((PVR_LAGUNA_ALIGN_CHECKS_KM_COUNT + PVR_LAGUNA_ALIGN_CHECKS_UM_MAX + 2U) * sizeof(u32))

#endif /* PVR_LAGUNA_FWIF_H */
