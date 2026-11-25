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

/* DDK 25.3 RGXFWIF_CLEANUP_TYPE (differs from upstream at FREELIST = 3). */
#define PVR_LAGUNA_CLEANUP_FWCOMMONCONTEXT	0U
#define PVR_LAGUNA_CLEANUP_HWRTDATA		1U
#define PVR_LAGUNA_CLEANUP_HWRTDATA_AND_FL	2U
#define PVR_LAGUNA_CLEANUP_FREELIST		3U
#define PVR_LAGUNA_CLEANUP_ZSBUFFER		4U

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
#define PVR_LAGUNA_FWIF_RUNTIME_HWPERFCFG_SIZE	24U
#define PVR_LAGUNA_FWIF_HWPERF_CTL_SIZE		4096U
#define PVR_LAGUNA_FWIF_TRACE_BUF_DWORDS	4096U

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
	u64 coremem_data_store_deprecated;
	u32 firmware_perf;
	u32 pad_15c;
	u8 opp_info[392];
	u32 opp_reserved;
	u32 gpio_validation_mode;
	u8 bvnc_km_feature_flags[192];
	u64 uvb_spill_mem_ctx_cat_base;
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_sysinit) == 952);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, clk_ctrl0) == 0x80);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, runtime_cfg) == 0x98);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, trace_buf_ctl) == 0x9c);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, fw_sys_data) == 0xa0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, align_checks) == 0xb0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, firmware_started) == 0xc0);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, marker_val) == 0x148);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, opp_info) == 0x160);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, gpio_validation_mode) == 0x2ec);
static_assert(offsetof(struct pvr_laguna_fwif_sysinit, uvb_spill_mem_ctx_cat_base) == 0x3b0);

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

struct pvr_laguna_fwif_sysdata {
	u32 config_flags;
	u32 config_flags_ext;
	u32 pow_state;
	u8 reserved_0c[0xda4];
	u32 fw_faults;
	u8 reserved_db4[0x34];
	u32 hwr_state_flags;
	u8 reserved_dec[0x144];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_sysdata) == 3888);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, pow_state) == 0x08);
static_assert(offsetof(struct pvr_laguna_fwif_sysdata, fw_faults) == 0xdb0);
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
	u8 assert_buf[408];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_tracebuf_space) == 424);

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
	u32 cleanup_ctl[4];
};

struct pvr_laguna_kccb_cmd_mmucache {
	u32 cache_flags;
	u32 mmu_cache_sync_fw_addr;
	u32 mmu_cache_sync_update_value;
};

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

struct pvr_laguna_fwif_fwccb_cmd {
	u32 cmd_type;
	u32 fwccb_flags;
	u8 data[104];
} __aligned(8);
static_assert(sizeof(struct pvr_laguna_fwif_fwccb_cmd) == 112);

#endif /* PVR_LAGUNA_FWIF_H */
