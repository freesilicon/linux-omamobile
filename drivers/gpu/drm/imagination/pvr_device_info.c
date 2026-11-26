// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright (c) 2023 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_device_info.h"
#include "pvr_rogue_fwif_dev_info.h"
#include "pvr_volcanic_defs.h"

#include <drm/drm_print.h>

#include <linux/bits.h>
#include <linux/minmax.h>
#include <linux/stddef.h>
#include <linux/types.h>

#define QUIRK_MAPPING(quirk) \
	[PVR_FW_HAS_BRN_##quirk] = offsetof(struct pvr_device, quirks.has_brn##quirk)

static const uintptr_t quirks_mapping[] = {
	QUIRK_MAPPING(44079),
	QUIRK_MAPPING(47217),
	QUIRK_MAPPING(48492),
	QUIRK_MAPPING(48545),
	QUIRK_MAPPING(49927),
	QUIRK_MAPPING(50767),
	QUIRK_MAPPING(51764),
	QUIRK_MAPPING(62269),
	QUIRK_MAPPING(63142),
	QUIRK_MAPPING(63553),
	QUIRK_MAPPING(65104),
	QUIRK_MAPPING(66011),
	QUIRK_MAPPING(69700),
	QUIRK_MAPPING(71242),
	QUIRK_MAPPING(74812),
	QUIRK_MAPPING(77899),
};

#undef QUIRK_MAPPING

#define ENHANCEMENT_MAPPING(enhancement)                             \
	[PVR_FW_HAS_ERN_##enhancement] = offsetof(struct pvr_device, \
						  enhancements.has_ern##enhancement)

static const uintptr_t enhancements_mapping[] = {
	ENHANCEMENT_MAPPING(35421),
	ENHANCEMENT_MAPPING(38020),
	ENHANCEMENT_MAPPING(38748),
	ENHANCEMENT_MAPPING(42064),
	ENHANCEMENT_MAPPING(42290),
	ENHANCEMENT_MAPPING(42606),
	ENHANCEMENT_MAPPING(47025),
	ENHANCEMENT_MAPPING(57596),
};

#undef ENHANCEMENT_MAPPING

static void pvr_device_info_set_common(struct pvr_device *pvr_dev, const u64 *bitmask,
				       u32 bitmask_size, const uintptr_t *mapping, u32 mapping_max)
{
	const u32 mapping_max_size = (mapping_max + 63) >> 6;
	const u32 nr_bits = min(bitmask_size * 64, mapping_max);

	/* Warn if any unsupported values in the bitmask. */
	if (bitmask_size > mapping_max_size) {
		if (mapping == quirks_mapping)
			drm_warn(from_pvr_device(pvr_dev), "Unsupported quirks in firmware image");
		else
			drm_warn(from_pvr_device(pvr_dev),
				 "Unsupported enhancements in firmware image");
	} else if (bitmask_size == mapping_max_size && (mapping_max & 63)) {
		u64 invalid_mask = ~0ull << (mapping_max & 63);

		if (bitmask[bitmask_size - 1] & invalid_mask) {
			if (mapping == quirks_mapping)
				drm_warn(from_pvr_device(pvr_dev),
					 "Unsupported quirks in firmware image");
			else
				drm_warn(from_pvr_device(pvr_dev),
					 "Unsupported enhancements in firmware image");
		}
	}

	for (u32 i = 0; i < nr_bits; i++) {
		if (bitmask[i >> 6] & BIT_ULL(i & 63))
			*(bool *)((u8 *)pvr_dev + mapping[i]) = true;
	}
}

/**
 * pvr_device_info_set_quirks() - Set device quirks from device information in firmware
 * @pvr_dev: Device pointer.
 * @quirks: Pointer to quirks mask in device information.
 * @quirks_size: Size of quirks mask, in u64s.
 */
void pvr_device_info_set_quirks(struct pvr_device *pvr_dev, const u64 *quirks, u32 quirks_size)
{
	BUILD_BUG_ON(ARRAY_SIZE(quirks_mapping) != PVR_FW_HAS_BRN_MAX);

	pvr_device_info_set_common(pvr_dev, quirks, quirks_size, quirks_mapping,
				   ARRAY_SIZE(quirks_mapping));
}

/**
 * pvr_device_info_set_enhancements() - Set device enhancements from device information in firmware
 * @pvr_dev: Device pointer.
 * @enhancements: Pointer to enhancements mask in device information.
 * @enhancements_size: Size of enhancements mask, in u64s.
 */
void pvr_device_info_set_enhancements(struct pvr_device *pvr_dev, const u64 *enhancements,
				      u32 enhancements_size)
{
	BUILD_BUG_ON(ARRAY_SIZE(enhancements_mapping) != PVR_FW_HAS_ERN_MAX);

	pvr_device_info_set_common(pvr_dev, enhancements, enhancements_size,
				   enhancements_mapping, ARRAY_SIZE(enhancements_mapping));
}

#define FEATURE_MAPPING(fw_feature, feature)                                        \
	[PVR_FW_HAS_FEATURE_##fw_feature] = {                                       \
		.flag_offset = offsetof(struct pvr_device, features.has_##feature), \
		.value_offset = 0                                                   \
	}

#define FEATURE_MAPPING_VALUE(fw_feature, feature)                                  \
	[PVR_FW_HAS_FEATURE_##fw_feature] = {                                       \
		.flag_offset = offsetof(struct pvr_device, features.has_##feature), \
		.value_offset = offsetof(struct pvr_device, features.feature)       \
	}

static const struct {
	uintptr_t flag_offset;
	uintptr_t value_offset;
} features_mapping[] = {
	FEATURE_MAPPING(AXI_ACELITE, axi_acelite),
	FEATURE_MAPPING_VALUE(CDM_CONTROL_STREAM_FORMAT, cdm_control_stream_format),
	FEATURE_MAPPING(CLUSTER_GROUPING, cluster_grouping),
	FEATURE_MAPPING_VALUE(COMMON_STORE_SIZE_IN_DWORDS, common_store_size_in_dwords),
	FEATURE_MAPPING(COMPUTE, compute),
	FEATURE_MAPPING(COMPUTE_MORTON_CAPABLE, compute_morton_capable),
	FEATURE_MAPPING(COMPUTE_OVERLAP, compute_overlap),
	FEATURE_MAPPING(COREID_PER_OS, coreid_per_os),
	FEATURE_MAPPING(DYNAMIC_DUST_POWER, dynamic_dust_power),
	FEATURE_MAPPING_VALUE(ECC_RAMS, ecc_rams),
	FEATURE_MAPPING_VALUE(FBCDC, fbcdc),
	FEATURE_MAPPING_VALUE(FBCDC_ALGORITHM, fbcdc_algorithm),
	FEATURE_MAPPING_VALUE(FBCDC_ARCHITECTURE, fbcdc_architecture),
	FEATURE_MAPPING_VALUE(FBC_MAX_DEFAULT_DESCRIPTORS, fbc_max_default_descriptors),
	FEATURE_MAPPING_VALUE(FBC_MAX_LARGE_DESCRIPTORS, fbc_max_large_descriptors),
	FEATURE_MAPPING(FB_CDC_V4, fb_cdc_v4),
	FEATURE_MAPPING(GPU_MULTICORE_SUPPORT, gpu_multicore_support),
	FEATURE_MAPPING(GPU_VIRTUALISATION, gpu_virtualisation),
	FEATURE_MAPPING(GS_RTA_SUPPORT, gs_rta_support),
	FEATURE_MAPPING(IRQ_PER_OS, irq_per_os),
	FEATURE_MAPPING_VALUE(ISP_MAX_TILES_IN_FLIGHT, isp_max_tiles_in_flight),
	FEATURE_MAPPING_VALUE(ISP_SAMPLES_PER_PIXEL, isp_samples_per_pixel),
	FEATURE_MAPPING(ISP_ZLS_D24_S8_PACKING_OGL_MODE, isp_zls_d24_s8_packing_ogl_mode),
	FEATURE_MAPPING_VALUE(LAYOUT_MARS, layout_mars),
	FEATURE_MAPPING_VALUE(MAX_PARTITIONS, max_partitions),
	FEATURE_MAPPING_VALUE(META, meta),
	FEATURE_MAPPING_VALUE(META_COREMEM_SIZE, meta_coremem_size),
	FEATURE_MAPPING(MIPS, mips),
	FEATURE_MAPPING_VALUE(NUM_CLUSTERS, num_clusters),
	FEATURE_MAPPING_VALUE(NUM_ISP_IPP_PIPES, num_isp_ipp_pipes),
	FEATURE_MAPPING_VALUE(NUM_OSIDS, num_osids),
	FEATURE_MAPPING_VALUE(NUM_RASTER_PIPES, num_raster_pipes),
	FEATURE_MAPPING(PBE2_IN_XE, pbe2_in_xe),
	FEATURE_MAPPING(PBVNC_COREID_REG, pbvnc_coreid_reg),
	FEATURE_MAPPING(PERFBUS, perfbus),
	FEATURE_MAPPING(PERF_COUNTER_BATCH, perf_counter_batch),
	FEATURE_MAPPING_VALUE(PHYS_BUS_WIDTH, phys_bus_width),
	FEATURE_MAPPING(RISCV_FW_PROCESSOR, riscv_fw_processor),
	FEATURE_MAPPING(ROGUEXE, roguexe),
	FEATURE_MAPPING(S7_TOP_INFRASTRUCTURE, s7_top_infrastructure),
	FEATURE_MAPPING(SIMPLE_INTERNAL_PARAMETER_FORMAT, simple_internal_parameter_format),
	FEATURE_MAPPING(SIMPLE_INTERNAL_PARAMETER_FORMAT_V2, simple_internal_parameter_format_v2),
	FEATURE_MAPPING_VALUE(SIMPLE_PARAMETER_FORMAT_VERSION, simple_parameter_format_version),
	FEATURE_MAPPING_VALUE(SLC_BANKS, slc_banks),
	FEATURE_MAPPING_VALUE(SLC_CACHE_LINE_SIZE_BITS, slc_cache_line_size_bits),
	FEATURE_MAPPING(SLC_SIZE_CONFIGURABLE, slc_size_configurable),
	FEATURE_MAPPING_VALUE(SLC_SIZE_IN_KILOBYTES, slc_size_in_kilobytes),
	FEATURE_MAPPING(SOC_TIMER, soc_timer),
	FEATURE_MAPPING(SYS_BUS_SECURE_RESET, sys_bus_secure_reset),
	FEATURE_MAPPING(TESSELLATION, tessellation),
	FEATURE_MAPPING(TILE_REGION_PROTECTION, tile_region_protection),
	FEATURE_MAPPING_VALUE(TILE_SIZE_X, tile_size_x),
	FEATURE_MAPPING_VALUE(TILE_SIZE_Y, tile_size_y),
	FEATURE_MAPPING(TLA, tla),
	FEATURE_MAPPING(TPU_CEM_DATAMASTER_GLOBAL_REGISTERS, tpu_cem_datamaster_global_registers),
	FEATURE_MAPPING(TPU_DM_GLOBAL_REGISTERS, tpu_dm_global_registers),
	FEATURE_MAPPING(TPU_FILTERING_MODE_CONTROL, tpu_filtering_mode_control),
	FEATURE_MAPPING_VALUE(USC_MIN_OUTPUT_REGISTERS_PER_PIX, usc_min_output_registers_per_pix),
	FEATURE_MAPPING(VDM_DRAWINDIRECT, vdm_drawindirect),
	FEATURE_MAPPING(VDM_OBJECT_LEVEL_LLS, vdm_object_level_lls),
	FEATURE_MAPPING_VALUE(VIRTUAL_ADDRESS_SPACE_BITS, virtual_address_space_bits),
	FEATURE_MAPPING(WATCHDOG_TIMER, watchdog_timer),
	FEATURE_MAPPING(WORKGROUP_PROTECTION, workgroup_protection),
	FEATURE_MAPPING_VALUE(XE_ARCHITECTURE, xe_architecture),
	FEATURE_MAPPING(XE_MEMORY_HIERARCHY, xe_memory_hierarchy),
	FEATURE_MAPPING(XE_TPU2, xe_tpu2),
	FEATURE_MAPPING_VALUE(XPU_MAX_REGBANKS_ADDR_WIDTH, xpu_max_regbanks_addr_width),
	FEATURE_MAPPING_VALUE(XPU_MAX_SLAVES, xpu_max_slaves),
	FEATURE_MAPPING_VALUE(XPU_REGISTER_BROADCAST, xpu_register_broadcast),
	FEATURE_MAPPING(XT_TOP_INFRASTRUCTURE, xt_top_infrastructure),
	FEATURE_MAPPING(ZLS_SUBTILE, zls_subtile),
};

#undef FEATURE_MAPPING_VALUE
#undef FEATURE_MAPPING

/**
 * pvr_device_info_set_features() - Set device features from device information in firmware
 * @pvr_dev: Device pointer.
 * @features: Pointer to features mask in device information.
 * @features_size: Size of features mask, in u64s.
 * @feature_param_size: Size of feature parameters, in u64s.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL on malformed stream.
 */
int pvr_device_info_set_features(struct pvr_device *pvr_dev, const u64 *features, u32 features_size,
				 u32 feature_param_size)
{
	const u32 mapping_max = ARRAY_SIZE(features_mapping);
	const u32 mapping_max_size = (mapping_max + 63) >> 6;
	const u32 nr_bits = min(features_size * 64, mapping_max);
	const u64 *feature_params = features + features_size;
	u32 param_idx = 0;

	BUILD_BUG_ON(ARRAY_SIZE(features_mapping) != PVR_FW_HAS_FEATURE_MAX);

	/* Verify no unsupported values in the bitmask. */
	if (features_size > mapping_max_size) {
		drm_warn(from_pvr_device(pvr_dev), "Unsupported features in firmware image");
	} else if (features_size == mapping_max_size &&
		   ((mapping_max & 63) != 0)) {
		u64 invalid_mask = ~0ull << (mapping_max & 63);

		if (features[features_size - 1] & invalid_mask)
			drm_warn(from_pvr_device(pvr_dev),
				 "Unsupported features in firmware image");
	}

	for (u32 i = 0; i < nr_bits; i++) {
		if (features[i >> 6] & BIT_ULL(i & 63)) {
			*(bool *)((u8 *)pvr_dev + features_mapping[i].flag_offset) = true;

			if (features_mapping[i].value_offset) {
				if (param_idx >= feature_param_size)
					return -EINVAL;

				*(u64 *)((u8 *)pvr_dev + features_mapping[i].value_offset) =
					feature_params[param_idx];
				param_idx++;
			}
		}
	}

	return 0;
}

struct pvr_volcanic_bvnc_entry {
	u64 packed_bvnc;
	struct pvr_device_features features;
	struct pvr_device_quirks quirks;
};

static const struct pvr_volcanic_bvnc_entry pvr_volcanic_bvnc_table[] = {
	{
		/* Tensor G5 (Laguna) PowerVR Volcanic DXT-48-1536 (71.2.2448.1212) */
		.packed_bvnc = PVR_VOLCANIC_DXT_48_1536_BVNC,
		.features = {
			.is_volcanic = true,
			.has_cdm_control_stream_format = true,
			.has_common_store_size_in_dwords = true,
			.has_compute = true,
			.has_compute_overlap = true,
			.has_coreid_per_os = true,
			.has_fbcdc = true,
			.has_fbcdc_algorithm = true,
			.has_fbcdc_architecture = true,
			.has_fbc_max_default_descriptors = true,
			.has_fbc_max_large_descriptors = true,
			.has_gpu_virtualisation = true,
			.has_host_security_version = true,
			.has_irq_per_os = true,
			.has_isp_max_tiles_in_flight = true,
			.has_max_partitions = true,
			.has_mmu_version = true,
			.has_num_clusters = true,
			.has_num_isp_ipp_pipes = true,
			.has_num_osids = true,
			.has_num_raster_pipes = true,
			.has_num_spus = true,
			.has_pbvnc_coreid_reg = true,
			.has_phys_bus_width = true,
			.has_ray_tracing_arch = true,
			.has_riscv_fw_processor = true,
			.has_s7_top_infrastructure = true,
			.has_slc_banks = true,
			.has_slc_cache_line_size_bits = true,
			.has_slc_size_in_kilobytes = true,
			.has_slc_vivt = true,
			.has_sys_bus_secure_reset = true,
			.has_tdm_control_stream_format = true,
			.has_tile_size_x = true,
			.has_tile_size_y = true,
			.has_vdm_object_level_lls = true,
			.has_virtual_address_space_bits = true,

			.cdm_control_stream_format = 5,
			.common_store_size_in_dwords = 2560U * 4U * 4U,
			.fbc_max_default_descriptors = 2048,
			.fbc_max_large_descriptors = 32,
			.fbcdc = 5,
			.fbcdc_algorithm = 5,
			.fbcdc_architecture = 6,
			.host_security_version = 4,
			.isp_max_tiles_in_flight = 24,
			.max_partitions = 16,
			.mmu_version = 4,
			.num_clusters = 6,
			.num_isp_ipp_pipes = 24,
			.num_osids = 8,
			.num_raster_pipes = 2,
			.num_spus = 2,
			.phys_bus_width = 40,
			.ray_tracing_arch = 3,
			.slc_banks = 6,
			.slc_cache_line_size_bits = 1024,
			.slc_size_in_kilobytes = 1536,
			.tdm_control_stream_format = 3,
			.tile_size_x = 32,
			.tile_size_y = 32,
			.virtual_address_space_bits = 40,
		},
		.quirks = {
			.has_brn65104 = true,
			.has_brn69700 = true,
			.has_brn74812 = true,
			.has_brn77899 = true,
		},
	},
};

/**
 * pvr_device_info_init_volcanic() - Populate built-in Volcanic BVNC features and BRNs
 * @pvr_dev: Target PowerVR device with &pvr_device.gpu_id already decoded.
 *
 * Returns:
 *  * 0 if &pvr_device.gpu_id matched a known Volcanic BVNC entry, or
 *  * -%ENODEV if no Volcanic entry exists for &pvr_device.gpu_id.
 */
int pvr_device_info_init_volcanic(struct pvr_device *pvr_dev)
{
	u64 packed_bvnc = pvr_gpu_id_to_packed_bvnc(&pvr_dev->gpu_id);

	for (size_t i = 0; i < ARRAY_SIZE(pvr_volcanic_bvnc_table); i++) {
		if (pvr_volcanic_bvnc_table[i].packed_bvnc == packed_bvnc) {
			pvr_dev->features = pvr_volcanic_bvnc_table[i].features;
			pvr_dev->quirks = pvr_volcanic_bvnc_table[i].quirks;
			return 0;
		}
	}

	return -ENODEV;
}
