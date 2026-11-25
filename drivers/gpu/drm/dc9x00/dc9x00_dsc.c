// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room DSC 1.2a encoder programming for VeriSilicon DC9x00.
 */

#include <drm/display/drm_dsc.h>
#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/minmax.h>

#include "dc9x00_internal.h"
#include "dc9x00_regs.h"

#define DC9X00_DSC_HS0_DWORD		0x040
#define DC9X00_DSC_ENC_MAIN_CONF	0x000
#define DC9X00_DSC_HS_DF_CTRL		0x000
#define DC9X00_DSC_HS_MAIN_CONF		0x00C
#define DC9X00_DSC_HS_PICT_SIZE		0x00D
#define DC9X00_DSC_HS_SLICE_SIZE	0x00E
#define DC9X00_DSC_HS_MISC_SIZE		0x00F
#define DC9X00_DSC_HS_HRD_DELAY		0x010
#define DC9X00_DSC_HS_RC_SCALE		0x011
#define DC9X00_DSC_HS_RC_SCALE_INC_DEC	0x012
#define DC9X00_DSC_HS_RC_OFFSETS_1	0x013
#define DC9X00_DSC_HS_RC_OFFSETS_2	0x014
#define DC9X00_DSC_HS_RC_OFFSETS_3	0x015
#define DC9X00_DSC_HS_RC_OFFSETS_4	0x016
#define DC9X00_DSC_HS_FLATNESS_QP	0x017
#define DC9X00_DSC_HS_RC_MODEL_SIZE	0x018
#define DC9X00_DSC_HS_RC_CONFIG		0x019
#define DC9X00_DSC_HS_RC_BUF_THRESH_0	0x01A
#define DC9X00_DSC_HS_RC_BUF_THRESH_1	0x01B
#define DC9X00_DSC_HS_RC_BUF_THRESH_2	0x01C
#define DC9X00_DSC_HS_RC_BUF_THRESH_3	0x01D
#define DC9X00_DSC_HS_RC_MIN_QP_0	0x01E
#define DC9X00_DSC_HS_RC_MIN_QP_1	0x01F
#define DC9X00_DSC_HS_RC_MIN_QP_2	0x020
#define DC9X00_DSC_HS_RC_MAX_QP_0	0x021
#define DC9X00_DSC_HS_RC_MAX_QP_1	0x022
#define DC9X00_DSC_HS_RC_MAX_QP_2	0x023
#define DC9X00_DSC_HS_RC_BPG_OFF_0	0x024
#define DC9X00_DSC_HS_RC_BPG_OFF_1	0x025
#define DC9X00_DSC_HS_RC_BPG_OFF_2	0x026

#define DC9X00_DSC_NB_HS_ENC		2
#define DC9X00_DSC_MAX_HS_PIXELS	4096

static inline void dc9x00_dsc_write(void __iomem *base, u8 panel_id, u32 dword_reg, u32 val)
{
	u32 block_base;

	switch (panel_id) {
	case 0:
		block_base = DC9X00_PANEL0_DSC_BASE;
		break;
	case 1:
		block_base = DC9X00_PANEL1_DSC_BASE;
		break;
	default:
		block_base = DC9X00_PANEL2_DSC_BASE;
		break;
	}

	writel(val, base + block_base + dword_reg * 4);
}

static u32 dc9x00_dsc_calc_drb_max(const struct dc9x00_dsc_usage *sw,
				   const struct drm_dsc_config *dsc)
{
	u32 hs_num = max_t(u32, 1, sw->slices_per_line / max_t(u8, 1, sw->ss_num));
	u32 container_hs_w = min_t(u32, (u32)dsc->slice_width * sw->ss_num,
				   DC9X00_DSC_MAX_HS_PIXELS);

	return round_down(container_hs_w * (hs_num - 1) / (hs_num * DC9X00_DSC_NB_HS_ENC), 2) + 3;
}

static u32 dc9x00_dsc_calc_initial_lines(const struct dc9x00_dsc_usage *sw,
					 const struct drm_dsc_config *dsc)
{
	const u32 pipeline_latency = 28;
	const u32 ssm_delay = 91;
	const u32 ob_data_width = 128;
	const u32 output_rate = 8;
	u32 hs_num = max_t(u32, 1, sw->slices_per_line / max_t(u8, 1, sw->ss_num));
	u32 bpp = max_t(u32, 1, dsc->bits_per_pixel >> 4);
	u32 mux_word = dsc->mux_word_size ? dsc->mux_word_size : 48;
	u32 input_ssm_lat = pipeline_latency + 3 * (ssm_delay + 2) * sw->ss_num;
	u32 obuf_lat = DIV_ROUND_UP(9 * ob_data_width + mux_word, bpp) + 1;
	u32 base_lat = dsc->initial_xmit_delay + input_ssm_lat + obuf_lat;
	u32 chunk_bits = (u32)dsc->slice_chunk_size * 8;
	u32 extra_bits = 0;
	u32 extra_lat;

	if (sw->split_panel_enable && sw->de_raster_enable) {
		if (!sw->multiplex_mode_enable && sw->ss_num != 1 && hs_num > 1)
			extra_bits = chunk_bits;
		else
			extra_bits = DIV_ROUND_UP((hs_num - 1) * chunk_bits, hs_num);
	} else if (sw->split_panel_enable && sw->multiplex_mode_enable) {
		extra_bits = chunk_bits;
	} else if (sw->ss_num != 1 && output_rate > bpp) {
		extra_bits = chunk_bits;
	}

	extra_lat = DIV_ROUND_UP(extra_bits, bpp) + (sw->de_raster_enable ? 5 : 0);

	return DIV_ROUND_UP(base_lat + extra_lat, max_t(u32, 1, dsc->slice_width));
}

int dc9x00_dsc_configure(void __iomem *base, u8 panel_id,
			 const struct dc9x00_dsc_usage *sw,
			 const struct drm_dsc_config *dsc)
{
	u32 split_cfg, main_conf, df_ctrl, dsc_main, flatness_thresh;
	u32 ob_max_addr, drb_max_addr, initial_lines;
	const u32 hs = DC9X00_DSC_HS0_DWORD;

	if (panel_id != 0)
		return -ENODEV;

	split_cfg = FIELD_PREP(DC9X00_PANEL0_SPLIT_SRC_MASK,
			       sw->enable ? DC9X00_PANEL0_SPLIT_SRC_DSC :
					    DC9X00_PANEL0_SPLIT_SRC_PIPE);
	writel(split_cfg, base + DC9X00_SH_PANEL0_SPLIT_CONFIG);

	if (!sw->enable || !dsc)
		return 0;

	ob_max_addr = (sw->ss_num == 1) ? 919 : 459;
	drb_max_addr = dc9x00_dsc_calc_drb_max(sw, dsc);
	initial_lines = dc9x00_dsc_calc_initial_lines(sw, dsc);

	main_conf = (sw->split_panel_enable ? BIT(0) : 0) |
		    (sw->multiplex_mode_enable ? BIT(1) : 0) |
		    ((u32)sw->multiplex_out_sel << 2) |
		    (sw->multiplex_eoc_enable ? BIT(5) : 0) |
		    (sw->de_raster_enable ? BIT(6) : 0) |
		    ((u32)sw->ss_num << 7) |
		    (drb_max_addr << 18);
	dc9x00_dsc_write(base, panel_id, DC9X00_DSC_ENC_MAIN_CONF, main_conf);

	df_ctrl = (initial_lines & 0x1ff) |
		  (sw->video_mode ? BIT(9) : 0) |
		  (ob_max_addr << 18);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_DF_CTRL, df_ctrl);

	dsc_main = ((u32)dsc->bits_per_component) |
		   ((u32)dsc->convert_rgb << 4) |
		   ((u32)dsc->simple_422 << 5) |
		   ((u32)dsc->line_buf_depth << 6) |
		   ((u32)dsc->bits_per_pixel << 10) |
		   ((u32)dsc->block_pred_enable << 20) |
		   ((u32)dsc->native_420 << 21) |
		   ((u32)dsc->native_422 << 22) |
		   ((u32)dsc->dsc_version_minor << 28);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_MAIN_CONF, dsc_main);

	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_PICT_SIZE,
			 (u32)dsc->pic_width | ((u32)dsc->pic_height << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_SLICE_SIZE,
			 (u32)dsc->slice_width | ((u32)dsc->slice_height << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_MISC_SIZE,
			 dsc->slice_chunk_size);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_HRD_DELAY,
			 (u32)dsc->initial_xmit_delay | ((u32)dsc->initial_dec_delay << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_SCALE,
			 dsc->initial_scale_value);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_SCALE_INC_DEC,
			 (u32)dsc->scale_increment_interval |
			 ((u32)dsc->scale_decrement_interval << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_OFFSETS_1,
			 (u32)dsc->first_line_bpg_offset |
			 ((u32)dsc->second_line_bpg_offset << 5));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_OFFSETS_2,
			 (u32)dsc->nfl_bpg_offset | ((u32)dsc->slice_bpg_offset << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_OFFSETS_3,
			 (u32)dsc->initial_offset | ((u32)dsc->final_offset << 16));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_OFFSETS_4,
			 (u32)dsc->nsl_bpg_offset | ((u32)dsc->second_line_offset_adj << 16));

	flatness_thresh = 2U << max_t(int, 0, (int)dsc->bits_per_component - 8);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_FLATNESS_QP,
			 (u32)dsc->flatness_min_qp |
			 ((u32)dsc->flatness_max_qp << 5) |
			 (flatness_thresh << 10));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MODEL_SIZE,
			 dsc->rc_model_size);
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_CONFIG,
			 (u32)dsc->rc_edge_factor |
			 ((u32)dsc->rc_quant_incr_limit0 << 8) |
			 ((u32)dsc->rc_quant_incr_limit1 << 13) |
			 ((u32)dsc->rc_tgt_offset_high << 20) |
			 ((u32)dsc->rc_tgt_offset_low << 24));

	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BUF_THRESH_0,
			 (u32)dsc->rc_buf_thresh[0] | ((u32)dsc->rc_buf_thresh[1] << 8) |
			 ((u32)dsc->rc_buf_thresh[2] << 16) | ((u32)dsc->rc_buf_thresh[3] << 24));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BUF_THRESH_1,
			 (u32)dsc->rc_buf_thresh[4] | ((u32)dsc->rc_buf_thresh[5] << 8) |
			 ((u32)dsc->rc_buf_thresh[6] << 16) | ((u32)dsc->rc_buf_thresh[7] << 24));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BUF_THRESH_2,
			 (u32)dsc->rc_buf_thresh[8] | ((u32)dsc->rc_buf_thresh[9] << 8) |
			 ((u32)dsc->rc_buf_thresh[10] << 16) | ((u32)dsc->rc_buf_thresh[11] << 24));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BUF_THRESH_3,
			 (u32)dsc->rc_buf_thresh[12] | ((u32)dsc->rc_buf_thresh[13] << 8));

	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MIN_QP_0,
			 (u32)dsc->rc_range_params[0].range_min_qp |
			 ((u32)dsc->rc_range_params[1].range_min_qp << 5) |
			 ((u32)dsc->rc_range_params[2].range_min_qp << 10) |
			 ((u32)dsc->rc_range_params[3].range_min_qp << 15) |
			 ((u32)dsc->rc_range_params[4].range_min_qp << 20));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MIN_QP_1,
			 (u32)dsc->rc_range_params[5].range_min_qp |
			 ((u32)dsc->rc_range_params[6].range_min_qp << 5) |
			 ((u32)dsc->rc_range_params[7].range_min_qp << 10) |
			 ((u32)dsc->rc_range_params[8].range_min_qp << 15) |
			 ((u32)dsc->rc_range_params[9].range_min_qp << 20));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MIN_QP_2,
			 (u32)dsc->rc_range_params[10].range_min_qp |
			 ((u32)dsc->rc_range_params[11].range_min_qp << 5) |
			 ((u32)dsc->rc_range_params[12].range_min_qp << 10) |
			 ((u32)dsc->rc_range_params[13].range_min_qp << 15) |
			 ((u32)dsc->rc_range_params[14].range_min_qp << 20));

	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MAX_QP_0,
			 (u32)dsc->rc_range_params[0].range_max_qp |
			 ((u32)dsc->rc_range_params[1].range_max_qp << 5) |
			 ((u32)dsc->rc_range_params[2].range_max_qp << 10) |
			 ((u32)dsc->rc_range_params[3].range_max_qp << 15) |
			 ((u32)dsc->rc_range_params[4].range_max_qp << 20));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MAX_QP_1,
			 (u32)dsc->rc_range_params[5].range_max_qp |
			 ((u32)dsc->rc_range_params[6].range_max_qp << 5) |
			 ((u32)dsc->rc_range_params[7].range_max_qp << 10) |
			 ((u32)dsc->rc_range_params[8].range_max_qp << 15) |
			 ((u32)dsc->rc_range_params[9].range_max_qp << 20));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_MAX_QP_2,
			 (u32)dsc->rc_range_params[10].range_max_qp |
			 ((u32)dsc->rc_range_params[11].range_max_qp << 5) |
			 ((u32)dsc->rc_range_params[12].range_max_qp << 10) |
			 ((u32)dsc->rc_range_params[13].range_max_qp << 15) |
			 ((u32)dsc->rc_range_params[14].range_max_qp << 20));

	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BPG_OFF_0,
			 ((u32)dsc->rc_range_params[0].range_bpg_offset & 0x3f) |
			 (((u32)dsc->rc_range_params[1].range_bpg_offset & 0x3f) << 6) |
			 (((u32)dsc->rc_range_params[2].range_bpg_offset & 0x3f) << 12) |
			 (((u32)dsc->rc_range_params[3].range_bpg_offset & 0x3f) << 18) |
			 (((u32)dsc->rc_range_params[4].range_bpg_offset & 0x3f) << 24));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BPG_OFF_1,
			 ((u32)dsc->rc_range_params[5].range_bpg_offset & 0x3f) |
			 (((u32)dsc->rc_range_params[6].range_bpg_offset & 0x3f) << 6) |
			 (((u32)dsc->rc_range_params[7].range_bpg_offset & 0x3f) << 12) |
			 (((u32)dsc->rc_range_params[8].range_bpg_offset & 0x3f) << 18) |
			 (((u32)dsc->rc_range_params[9].range_bpg_offset & 0x3f) << 24));
	dc9x00_dsc_write(base, panel_id, hs | DC9X00_DSC_HS_RC_BPG_OFF_2,
			 ((u32)dsc->rc_range_params[10].range_bpg_offset & 0x3f) |
			 (((u32)dsc->rc_range_params[11].range_bpg_offset & 0x3f) << 6) |
			 (((u32)dsc->rc_range_params[12].range_bpg_offset & 0x3f) << 12) |
			 (((u32)dsc->rc_range_params[13].range_bpg_offset & 0x3f) << 18) |
			 (((u32)dsc->rc_range_params[14].range_bpg_offset & 0x3f) << 24));

	return 0;
}
