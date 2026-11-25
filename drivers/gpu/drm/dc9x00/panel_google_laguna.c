// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room MIPI DSI command-mode panel driver for Tensor G5 Laguna displays
 * (Mustang google-mtea; Rango google-rgeb / google-rgea descriptors ready).
 *
 * Designed for ABL handoff without toggling panel reset-gpios by default,
 * leaving calibrated bootloader DDIC state intact.
 */

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>

#include "dc9x00_internal.h"

static bool panel_full_reset;
module_param(panel_full_reset, bool, 0644);
MODULE_PARM_DESC(panel_full_reset,
		 "Pulse panel reset GPIO during prepare instead of preserving ABL state (default: N)");

enum laguna_panel_type {
	LAGUNA_PANEL_MTEA,
	LAGUNA_PANEL_RGEB,
	LAGUNA_PANEL_RGEA,
};

struct laguna_panel_desc {
	enum laguna_panel_type type;
	const struct drm_display_mode mode;
	const struct drm_dsc_config dsc;
	unsigned long mode_flags;
	u32 lanes;
	u32 hs_rate_mbps;
	u32 te_fps;
	u32 te_usec;
	bool cold_init;
};

struct laguna_dcs_cmd {
	u8 len;
	u8 data[8];
};

#define LAGUNA_DCS_CMD(...) {					\
	.len = sizeof((const u8[]){ __VA_ARGS__ }),		\
	.data = { __VA_ARGS__ },				\
}

struct laguna_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct laguna_panel_desc *desc;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator *vci;
	struct regulator *vddi;
	struct regulator *vddd;
	bool prepared;
};

static inline struct laguna_panel *to_laguna_panel(struct drm_panel *panel)
{
	return container_of(panel, struct laguna_panel, panel);
}

static void laguna_panel_dcs_send(struct mipi_dsi_device *dsi,
				  const u8 *cmd, size_t len)
{
	ssize_t ret;

	ret = mipi_dsi_dcs_write_buffer(dsi, cmd, len);
	if (ret < 0)
		dev_warn(&dsi->dev, "DCS %#02x (%zu bytes) returned %zd\n",
			 cmd[0], len, ret);
}

static void laguna_panel_dcs_seq(struct mipi_dsi_device *dsi,
				 const struct laguna_dcs_cmd *cmds,
				 size_t count)
{
	size_t i;

	for (i = 0; i < count; i++)
		laguna_panel_dcs_send(dsi, cmds[i].data, cmds[i].len);
}

static int laguna_panel_dcs_window(struct mipi_dsi_device *dsi, u16 w, u16 h)
{
	int ret;

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, w - 1);
	if (ret < 0)
		return ret;

	return mipi_dsi_dcs_set_page_address(dsi, 0, h - 1);
}

static void laguna_panel_rgeb_prepare(struct laguna_panel *ctx)
{
	static const struct laguna_dcs_cmd crosstalk_cmds[] = {
		LAGUNA_DCS_CMD(0xf0, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xf1, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xd0, 0x08),
		LAGUNA_DCS_CMD(0x91, 0x02),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x48, 0x8f),
		LAGUNA_DCS_CMD(0x8f, 0x63),
		LAGUNA_DCS_CMD(0xf0, 0xa5, 0xa5),
		LAGUNA_DCS_CMD(0xf1, 0xa5, 0xa5),
	};
	static const struct laguna_dcs_cmd init_ffc_cmds[] = {
		LAGUNA_DCS_CMD(0xf0, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xfc, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x3e, 0xc5),
		LAGUNA_DCS_CMD(0xc5, 0x59, 0x02),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x36, 0xc5),
		LAGUNA_DCS_CMD(0xc5, 0x11, 0x10, 0x50, 0x05),
	};
	static const struct laguna_dcs_cmd evt1_vddd_cmds[] = {
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x58, 0xd7),
		LAGUNA_DCS_CMD(0xd7, 0x06),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x5b, 0xd7),
		LAGUNA_DCS_CMD(0xd7, 0x06),
		LAGUNA_DCS_CMD(0xfe, 0x80),
		LAGUNA_DCS_CMD(0xfe, 0x00),
	};
	static const struct laguna_dcs_cmd lock_cmds[] = {
		LAGUNA_DCS_CMD(0xfc, 0xa5, 0xa5),
		LAGUNA_DCS_CMD(0xf0, 0xa5, 0xa5),
	};
	static const struct laguna_dcs_cmd mode_60hz_cmds[] = {
		LAGUNA_DCS_CMD(0xf0, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0x83, 0x08),
		LAGUNA_DCS_CMD(0xf7, 0x2f),
		LAGUNA_DCS_CMD(0xf0, 0xa5, 0xa5),
	};
	static const u8 wrctrld_cmd[] = { MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20 };
	static const u8 dbv_cmd[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x04, 0xb9 };
	static const u8 normal_cmd[] = { MIPI_DCS_ENTER_NORMAL_MODE };
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	u16 w = ctx->desc->mode.hdisplay;
	u16 h = ctx->desc->mode.vdisplay;
	u8 rev_id = 0;
	int ret;

	laguna_panel_dcs_seq(dsi, crosstalk_cmds, ARRAY_SIZE(crosstalk_cmds));

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgeb SLPOUT returned %d\n", ret);
	msleep(120);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgeb set_tear_on returned %d\n", ret);

	ret = laguna_panel_dcs_window(dsi, w, h);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgeb window update returned %d\n", ret);

	laguna_panel_dcs_seq(dsi, init_ffc_cmds, ARRAY_SIZE(init_ffc_cmds));

	ret = mipi_dsi_dcs_read(dsi, 0xdb, &rev_id, sizeof(rev_id));
	if (ret != sizeof(rev_id) || (rev_id >> 3) >= 8)
		laguna_panel_dcs_seq(dsi, evt1_vddd_cmds,
				     ARRAY_SIZE(evt1_vddd_cmds));

	laguna_panel_dcs_seq(dsi, lock_cmds, ARRAY_SIZE(lock_cmds));
	laguna_panel_dcs_seq(dsi, mode_60hz_cmds, ARRAY_SIZE(mode_60hz_cmds));

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgeb compression enable returned %d\n", ret);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgeb PPS write returned %d\n", ret);

	laguna_panel_dcs_send(dsi, wrctrld_cmd, sizeof(wrctrld_cmd));
	laguna_panel_dcs_send(dsi, dbv_cmd, sizeof(dbv_cmd));
	laguna_panel_dcs_send(dsi, normal_cmd, sizeof(normal_cmd));
}

static void laguna_panel_handoff_prepare(struct laguna_panel *ctx)
{
	static const struct laguna_dcs_cmd mtea_unlock_cmds[] = {
		LAGUNA_DCS_CMD(0x9d, 0x01),
		LAGUNA_DCS_CMD(0xf0, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xc3, 0x02),
	};
	static const struct laguna_dcs_cmd mtea_latch_cmds[] = {
		LAGUNA_DCS_CMD(0xf7, 0x2f),
		LAGUNA_DCS_CMD(0xf0, 0xa5, 0xa5),
	};
	static const u8 wrctrld_cmd[] = { MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20 };
	static const u8 mtea_dbv_cmd[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x12, 0x1d };
	static const u8 rgea_dbv_cmd[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x04, 0xb9 };
	static const u8 normal_cmd[] = { MIPI_DCS_ENTER_NORMAL_MODE };
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	u16 w = ctx->desc->mode.hdisplay;
	u16 h = ctx->desc->mode.vdisplay;
	u8 pwr_mode = 0;
	u8 brt[2] = {};
	bool waking;
	int ret;

	ret = mipi_dsi_dcs_get_power_mode(dsi, &pwr_mode);
	waking = panel_full_reset || ret < 0 ||
		 !(pwr_mode & MIPI_DSI_DCS_POWER_MODE_SLEEP);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		dev_warn(&dsi->dev, "SLPOUT returned %d (continuing handoff)\n", ret);
	msleep(waking ? 120 : 10);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0)
		dev_warn(&dsi->dev, "compression mode enable returned %d\n", ret);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_warn(&dsi->dev, "PPS write returned %d\n", ret);

	if (ctx->desc->type == LAGUNA_PANEL_MTEA) {
		laguna_panel_dcs_seq(dsi, mtea_unlock_cmds,
				     ARRAY_SIZE(mtea_unlock_cmds));
		ret = laguna_panel_dcs_window(dsi, w, h);
		if (ret < 0)
			dev_warn(&dsi->dev, "window update returned %d\n", ret);
		laguna_panel_dcs_seq(dsi, mtea_latch_cmds,
				     ARRAY_SIZE(mtea_latch_cmds));
	} else {
		ret = laguna_panel_dcs_window(dsi, w, h);
		if (ret < 0)
			dev_warn(&dsi->dev, "window update returned %d\n", ret);
	}

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		dev_warn(&dsi->dev, "set_tear_on returned %d\n", ret);

	laguna_panel_dcs_send(dsi, wrctrld_cmd, sizeof(wrctrld_cmd));

	ret = mipi_dsi_dcs_read(dsi, MIPI_DCS_GET_DISPLAY_BRIGHTNESS,
				brt, sizeof(brt));
	if (panel_full_reset || ret < 0 || (!brt[0] && !brt[1])) {
		if (ctx->desc->type == LAGUNA_PANEL_MTEA)
			laguna_panel_dcs_send(dsi, mtea_dbv_cmd,
					      sizeof(mtea_dbv_cmd));
		else
			laguna_panel_dcs_send(dsi, rgea_dbv_cmd,
					      sizeof(rgea_dbv_cmd));
	}

	laguna_panel_dcs_send(dsi, normal_cmd, sizeof(normal_cmd));
}

static void laguna_panel_rgea_prepare(struct laguna_panel *ctx)
{
	static const struct laguna_dcs_cmd init_cmds[] = {
		LAGUNA_DCS_CMD(0xf0, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xb9, 0x44, 0x51),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x04, 0xb9),
		LAGUNA_DCS_CMD(0xb9, 0x81, 0xe0, 0x02, 0x81, 0xe0, 0x02),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x0d, 0xb9),
		LAGUNA_DCS_CMD(0xb9, 0x02, 0x02, 0x07, 0x02, 0x02, 0x07),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x06, 0xbd),
		LAGUNA_DCS_CMD(0xbd, 0x80),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x04, 0xbb),
		LAGUNA_DCS_CMD(0xbb, 0x0c),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x03, 0xbb),
		LAGUNA_DCS_CMD(0xbb, 0x00, 0x0c),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x48, 0xf4),
		LAGUNA_DCS_CMD(0xf4, 0x73, 0x73, 0x73, 0x73),
		LAGUNA_DCS_CMD(0xfc, 0x5a, 0x5a),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0xa1, 0x62),
		LAGUNA_DCS_CMD(0x62, 0x01),
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x02, 0xc4),
		LAGUNA_DCS_CMD(0xc4, 0x00),
		LAGUNA_DCS_CMD(0xfc, 0xa5, 0xa5),
	};
	static const struct laguna_dcs_cmd dvt1_timing_cmds[] = {
		LAGUNA_DCS_CMD(0xb0, 0x00, 0x88, 0xad),
		LAGUNA_DCS_CMD(0xad, 0x23, 0x00, 0x04),
	};
	static const struct laguna_dcs_cmd mode_60hz_lock_cmds[] = {
		LAGUNA_DCS_CMD(0xbd, 0x01),
		LAGUNA_DCS_CMD(0x83, 0x02),
		LAGUNA_DCS_CMD(0xf7, 0x2f),
		LAGUNA_DCS_CMD(0xf0, 0xa5, 0xa5),
	};
	static const u8 dsc_on_cmd[] = { 0x9d, 0x01 };
	static const u8 wrctrld_cmd[] = { MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20 };
	static const u8 dbv_cmd[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x04, 0x4f };
	static const u8 normal_cmd[] = { MIPI_DCS_ENTER_NORMAL_MODE };
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	u16 w = ctx->desc->mode.hdisplay;
	u16 h = ctx->desc->mode.vdisplay;
	u8 rev_id = 0;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgea SLPOUT returned %d\n", ret);
	msleep(120);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgea set_tear_on returned %d\n", ret);

	laguna_panel_dcs_seq(dsi, init_cmds, ARRAY_SIZE(init_cmds));

	ret = mipi_dsi_dcs_read(dsi, 0xdb, &rev_id, sizeof(rev_id));
	if (ret != sizeof(rev_id) || (rev_id >> 3) >= 0x10)
		laguna_panel_dcs_seq(dsi, dvt1_timing_cmds,
				     ARRAY_SIZE(dvt1_timing_cmds));

	laguna_panel_dcs_seq(dsi, mode_60hz_lock_cmds,
			     ARRAY_SIZE(mode_60hz_lock_cmds));

	ret = laguna_panel_dcs_window(dsi, w, h);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgea window update returned %d\n", ret);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgea compression enable returned %d\n", ret);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_warn(&dsi->dev, "rgea PPS write returned %d\n", ret);

	laguna_panel_dcs_send(dsi, dsc_on_cmd, sizeof(dsc_on_cmd));
	laguna_panel_dcs_send(dsi, wrctrld_cmd, sizeof(wrctrld_cmd));
	laguna_panel_dcs_send(dsi, dbv_cmd, sizeof(dbv_cmd));
	laguna_panel_dcs_send(dsi, normal_cmd, sizeof(normal_cmd));
}

static int laguna_panel_prepare(struct drm_panel *panel)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	bool do_reset = ctx->desc->cold_init || panel_full_reset;
	int ret;

	if (ctx->prepared)
		return 0;

	if (ctx->vddi) {
		ret = regulator_enable(ctx->vddi);
		if (ret)
			return ret;
	}
	if (ctx->vci) {
		ret = regulator_enable(ctx->vci);
		if (ret)
			goto err_vddi;
		if (do_reset)
			usleep_range(10000, 11000);
	}
	if (ctx->vddd) {
		ret = regulator_enable(ctx->vddd);
		if (ret)
			goto err_vci;
		if (do_reset)
			usleep_range(10000, 11000);
	}

	if (ctx->reset_gpio) {
		if (do_reset) {
			gpiod_direction_output(ctx->reset_gpio, 0);
			usleep_range(1000, 1100);
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
			usleep_range(10000, 11000);
		} else {
			/* Keep reset deasserted without pulsing to preserve ABL state */
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		}
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (do_reset && ctx->desc->type == LAGUNA_PANEL_RGEB)
		laguna_panel_rgeb_prepare(ctx);
	else if (do_reset && ctx->desc->type == LAGUNA_PANEL_RGEA)
		laguna_panel_rgea_prepare(ctx);
	else
		laguna_panel_handoff_prepare(ctx);

	ctx->prepared = true;
	return 0;

err_vci:
	if (ctx->vci)
		regulator_disable(ctx->vci);
err_vddi:
	if (ctx->vddi)
		regulator_disable(ctx->vddi);
	return ret;
}

static int laguna_panel_enable(struct drm_panel *panel)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);
	static const u8 normal_cmd[] = { MIPI_DCS_ENTER_NORMAL_MODE };
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	laguna_panel_dcs_send(ctx->dsi, normal_cmd, sizeof(normal_cmd));
	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0)
		dev_warn(&ctx->dsi->dev, "DISPON returned %d\n", ret);

	return 0;
}

static int laguna_panel_disable(struct drm_panel *panel)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_warn(&ctx->dsi->dev, "DISPOFF returned %d\n", ret);

	return 0;
}

static int laguna_panel_unprepare(struct drm_panel *panel)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);

	if (!ctx->prepared)
		return 0;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	msleep(20);

	if (ctx->vddd)
		regulator_disable(ctx->vddd);
	if (ctx->vci)
		regulator_disable(ctx->vci);
	if (ctx->vddi)
		regulator_disable(ctx->vddi);

	ctx->prepared = false;
	return 0;
}

static int laguna_panel_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ctx->desc->mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = ctx->desc->mode.width_mm;
	connector->display_info.height_mm = ctx->desc->mode.height_mm;
	connector->display_info.bpc = 8;

	return 1;
}

static const struct drm_panel_funcs laguna_panel_funcs = {
	.prepare = laguna_panel_prepare,
	.enable = laguna_panel_enable,
	.disable = laguna_panel_disable,
	.unprepare = laguna_panel_unprepare,
	.get_modes = laguna_panel_get_modes,
};

#define MTEA_HTOTAL	(1344 + 88 + 24 + 44)
#define MTEA_VTOTAL	(2992 + 12 + 4 + 22)

static const struct laguna_panel_desc google_mtea_desc = {
	.type = LAGUNA_PANEL_MTEA,
	.mode = {
		.clock = MTEA_HTOTAL * MTEA_VTOTAL * 60 / 1000,
		.hdisplay = 1344,
		.hsync_start = 1344 + 88,
		.hsync_end = 1344 + 88 + 24,
		.htotal = MTEA_HTOTAL,
		.vdisplay = 2992,
		.vsync_start = 2992 + 12,
		.vsync_end = 2992 + 12 + 4,
		.vtotal = MTEA_VTOTAL,
		.width_mm = 70,
		.height_mm = 156,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	},
	.dsc = {
		.line_buf_depth = 9,
		.bits_per_component = 8,
		.convert_rgb = true,
		.slice_count = 2,
		.slice_width = 672,
		.slice_height = 34,
		.simple_422 = false,
		.pic_width = 1344,
		.pic_height = 2992,
		.rc_tgt_offset_high = 3,
		.rc_tgt_offset_low = 3,
		.bits_per_pixel = 128,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit1 = 11,
		.rc_quant_incr_limit0 = 11,
		.initial_xmit_delay = 512,
		.initial_dec_delay = 592,
		.block_pred_enable = true,
		.first_line_bpg_offset = 12,
		.initial_offset = 6144,
		.rc_buf_thresh = {
			14, 28, 42, 56, 70, 84, 98, 105,
			112, 119, 121, 123, 125, 126
		},
		.rc_range_params = {
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 2 },
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 5, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 6, .range_bpg_offset = 62 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 60 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 58 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 8, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 9, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 10, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 11, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 12, .range_bpg_offset = 52 },
			{ .range_min_qp = 5, .range_max_qp = 13, .range_bpg_offset = 52 },
			{ .range_min_qp = 7, .range_max_qp = 13, .range_bpg_offset = 52 },
			{ .range_min_qp = 13, .range_max_qp = 15, .range_bpg_offset = 52 },
		},
		.rc_model_size = 8192,
		.flatness_min_qp = 3,
		.flatness_max_qp = 12,
		.initial_scale_value = 32,
		.scale_decrement_interval = 9,
		.scale_increment_interval = 932,
		.nfl_bpg_offset = 745,
		.slice_bpg_offset = 616,
		.final_offset = 4336,
		.slice_chunk_size = 672,
		.dsc_version_major = 1,
		.dsc_version_minor = 2,
		.mux_word_size = 48,
	},
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.lanes = 4,
	.hs_rate_mbps = 1368,
	.te_fps = 120,
	.te_usec = 275,
	.cold_init = false,
};

#define RGEB_HTOTAL	(1080 + 44 + 16 + 20)
#define RGEB_VTOTAL	(2364 + 10 + 6 + 10)

static const struct laguna_panel_desc google_rgeb_desc = {
	.type = LAGUNA_PANEL_RGEB,
	.mode = {
		.clock = RGEB_HTOTAL * RGEB_VTOTAL * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 44,
		.hsync_end = 1080 + 44 + 16,
		.htotal = RGEB_HTOTAL,
		.vdisplay = 2364,
		.vsync_start = 2364 + 10,
		.vsync_end = 2364 + 10 + 6,
		.vtotal = RGEB_VTOTAL,
		.width_mm = 67,
		.height_mm = 147,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	},
	.dsc = {
		.line_buf_depth = 9,
		.bits_per_component = 8,
		.convert_rgb = true,
		.slice_count = 2,
		.slice_width = 540,
		.slice_height = 197,
		.pic_width = 1080,
		.pic_height = 2364,
		.rc_tgt_offset_high = 3,
		.rc_tgt_offset_low = 3,
		.bits_per_pixel = 128,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit1 = 11,
		.rc_quant_incr_limit0 = 11,
		.initial_xmit_delay = 512,
		.initial_dec_delay = 526,
		.block_pred_enable = true,
		.first_line_bpg_offset = 12,
		.initial_offset = 6144,
		.rc_buf_thresh = {
			14, 28, 42, 56, 70, 84, 98, 105,
			112, 119, 121, 123, 125, 126
		},
		.rc_range_params = {
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 2 },
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 5, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 6, .range_bpg_offset = 62 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 60 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 58 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 8, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 9, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 10, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 11, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 12, .range_bpg_offset = 52 },
			{ .range_min_qp = 5, .range_max_qp = 13, .range_bpg_offset = 52 },
			{ .range_min_qp = 7, .range_max_qp = 13, .range_bpg_offset = 52 },
			{ .range_min_qp = 13, .range_max_qp = 15, .range_bpg_offset = 52 },
		},
		.rc_model_size = 8192,
		.flatness_min_qp = 3,
		.flatness_max_qp = 12,
		.initial_scale_value = 32,
		.scale_decrement_interval = 7,
		.scale_increment_interval = 4898,
		.nfl_bpg_offset = 126,
		.slice_bpg_offset = 133,
		.final_offset = 4336,
		.slice_chunk_size = 540,
		.dsc_version_major = 1,
		.dsc_version_minor = 2,
		.mux_word_size = 48,
	},
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.lanes = 4,
	.hs_rate_mbps = 865,
	.te_fps = 60,
	.te_usec = 8623,
	.cold_init = true,
};

#define RGEA_HTOTAL	(2152 + 80 + 30 + 38)
#define RGEA_VTOTAL	(2076 + 6 + 4 + 14)

static const struct laguna_panel_desc google_rgea_desc = {
	.type = LAGUNA_PANEL_RGEA,
	.mode = {
		.clock = RGEA_HTOTAL * RGEA_VTOTAL * 60 / 1000,
		.hdisplay = 2152,
		.hsync_start = 2152 + 80,
		.hsync_end = 2152 + 80 + 30,
		.htotal = RGEA_HTOTAL,
		.vdisplay = 2076,
		.vsync_start = 2076 + 6,
		.vsync_end = 2076 + 6 + 4,
		.vtotal = RGEA_VTOTAL,
		.width_mm = 147,
		.height_mm = 141,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	},
	.dsc = {
		.line_buf_depth = 9,
		.bits_per_component = 8,
		.convert_rgb = true,
		.slice_count = 2,
		.slice_width = 1076,
		.slice_height = 173,
		.pic_width = 2152,
		.pic_height = 2076,
		.rc_tgt_offset_high = 3,
		.rc_tgt_offset_low = 3,
		.bits_per_pixel = 128,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit1 = 11,
		.rc_quant_incr_limit0 = 11,
		.initial_xmit_delay = 512,
		.initial_dec_delay = 930,
		.block_pred_enable = true,
		.first_line_bpg_offset = 15,
		.initial_offset = 6144,
		.rc_buf_thresh = {
			14, 28, 42, 56, 70, 84, 98, 105,
			112, 119, 121, 123, 125, 126
		},
		.rc_range_params = {
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 2 },
			{ .range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 5, .range_bpg_offset = 0 },
			{ .range_min_qp = 1, .range_max_qp = 6, .range_bpg_offset = 62 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 60 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 58 },
			{ .range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 8, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 9, .range_bpg_offset = 56 },
			{ .range_min_qp = 3, .range_max_qp = 10, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 10, .range_bpg_offset = 54 },
			{ .range_min_qp = 5, .range_max_qp = 11, .range_bpg_offset = 52 },
			{ .range_min_qp = 5, .range_max_qp = 11, .range_bpg_offset = 52 },
			{ .range_min_qp = 9, .range_max_qp = 12, .range_bpg_offset = 52 },
			{ .range_min_qp = 12, .range_max_qp = 13, .range_bpg_offset = 52 },
		},
		.rc_model_size = 8192,
		.flatness_min_qp = 3,
		.flatness_max_qp = 12,
		.initial_scale_value = 32,
		.scale_decrement_interval = 14,
		.scale_increment_interval = 4976,
		.nfl_bpg_offset = 179,
		.slice_bpg_offset = 75,
		.final_offset = 4320,
		.slice_chunk_size = 1076,
		.dsc_version_major = 1,
		.dsc_version_minor = 2,
		.mux_word_size = 48,
	},
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.lanes = 4,
	.hs_rate_mbps = 1468,
	.te_fps = 60,
	.te_usec = 8635,
	.cold_init = true,
};

static struct regulator *laguna_get_opt_regulator(struct device *dev, const char *id)
{
	struct regulator *reg = devm_regulator_get_optional(dev, id);

	if (IS_ERR(reg) && PTR_ERR(reg) == -ENODEV)
		return NULL;
	return reg;
}

static const struct laguna_panel_desc *
laguna_panel_find_desc(u16 hdisplay, u16 vdisplay)
{
	static const struct laguna_panel_desc * const descs[] = {
		&google_mtea_desc,
		&google_rgeb_desc,
		&google_rgea_desc,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(descs); i++) {
		if (descs[i]->mode.hdisplay == hdisplay &&
		    descs[i]->mode.vdisplay == vdisplay)
			return descs[i];
	}
	return NULL;
}

const struct drm_dsc_config *laguna_panel_get_dsc_by_mode(u16 hdisplay, u16 vdisplay)
{
	const struct laguna_panel_desc *desc = laguna_panel_find_desc(hdisplay, vdisplay);

	return desc ? &desc->dsc : NULL;
}

void laguna_panel_get_te_info(u16 hdisplay, u16 vdisplay, u32 *te_fps, u32 *te_usec)
{
	const struct laguna_panel_desc *desc = laguna_panel_find_desc(hdisplay, vdisplay);

	*te_fps = desc ? desc->te_fps : 60;
	*te_usec = desc ? desc->te_usec : 373;
}

static int laguna_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct laguna_panel_desc *desc;
	struct laguna_panel *ctx;
	int ret;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -ENODEV;

	ctx = devm_drm_panel_alloc(dev, struct laguna_panel, panel,
				   &laguna_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	ctx->desc = desc;
	ctx->dsc = desc->dsc;
	ctx->panel.prepare_prev_first = true;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	ctx->vddi = laguna_get_opt_regulator(dev, "vddi");
	if (IS_ERR(ctx->vddi))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi), "failed to get vddi\n");

	ctx->vci = laguna_get_opt_regulator(dev, "vci");
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci), "failed to get vci\n");

	ctx->vddd = laguna_get_opt_regulator(dev, "vddd");
	if (IS_ERR(ctx->vddd))
		return dev_err_probe(dev, PTR_ERR(ctx->vddd), "failed to get vddd\n");

	dsi->lanes = desc->lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = desc->mode_flags;
	dsi->dsc = &ctx->dsc;

	mipi_dsi_set_drvdata(dsi, ctx);
	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void laguna_panel_remove(struct mipi_dsi_device *dsi)
{
	struct laguna_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id laguna_panel_of_match[] = {
	{ .compatible = "google,gs-mtea", .data = &google_mtea_desc },
	{ .compatible = "google,gs-rgeb", .data = &google_rgeb_desc },
	{ .compatible = "google,gs-rgea", .data = &google_rgea_desc },
	{}
};
MODULE_DEVICE_TABLE(of, laguna_panel_of_match);

struct mipi_dsi_driver laguna_panel_driver = {
	.probe = laguna_panel_probe,
	.remove = laguna_panel_remove,
	.driver = {
		.name = "panel-google-laguna",
		.of_match_table = laguna_panel_of_match,
	},
};
