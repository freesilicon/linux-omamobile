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

struct laguna_panel_desc {
	const struct drm_display_mode mode;
	const struct drm_dsc_config dsc;
	unsigned long mode_flags;
	u32 lanes;
	u32 hs_rate_mbps;
};

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

static int laguna_panel_dcs_window(struct mipi_dsi_device *dsi, u16 w, u16 h)
{
	int ret;

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, w - 1);
	if (ret < 0)
		return ret;

	return mipi_dsi_dcs_set_page_address(dsi, 0, h - 1);
}

static int laguna_panel_prepare(struct drm_panel *panel)
{
	struct laguna_panel *ctx = to_laguna_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	u16 w = ctx->desc->mode.hdisplay;
	u16 h = ctx->desc->mode.vdisplay;
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
	}
	if (ctx->vddd) {
		ret = regulator_enable(ctx->vddd);
		if (ret)
			goto err_vci;
	}

	if (ctx->reset_gpio) {
		if (panel_full_reset) {
			gpiod_set_value_cansleep(ctx->reset_gpio, 0);
			usleep_range(10000, 12000);
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
			usleep_range(10000, 12000);
		} else {
			/* Keep reset deasserted without pulsing to preserve ABL state */
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		}
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		dev_warn(&dsi->dev, "SLPOUT returned %d (continuing handoff)\n", ret);
	msleep(panel_full_reset ? 120 : 10);

	ret = laguna_panel_dcs_window(dsi, w, h);
	if (ret < 0)
		dev_warn(&dsi->dev, "window update returned %d\n", ret);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0)
		dev_warn(&dsi->dev, "compression mode enable returned %d\n", ret);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_warn(&dsi->dev, "PPS write returned %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		dev_warn(&dsi->dev, "set_tear_on returned %d\n", ret);

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
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
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
		.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
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
};

#define RGEB_HTOTAL	(1080 + 44 + 16 + 20)
#define RGEB_VTOTAL	(2364 + 10 + 6 + 10)

static const struct laguna_panel_desc google_rgeb_desc = {
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
		.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
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
};

#define RGEA_HTOTAL	(2152 + 80 + 30 + 38)
#define RGEA_VTOTAL	(2076 + 6 + 4 + 14)

static const struct laguna_panel_desc google_rgea_desc = {
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
		.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
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
};

static struct regulator *laguna_get_opt_regulator(struct device *dev, const char *id)
{
	struct regulator *reg = devm_regulator_get_optional(dev, id);

	if (IS_ERR(reg) && PTR_ERR(reg) == -ENODEV)
		return NULL;
	return reg;
}

const struct drm_dsc_config *laguna_panel_get_dsc_by_mode(u16 hdisplay, u16 vdisplay)
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
			return &descs[i]->dsc;
	}
	return NULL;
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
