/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DC9X00_INTERNAL_H_
#define _DC9X00_INTERNAL_H_

#include <linux/types.h>

struct dentry;
struct drm_dsc_config;
struct mipi_dsi_driver;
struct platform_driver;

struct dc9x00_dsc_usage {
	bool enable;
	bool video_mode;
	u8 slices_per_line;
	u8 ss_num;
	bool split_panel_enable;
	bool multiplex_mode_enable;
	u8 multiplex_out_sel;
	bool multiplex_eoc_enable;
	bool de_raster_enable;
};

int dc9x00_dsc_configure(void __iomem *base, u8 panel_id,
			 const struct dc9x00_dsc_usage *sw_cfg,
			 const struct drm_dsc_config *dsc_cfg);

const struct drm_dsc_config *laguna_panel_get_dsc_by_mode(u16 hdisplay, u16 vdisplay);
void laguna_panel_get_te_info(u16 hdisplay, u16 vdisplay,
			      u32 *te_fps, u32 *te_usec);

void dc9x00_gpu_probe_debugfs_init(struct dentry *root);
void dc9x00_gpu_probe_debugfs_cleanup(void);

extern struct platform_driver dw_cdphy_g301_driver;
extern struct platform_driver dw_dsi2h_driver;
extern struct mipi_dsi_driver laguna_panel_driver;
extern struct platform_driver dc9x00_gpu_probe_driver;

#endif /* _DC9X00_INTERNAL_H_ */
