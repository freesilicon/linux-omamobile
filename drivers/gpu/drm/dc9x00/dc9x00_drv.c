// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room DRM/KMS driver for VeriSilicon DC9x00 on Tensor G5 Laguna.
 */

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_dumb_buffers.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_of.h>
#include <drm/drm_plane.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

#include "dc9x00_internal.h"
#include "dc9x00_regs.h"

static bool use_te_sync = true;
module_param(use_te_sync, bool, 0644);
MODULE_PARM_DESC(use_te_sync, "Synchronize DC9x00 Output0 command frames to panel TE (default: Y)");

static int te_sel;
module_param(te_sel, int, 0644);
MODULE_PARM_DESC(te_sel, "DC9x00 OUTPUT0_TE_SEL value (default: 0 matching stock vs_drm)");

static int cmd_opt;
module_param(cmd_opt, int, 0644);
MODULE_PARM_DESC(cmd_opt, "DC9x00 OUTPUT0_CMD_OPT value (default: 0 trigger mode)");

struct dc9x00_drm {
	struct drm_device drm;
	struct device *dev;
	void __iomem *base;
	void __iomem *gpcsr;
	struct dev_pm_domain_list *pd_list;
	struct drm_plane primary;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector *connector;
	struct drm_bridge *bridge;
	struct dentry *debug_dir;

	spinlock_t event_lock;
	/* Protects BE_INTR_ENABLE, frame_pending, and underrun_enabled */
	spinlock_t intr_lock;
	struct drm_pending_vblank_event *pending_event;
	struct timer_list commit_watchdog;
	atomic64_t irq_frm_start;
	atomic64_t irq_frm_done;
	atomic64_t irq_underrun;
	atomic64_t irq_te;
	atomic64_t watchdog_fires;
	bool active;
	bool frame_pending;
	bool underrun_enabled;
};

static inline struct dc9x00_drm *to_dc9x00(struct drm_device *drm)
{
	return container_of(drm, struct dc9x00_drm, drm);
}

static void dc9x00_set_underrun_irq(struct dc9x00_drm *priv, bool enable)
{
	unsigned long flags;
	u32 intr_en;

	spin_lock_irqsave(&priv->intr_lock, flags);
	if (!priv->active) {
		priv->underrun_enabled = false;
		spin_unlock_irqrestore(&priv->intr_lock, flags);
		return;
	}

	writel(DC9X00_BE_OUTPATH0_UNDERRUN, priv->base + DC9X00_BE_INTR_STATUS);
	writel(DC9X00_BE_OUTPATH0_UNDERRUN, priv->base + DC9X00_BE_INTR_OVERFLOW);

	intr_en = readl(priv->base + DC9X00_BE_INTR_ENABLE);
	if (enable)
		intr_en |= DC9X00_BE_OUTPATH0_UNDERRUN;
	else
		intr_en &= ~DC9X00_BE_OUTPATH0_UNDERRUN;
	writel(intr_en, priv->base + DC9X00_BE_INTR_ENABLE);
	priv->underrun_enabled = enable;
	spin_unlock_irqrestore(&priv->intr_lock, flags);
}

static void dc9x00_complete_pending_event(struct dc9x00_drm *priv)
{
	struct drm_pending_vblank_event *ev;
	unsigned long flags;

	spin_lock_irqsave(&priv->event_lock, flags);
	ev = priv->pending_event;
	priv->pending_event = NULL;
	if (ev) {
		spin_lock(&priv->drm.event_lock);
		drm_crtc_send_vblank_event(&priv->crtc, ev);
		spin_unlock(&priv->drm.event_lock);
	}
	spin_unlock_irqrestore(&priv->event_lock, flags);
}

static void dc9x00_commit_watchdog_fn(struct timer_list *t)
{
	struct dc9x00_drm *priv = timer_container_of(priv, t, commit_watchdog);

	dc9x00_set_underrun_irq(priv, false);
	WRITE_ONCE(priv->frame_pending, false);

	if (priv->pending_event) {
		atomic64_inc(&priv->watchdog_fires);
		if (priv->active)
			dev_warn_ratelimited(priv->dev,
					     "DC9x00 frame watchdog (fe0=%#x be0=%#x be1=%#x coord=%#x req=%#x rev=%#x)\n",
					     readl(priv->base + DC9X00_FE0_INTR_STATUS),
					     readl(priv->base + DC9X00_BE_INTR_STATUS),
					     readl(priv->base + DC9X00_BE_INTR_STATUS1),
					     readl(priv->base + DC9X00_SH_OUTPUT0_CURRENT_COORD),
					     readl(priv->base + DC9X00_BE_DEBUG_REQ_BURST_CNT),
					     readl(priv->base + DC9X00_BE_DEBUG_REV_BURST_CNT));
	}
	dc9x00_complete_pending_event(priv);
}

static irqreturn_t dc9x00_be_irq(int irq, void *data)
{
	struct dc9x00_drm *priv = data;
	u32 st0, ov0, st1, ov1;

	if (!priv->active)
		return IRQ_NONE;

	st0 = readl(priv->base + DC9X00_BE_INTR_STATUS);
	if (st0)
		writel(st0, priv->base + DC9X00_BE_INTR_STATUS);

	ov0 = readl(priv->base + DC9X00_BE_INTR_OVERFLOW);
	if (ov0)
		writel(ov0, priv->base + DC9X00_BE_INTR_OVERFLOW);

	st1 = readl(priv->base + DC9X00_BE_INTR_STATUS1);
	if (st1)
		writel(st1, priv->base + DC9X00_BE_INTR_STATUS1);

	ov1 = readl(priv->base + DC9X00_BE_INTR_OVERFLOW1);
	if (ov1)
		writel(ov1, priv->base + DC9X00_BE_INTR_OVERFLOW1);

	if (!st0 && !ov0 && !st1 && !ov1)
		return IRQ_NONE;

	if ((st0 & DC9X00_BE_OUTPATH0_UNDERRUN) &&
	    READ_ONCE(priv->underrun_enabled) &&
	    READ_ONCE(priv->frame_pending)) {
		atomic64_inc(&priv->irq_underrun);
		dev_warn_ratelimited(priv->dev, "DC9x00 Outpath0 underrun (st0=%#x)\n", st0);
	}

	if (st0 & DC9X00_BE_OUTPATH0_FRM_START) {
		atomic64_inc(&priv->irq_frm_start);
		if (!(st0 & DC9X00_BE_OUTPATH0_FRM_DONE) &&
		    READ_ONCE(priv->frame_pending))
			dc9x00_set_underrun_irq(priv, true);
	}

	if (st0 & DC9X00_BE_OUTPATH0_FRM_DONE) {
		atomic64_inc(&priv->irq_frm_done);
		dc9x00_set_underrun_irq(priv, false);
		WRITE_ONCE(priv->frame_pending, false);
	}

	if (st0 & DC9X00_BE_OUTPATH0_TE_RISING) {
		atomic64_inc(&priv->irq_te);
		drm_crtc_handle_vblank(&priv->crtc);
	}

	if ((st0 & DC9X00_BE_OUTPATH0_FRM_DONE) ||
	    (use_te_sync && (st0 & DC9X00_BE_OUTPATH0_TE_RISING))) {
		timer_delete(&priv->commit_watchdog);
		dc9x00_complete_pending_event(priv);
	}

	return IRQ_HANDLED;
}

/**
 * dc9x00_force_be_csr_read() - Flush BE CSR clock domain before clearing isolation.
 * @priv: DC9x00 device context.
 *
 * Performing 30 dummy reads of POST_PROCESS_OUT prior to clearing
 * CLOCK_DOMAIN_ISOLATION ensures the back-end CSR interface latches reliably.
 */
static void dc9x00_force_be_csr_read(struct dc9x00_drm *priv)
{
	unsigned int i;

	for (i = 0; i < 30; i++)
		readl(priv->base + DC9X00_POST_PROCESS_OUT);
}

static void dc9x00_sw_reset(struct dc9x00_drm *priv)
{
	u32 sts;

	writel(DC9X00_FE0_INTR_SW_RST_DONE, priv->base + DC9X00_FE0_INTR_ENABLE);
	writel(DC9X00_FE0_SW_RESET_RESET, priv->base + DC9X00_FE0_SW_RESET);
	if (readl_poll_timeout(priv->base + DC9X00_FE0_INTR_STATUS, sts,
			       sts & DC9X00_FE0_INTR_SW_RST_DONE, 100, 20000))
		dev_warn(priv->dev, "FE0 SW reset poll timeout\n");
	writel(DC9X00_FE0_INTR_SW_RST_DONE, priv->base + DC9X00_FE0_INTR_STATUS);
	writel(0, priv->base + DC9X00_FE0_INTR_ENABLE);

	writel(DC9X00_BE_INTR1_SW_RST_DONE, priv->base + DC9X00_BE_INTR_ENABLE1);
	writel(DC9X00_BE_SW_RESET_RESET, priv->base + DC9X00_BE_SW_RESET);
	if (readl_poll_timeout(priv->base + DC9X00_BE_INTR_STATUS1, sts,
			       sts & DC9X00_BE_INTR1_SW_RST_DONE, 100, 20000))
		dev_warn(priv->dev, "BE SW reset poll timeout\n");
	writel(DC9X00_BE_INTR1_SW_RST_DONE, priv->base + DC9X00_BE_INTR_STATUS1);
	writel(0, priv->base + DC9X00_BE_INTR_ENABLE1);
}

static int dc9x00_plane_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(new_plane_state->state, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	return drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, true);
}

static void dc9x00_plane_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct dc9x00_drm *priv = to_dc9x00(plane->dev);
	struct drm_plane_state *pstate = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = pstate->fb;
	struct drm_gem_dma_object *dma_obj;
	dma_addr_t paddr;
	u32 fmt, cfg, w, h;

	if (!priv->active || !fb || !pstate->visible) {
		if (priv->active) {
			writel(0, priv->base + DC9X00_SH_LAYER0_CONFIG);
			writel(DC9X00_LAYER0_REG_SWITCH, priv->base + DC9X00_LAYER0_CONFIG);
		}
		return;
	}

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	paddr = dma_obj->dma_addr + fb->offsets[0];
	w = drm_rect_width(&pstate->dst);
	h = drm_rect_height(&pstate->dst);

	switch (fb->format->format) {
	case DRM_FORMAT_ARGB8888:
		fmt = DC9X00_LAYER_CFG_FMT_A8R8G8B8;
		break;
	case DRM_FORMAT_XRGB8888:
	default:
		fmt = DC9X00_LAYER_CFG_FMT_X8R8G8B8;
		break;
	}

	writel(0, priv->base + DC9X00_LAYER0_OUTPUT_PATH_ID);
	writel(0, priv->base + DC9X00_SH_LAYER0_BLEND_STACK_ID);
	writel(0, priv->base + DC9X00_SH_LAYER0_ALPHA_BLEND_CFG);

	writel(lower_32_bits(paddr), priv->base + DC9X00_SH_LAYER0_ADDR);
	writel(upper_32_bits(paddr) & DC9X00_SH_LAYER0_HIGH_ADDR_MASK,
	       priv->base + DC9X00_SH_LAYER0_HIGH_ADDR);
	writel(fb->pitches[0], priv->base + DC9X00_SH_LAYER0_STRIDE);
	writel((w & 0xffff) | ((h & 0xffff) << 16), priv->base + DC9X00_SH_LAYER0_SIZE);
	writel(DC9X00_LAYER_DMA_MODE_NORMAL, priv->base + DC9X00_SH_LAYER0_DMA_MODE);
	writel(0, priv->base + DC9X00_SH_LAYER0_OUT_ROI_ORIGIN);
	writel((w & 0xffff) | ((h & 0xffff) << 16), priv->base + DC9X00_SH_LAYER0_OUT_ROI_SIZE);

	cfg = DC9X00_LAYER_CFG_ENABLE |
	      DC9X00_LAYER_CFG_BYPASS_HDR |
	      FIELD_PREP(DC9X00_LAYER_CFG_SWIZZLE_MASK, DC9X00_LAYER_CFG_SWIZZLE_ARGB) |
	      FIELD_PREP(DC9X00_LAYER_CFG_TILE_MASK, DC9X00_LAYER_CFG_TILE_LINEAR) |
	      FIELD_PREP(DC9X00_LAYER_CFG_FORMAT_MASK, fmt);
	writel(cfg, priv->base + DC9X00_SH_LAYER0_CONFIG);
}

static void dc9x00_plane_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_commit *state)
{
	struct dc9x00_drm *priv = to_dc9x00(plane->dev);

	if (!priv->active)
		return;

	writel(0, priv->base + DC9X00_SH_LAYER0_CONFIG);
	writel(DC9X00_LAYER0_REG_SWITCH, priv->base + DC9X00_LAYER0_CONFIG);
}

static const struct drm_plane_helper_funcs dc9x00_plane_helper_funcs = {
	.atomic_check = dc9x00_plane_atomic_check,
	.atomic_update = dc9x00_plane_atomic_update,
	.atomic_disable = dc9x00_plane_atomic_disable,
};

static const struct drm_plane_funcs dc9x00_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static void dc9x00_crtc_atomic_enable(struct drm_crtc *crtc,
				      struct drm_atomic_commit *state)
{
	struct dc9x00_drm *priv = to_dc9x00(crtc->dev);
	struct drm_crtc_state *cstate = drm_atomic_get_new_crtc_state(state, crtc);
	const struct drm_display_mode *mode = &cstate->adjusted_mode;
	const struct drm_dsc_config *dsc_cfg;
	struct dc9x00_dsc_usage dsc_usage = {
		.enable = true,
		.video_mode = false,
		.slices_per_line = 2,
		.ss_num = 1,
		.split_panel_enable = true,
		.multiplex_mode_enable = true,
		.de_raster_enable = true,
	};
	u32 w = mode->hdisplay;
	u32 h = mode->vdisplay;
	u32 hfp = mode->hsync_start - mode->hdisplay;
	u32 hsa = mode->hsync_end - mode->hsync_start;
	u32 hbp = mode->htotal - mode->hsync_end;
	u32 vfp = mode->vsync_start - mode->vdisplay;
	u32 vsa = mode->vsync_end - mode->vsync_start;
	u32 vbp = mode->vtotal - mode->vsync_end;
	u32 te_fps, te_usec, sys_us, active_us, line_cycles, sys_cycles, urgent;
	int ret;

	ret = pm_runtime_resume_and_get(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to power on DPU domains: %d\n", ret);
		return;
	}

	dev_info(priv->dev, "DC9x00 Chip ID=%#x Rev=%#x Prod=%#x (%ux%u)\n",
		 readl(priv->base + DC9X00_CHIP_ID),
		 readl(priv->base + DC9X00_CHIP_REV),
		 readl(priv->base + DC9X00_PRODUCT_ID), w, h);

	dc9x00_sw_reset(priv);
	dc9x00_force_be_csr_read(priv);
	writel(0x0, priv->base + DC9X00_CLOCK_DOMAIN_ISOLATION);
	readl(priv->base + DC9X00_CLOCK_DOMAIN_ISOLATION);

	/* Output0 clock must be enabled before writing Output0 CSRs */
	writel(1, priv->base + DC9X00_SH_OUTPUT0_CLK_EN);
	readl(priv->base + DC9X00_SH_OUTPUT0_CLK_EN);

	writel(readl(priv->base + DC9X00_BE_INTR_STATUS), priv->base + DC9X00_BE_INTR_STATUS);
	writel(readl(priv->base + DC9X00_BE_INTR_STATUS1), priv->base + DC9X00_BE_INTR_STATUS1);
	writel(readl(priv->base + DC9X00_BE_INTR_OVERFLOW), priv->base + DC9X00_BE_INTR_OVERFLOW);

	priv->underrun_enabled = false;
	priv->frame_pending = false;
	writel(DC9X00_BE_OUTPATH0_FRM_START | DC9X00_BE_OUTPATH0_FRM_DONE |
	       DC9X00_BE_OUTPATH0_TE_RISING,
	       priv->base + DC9X00_BE_INTR_ENABLE);

	writel(DC9X00_SH_PANEL0_OUTPUT_PATH, priv->base + DC9X00_SH_PANEL0_CONFIG);
	writel(DC9X00_SH_PANEL0_FMT_RGB888, priv->base + DC9X00_SH_PANEL0_FORMAT);
	writel(0, priv->base + DC9X00_SH_PANEL0_BG_ALPHA);
	writel(0, priv->base + DC9X00_SH_PANEL0_BG_RED);
	writel(0, priv->base + DC9X00_SH_PANEL0_BG_GREEN);
	writel(0, priv->base + DC9X00_SH_PANEL0_BG_BLUE);
	writel(w, priv->base + DC9X00_SH_PANEL0_IMAGE_WIDTH);
	writel(h, priv->base + DC9X00_SH_PANEL0_IMAGE_HEIGHT);
	writel(w, priv->base + DC9X00_SH_PANEL0_DITHER_WIDTH);
	writel(h, priv->base + DC9X00_SH_PANEL0_DITHER_HEIGHT);
	writel(w, priv->base + DC9X00_SH_PANEL0_WIDTH);
	writel(h, priv->base + DC9X00_SH_PANEL0_HEIGHT);

	dsc_cfg = laguna_panel_get_dsc_by_mode(w, h);
	if (!dsc_cfg)
		dsc_usage.enable = false;
	dc9x00_dsc_configure(priv->base, 0, &dsc_usage, dsc_cfg);

	writel(DC9X00_OUTPUT0_WORK_MODE_CMD, priv->base + DC9X00_OUTPUT0_CTRL);
	writel(cmd_opt & 0x3, priv->base + DC9X00_OUTPUT0_CMD_OPT);
	writel(0, priv->base + DC9X00_OUTPUT0_DE_SYNC_MODE);
	writel(te_sel & 0x1, priv->base + DC9X00_OUTPUT0_TE_SEL);
	writel(0, priv->base + DC9X00_OUTPUT0_TE_POLARITY);

	writel((mode->flags & DRM_MODE_FLAG_NHSYNC) ? 1 : 0,
	       priv->base + DC9X00_OUTPUT0_TIMING_HSYNC);
	writel(hfp, priv->base + DC9X00_SH_OUTPUT0_TIMING_HFP);
	writel(w, priv->base + DC9X00_SH_OUTPUT0_TIMING_HA);
	writel(hbp, priv->base + DC9X00_SH_OUTPUT0_TIMING_HBP);
	writel(hsa, priv->base + DC9X00_SH_OUTPUT0_TIMING_HS);
	writel((mode->flags & DRM_MODE_FLAG_NVSYNC) ? 1 : 0,
	       priv->base + DC9X00_OUTPUT0_TIMING_VSYNC);
	writel(vfp, priv->base + DC9X00_SH_OUTPUT0_TIMING_VFP);
	writel(h, priv->base + DC9X00_SH_OUTPUT0_TIMING_VA);
	writel(vbp, priv->base + DC9X00_SH_OUTPUT0_TIMING_VBP);
	writel(vsa, priv->base + DC9X00_SH_OUTPUT0_TIMING_VS);

	writel(dsc_usage.enable ? DC9X00_IPI_FMT_COMPRESSED : DC9X00_IPI_FMT_RGB,
	       priv->base + DC9X00_SH_OUTPUT0_IPI_FORMAT);
	writel(DC9X00_IPI_DEPTH_BITS8, priv->base + DC9X00_SH_OUTPUT0_IPI_DEPTH);

	laguna_panel_get_te_info(w, h, &te_fps, &te_usec);
	sys_us = USEC_PER_SEC / max_t(u32, 1, te_fps);
	active_us = sys_us > te_usec ? sys_us - te_usec : sys_us;
	line_cycles = mult_frac(active_us, 100 * 99, 100 * max_t(u32, 1, h));
	writel((line_cycles & 0xffff) | ((h & 0xffff) << 16),
	       priv->base + DC9X00_SH_OUTPUT0_SCANOUT_COUNTER);
	writel(0, priv->base + DC9X00_OUTPUT0_SCANOUT_DELAY_COUNTER);

	sys_cycles = mult_frac(sys_us, 100 * 90, 100 * max_t(u32, 1, h));
	writel((sys_cycles & 0xffff) | ((h & 0xffff) << 16),
	       priv->base + DC9X00_SH_OUTPUT0_SYSTEM_COUNTER);
	writel(200 * 100, priv->base + DC9X00_OUTPUT0_SYS_DELAY_COUNTER);

	urgent = readl(priv->base + DC9X00_SH_OUTPUT0_URGENT_VALUE);
	urgent = (urgent & ~DC9X00_URGENT_VALUE_MASK) |
		 FIELD_PREP(DC9X00_URGENT_VALUE_MASK, 3);
	writel(urgent, priv->base + DC9X00_SH_OUTPUT0_URGENT_VALUE);

	priv->active = true;
	drm_crtc_vblank_on(crtc);
}

static void dc9x00_crtc_atomic_disable(struct drm_crtc *crtc,
				       struct drm_atomic_commit *state)
{
	struct dc9x00_drm *priv = to_dc9x00(crtc->dev);
	unsigned long flags;

	timer_delete_sync(&priv->commit_watchdog);
	dc9x00_complete_pending_event(priv);
	drm_crtc_vblank_off(crtc);

	if (!priv->active)
		return;

	spin_lock_irqsave(&priv->intr_lock, flags);
	priv->active = false;
	priv->frame_pending = false;
	priv->underrun_enabled = false;
	writel(0, priv->base + DC9X00_BE_INTR_ENABLE);
	spin_unlock_irqrestore(&priv->intr_lock, flags);

	writel(0, priv->base + DC9X00_OUTPUT0_SW_CONFIG);
	writel(0, priv->base + DC9X00_OUTPUT0_START);
	writel(0, priv->base + DC9X00_SH_OUTPUT0_CLK_EN);
	writel(0xf, priv->base + DC9X00_CLOCK_DOMAIN_ISOLATION);

	pm_runtime_put_sync(priv->dev);
}

static void dc9x00_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_commit *state)
{
	struct dc9x00_drm *priv = to_dc9x00(crtc->dev);
	struct drm_crtc_state *cstate = drm_atomic_get_new_crtc_state(state, crtc);
	u32 mux;
	unsigned long flags;

	if (!priv->active) {
		if (cstate->event) {
			spin_lock_irqsave(&crtc->dev->event_lock, flags);
			drm_crtc_send_vblank_event(crtc, cstate->event);
			cstate->event = NULL;
			spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		}
		return;
	}

	writel(DC9X00_LAYER0_REG_SWITCH, priv->base + DC9X00_LAYER0_CONFIG);
	writel(DC9X00_PANEL0_REG_SWITCH, priv->base + DC9X00_PANEL0_CONFIG);
	writel(DC9X00_OUTPUT0_REG_SWITCH, priv->base + DC9X00_OUTPUT0_CONFIG);

	mux = readl(priv->base + DC9X00_POST_PROCESS_OUT);
	mux &= ~DC9X00_POST_MUX_OUT0_MASK;
	writel(mux, priv->base + DC9X00_POST_PROCESS_OUT);

	if (cstate->event) {
		spin_lock_irqsave(&priv->event_lock, flags);
		WARN_ON(priv->pending_event);
		priv->pending_event = cstate->event;
		cstate->event = NULL;
		spin_unlock_irqrestore(&priv->event_lock, flags);
		mod_timer(&priv->commit_watchdog, jiffies + msecs_to_jiffies(50));
	}

	spin_lock_irqsave(&priv->intr_lock, flags);
	priv->frame_pending = true;
	writel(DC9X00_BE_OUTPATH0_FRM_START | DC9X00_BE_OUTPATH0_UNDERRUN,
	       priv->base + DC9X00_BE_INTR_STATUS);
	spin_unlock_irqrestore(&priv->intr_lock, flags);

	writel(0x1, priv->base + DC9X00_OUTPUT0_START);
	writel(0x1, priv->base + DC9X00_OUTPUT0_SW_CONFIG);
}

static const struct drm_crtc_helper_funcs dc9x00_crtc_helper_funcs = {
	.atomic_enable = dc9x00_crtc_atomic_enable,
	.atomic_disable = dc9x00_crtc_atomic_disable,
	.atomic_flush = dc9x00_crtc_atomic_flush,
};

static const struct drm_crtc_funcs dc9x00_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = drm_crtc_cleanup,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs dc9x00_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dc9x00_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
			      struct drm_mode_create_dumb *args)
{
	int ret;

	ret = drm_mode_size_dumb(dev, args, 128, 0);
	if (ret)
		return ret;

	return drm_gem_dma_dumb_create_internal(file_priv, dev, args);
}

static void dc9x00_atomic_commit_tail(struct drm_atomic_commit *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	/*
	 * Enable the DSI2H/CDPHY/panel chain before resetting BE and bringing
	 * up DC9x00 Output0, then commit planes after priv->active is true.
	 */
	drm_atomic_helper_commit_encoder_bridge_pre_enable(dev, state);
	drm_atomic_helper_commit_encoder_bridge_enable(dev, state);
	drm_atomic_helper_commit_crtc_enable(dev, state);

	drm_atomic_helper_commit_planes(dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_vblanks(dev, state);
	drm_atomic_helper_cleanup_planes(dev, state);
}

static const struct drm_mode_config_helper_funcs dc9x00_mode_config_helper_funcs = {
	.atomic_commit_tail = dc9x00_atomic_commit_tail,
};

static const struct drm_mode_config_funcs dc9x00_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(dc9x00_fops);

static const struct drm_driver dc9x00_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops = &dc9x00_fops,
	.name = "dc9x00",
	.desc = "VeriSilicon DC9x00 Display Controller (Laguna)",
	.major = 1,
	.minor = 0,
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(dc9x00_dumb_create),
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int dc9x00_setup_irqs(struct platform_device *pdev, struct dc9x00_drm *priv)
{
	static const char * const irq_names[] = {
		"outpath0_frm_start",
		"outpath0_frm_done",
		"outpath0_underrun",
		"output0_te_rising_edge",
	};
	unsigned int i;
	int irq, ret;

	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		irq = platform_get_irq_byname_optional(pdev, irq_names[i]);
		if (irq < 0)
			continue;

		ret = devm_request_irq(priv->dev, irq, dc9x00_be_irq,
				       IRQF_SHARED, irq_names[i], priv);
		if (ret)
			return ret;
	}

	return 0;
}

#define DC9X00_DPU_SMMU_BASE	0x0ed80000
#define DC9X00_DSI2H0_BASE	0x0efd8000

struct dc9x00_reg_dump_entry {
	const char *name;
	u32 off;
};

static void dc9x00_dump_reg_table(struct seq_file *m, void __iomem *base,
				  const struct dc9x00_reg_dump_entry *regs,
				  size_t count)
{
	size_t i;

	for (i = 0; i < count; i++)
		seq_printf(m, "%-19s = %#010x\n", regs[i].name,
			   readl(base + regs[i].off));
}

static int dc9x00_display_regs_show(struct seq_file *m, void *data)
{
	static const struct dc9x00_reg_dump_entry dc_regs[] = {
		{ "FE0_INTR_STATUS", DC9X00_FE0_INTR_STATUS },
		{ "SH_LAYER0_ADDR", DC9X00_SH_LAYER0_ADDR },
		{ "SH_LAYER0_HIGH", DC9X00_SH_LAYER0_HIGH_ADDR },
		{ "SH_LAYER0_STRIDE", DC9X00_SH_LAYER0_STRIDE },
		{ "SH_LAYER0_SIZE", DC9X00_SH_LAYER0_SIZE },
		{ "SH_LAYER0_CONFIG", DC9X00_SH_LAYER0_CONFIG },
		{ "SH_LAYER0_DMA_MODE", DC9X00_SH_LAYER0_DMA_MODE },
		{ "BE_REG_READ_SRC", DC9X00_BE_REG_READ_SRC },
		{ "BE_INTR_ENABLE", DC9X00_BE_INTR_ENABLE },
		{ "BE_INTR_STATUS", DC9X00_BE_INTR_STATUS },
		{ "BE_INTR_OVERFLOW", DC9X00_BE_INTR_OVERFLOW },
		{ "BE_INTR_STATUS1", DC9X00_BE_INTR_STATUS1 },
		{ "BE_REQ_BURST_CNT", DC9X00_BE_DEBUG_REQ_BURST_CNT },
		{ "BE_REV_BURST_CNT", DC9X00_BE_DEBUG_REV_BURST_CNT },
		{ "CLK_DOMAIN_ISO", DC9X00_CLOCK_DOMAIN_ISOLATION },
		{ "SH_PANEL0_CONFIG", DC9X00_SH_PANEL0_CONFIG },
		{ "SH_PANEL0_SPLIT", DC9X00_SH_PANEL0_SPLIT_CONFIG },
		{ "SH_PANEL0_WIDTH", DC9X00_SH_PANEL0_WIDTH },
		{ "SH_PANEL0_HEIGHT", DC9X00_SH_PANEL0_HEIGHT },
		{ "DSC_ENC_MAIN_CONF", DC9X00_PANEL0_DSC_BASE },
		{ "DSC_HS0_DF_CTRL", DC9X00_PANEL0_DSC_BASE + 0x100 },
		{ "DSC_HS0_GEN_STATUS", DC9X00_PANEL0_DSC_BASE + 0x104 },
		{ "OUTPUT0_START", DC9X00_OUTPUT0_START },
		{ "OUTPUT0_SW_CONFIG", DC9X00_OUTPUT0_SW_CONFIG },
		{ "OUTPUT0_CTRL", DC9X00_OUTPUT0_CTRL },
		{ "OUTPUT0_CMD_OPT", DC9X00_OUTPUT0_CMD_OPT },
		{ "OUTPUT0_DE_SYNC", DC9X00_OUTPUT0_DE_SYNC_MODE },
		{ "OUTPUT0_TE_SEL", DC9X00_OUTPUT0_TE_SEL },
		{ "SH_OUTPUT0_IPI_FMT", DC9X00_SH_OUTPUT0_IPI_FORMAT },
		{ "SH_OUTPUT0_IPI_DEP", DC9X00_SH_OUTPUT0_IPI_DEPTH },
		{ "SH_OUTPUT0_COORD", DC9X00_SH_OUTPUT0_CURRENT_COORD },
		{ "OUTPUT0_CONFIG", DC9X00_OUTPUT0_CONFIG },
		{ "SH_OUTPUT0_SYS_CNT", DC9X00_SH_OUTPUT0_SYSTEM_COUNTER },
		{ "SH_OUTPUT0_SCN_CNT", DC9X00_SH_OUTPUT0_SCANOUT_COUNTER },
		{ "SH_OUTPUT0_SCN_DLY", DC9X00_OUTPUT0_SCANOUT_DELAY_COUNTER },
		{ "OUTPUT0_SYS_DLY", DC9X00_OUTPUT0_SYS_DELAY_COUNTER },
		{ "SH_OUTPUT0_URGENT", DC9X00_SH_OUTPUT0_URGENT_VALUE },
		{ "SH_OUTPUT0_CLK_EN", DC9X00_SH_OUTPUT0_CLK_EN },
		{ "POST_PROCESS_OUT", DC9X00_POST_PROCESS_OUT },
	};
	static const struct dc9x00_reg_dump_entry smmu_table[] = {
		{ "DPU_SMMU_IDR0", 0x00 },
		{ "DPU_SMMU_IDR1", 0x04 },
		{ "DPU_SMMU_CR0", 0x20 },
		{ "DPU_SMMU_CR0ACK", 0x24 },
		{ "DPU_SMMU_GBPA", 0x44 },
		{ "DPU_SMMU_GERROR", 0x60 },
	};
	static const struct dc9x00_reg_dump_entry dsi2h_table[] = {
		{ "DSI2H_PWR_UP", DSI2H_PWR_UP },
		{ "DSI2H_MODE_CTRL", DSI2H_MODE_CTRL },
		{ "DSI2H_MODE_STATUS", DSI2H_MODE_STATUS },
		{ "DSI2H_CORE_STATUS", DSI2H_CORE_STATUS },
		{ "DSI2H_PHY_STATUS", 0x108 },
		{ "DSI2H_IPI_COLOR", 0x300 },
		{ "DSI2H_IPI_PIX_PKT", 0x344 },
		{ "DSI2H_INT_ST_PHY", 0x400 },
		{ "DSI2H_INT_ST_TO", 0x410 },
		{ "DSI2H_INT_ST_IPI", 0x430 },
	};
	struct dc9x00_drm *priv = m->private;
	void __iomem *smmu_regs;
	void __iomem *dsi2h_regs;
	int ret;

	seq_printf(m, "active=%d use_te_sync=%d te_sel=%d cmd_opt=%d\n",
		   priv->active, use_te_sync, te_sel, cmd_opt);
	seq_printf(m, "irqs: frm_start=%lld frm_done=%lld underrun=%lld te=%lld watchdog=%lld\n",
		   atomic64_read(&priv->irq_frm_start),
		   atomic64_read(&priv->irq_frm_done),
		   atomic64_read(&priv->irq_underrun),
		   atomic64_read(&priv->irq_te),
		   atomic64_read(&priv->watchdog_fires));

	ret = pm_runtime_get_if_active(priv->dev);
	if (ret <= 0) {
		seq_puts(m, "power=suspended\n");
		return 0;
	}

	dc9x00_dump_reg_table(m, priv->base, dc_regs, ARRAY_SIZE(dc_regs));

	smmu_regs = ioremap(DC9X00_DPU_SMMU_BASE, 0x100);
	if (smmu_regs) {
		dc9x00_dump_reg_table(m, smmu_regs, smmu_table,
				      ARRAY_SIZE(smmu_table));
		iounmap(smmu_regs);
	}

	dsi2h_regs = ioremap(DC9X00_DSI2H0_BASE, 0x500);
	if (dsi2h_regs) {
		dc9x00_dump_reg_table(m, dsi2h_regs, dsi2h_table,
				      ARRAY_SIZE(dsi2h_table));
		iounmap(dsi2h_regs);
	}

	pm_runtime_put(priv->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dc9x00_display_regs);

static int dc9x00_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	static const char * const pd_names[] = { "dpu_dsi0_pd", "dpu_fe0_pd" };
	const struct dev_pm_domain_attach_data pd_cfg = {
		.pd_names = pd_names,
		.num_pd_names = ARRAY_SIZE(pd_names),
		.pd_flags = PD_FLAG_DEV_LINK_ON,
	};
	static const u32 formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888 };
	struct drm_bridge *bridge;
	struct dc9x00_drm *priv;
	struct drm_device *drm;
	int ret;

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "waiting for DSI2H bridge\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	priv = devm_drm_dev_alloc(dev, &dc9x00_drm_driver, struct dc9x00_drm, drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	drm = &priv->drm;
	priv->dev = dev;
	priv->bridge = bridge;
	spin_lock_init(&priv->event_lock);
	spin_lock_init(&priv->intr_lock);
	timer_setup(&priv->commit_watchdog, dc9x00_commit_watchdog_fn, 0);

	priv->base = devm_platform_ioremap_resource_byname(pdev, "dpu-fe-be");
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->gpcsr = devm_platform_ioremap_resource_byname(pdev, "dpu-gpcsr");
	if (IS_ERR(priv->gpcsr))
		priv->gpcsr = NULL;

	ret = devm_pm_domain_attach_list(dev, &pd_cfg, &priv->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to attach DPU power domains\n");

	pm_runtime_enable(dev);

	ret = dc9x00_setup_irqs(pdev, priv);
	if (ret)
		goto err_pm;

	ret = drmm_mode_config_init(drm);
	if (ret)
		goto err_pm;

	drm->mode_config.min_width = 16;
	drm->mode_config.max_width = 4096;
	drm->mode_config.min_height = 16;
	drm->mode_config.max_height = 4096;
	drm->mode_config.funcs = &dc9x00_mode_config_funcs;
	drm->mode_config.helper_private = &dc9x00_mode_config_helper_funcs;

	ret = drm_universal_plane_init(drm, &priv->primary, 1, &dc9x00_plane_funcs,
				       formats, ARRAY_SIZE(formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, "dc9x00-layer0");
	if (ret)
		goto err_pm;
	drm_plane_helper_add(&priv->primary, &dc9x00_plane_helper_funcs);

	ret = drm_plane_create_blend_mode_property(&priv->primary,
						   BIT(DRM_MODE_BLEND_PIXEL_NONE));
	if (ret)
		goto err_pm;

	ret = drm_crtc_init_with_planes(drm, &priv->crtc, &priv->primary, NULL,
					&dc9x00_crtc_funcs, "dc9x00-crtc0");
	if (ret)
		goto err_pm;
	drm_crtc_helper_add(&priv->crtc, &dc9x00_crtc_helper_funcs);

	priv->encoder.possible_crtcs = drm_crtc_mask(&priv->crtc);
	ret = drm_encoder_init(drm, &priv->encoder, &dc9x00_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, "dc9x00-dsi0");
	if (ret)
		goto err_pm;

	ret = drm_bridge_attach(&priv->encoder, priv->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		goto err_pm;

	priv->connector = drm_bridge_connector_init(drm, &priv->encoder);
	if (IS_ERR(priv->connector)) {
		ret = PTR_ERR(priv->connector);
		goto err_pm;
	}
	drm_connector_attach_encoder(priv->connector, &priv->encoder);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_pm;

	drm_mode_config_reset(drm);

	platform_set_drvdata(pdev, priv);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_pm;

	drm_client_setup(drm, NULL);

	priv->debug_dir = debugfs_create_dir("dc9x00", NULL);
	debugfs_create_file("display_regs", 0444, priv->debug_dir, priv,
			    &dc9x00_display_regs_fops);
	dc9x00_gpu_probe_debugfs_init(priv->debug_dir);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void dc9x00_remove(struct platform_device *pdev)
{
	struct dc9x00_drm *priv = platform_get_drvdata(pdev);

	dc9x00_gpu_probe_debugfs_cleanup();
	debugfs_remove_recursive(priv->debug_dir);
	drm_dev_unplug(&priv->drm);
	drm_atomic_helper_shutdown(&priv->drm);
	timer_delete_sync(&priv->commit_watchdog);
	pm_runtime_disable(&pdev->dev);
}

static void dc9x00_shutdown(struct platform_device *pdev)
{
	struct dc9x00_drm *priv = platform_get_drvdata(pdev);

	if (priv)
		drm_atomic_helper_shutdown(&priv->drm);
}

static const struct of_device_id dc9x00_of_match[] = {
	{ .compatible = "verisilicon,dc9x00" },
	{}
};
MODULE_DEVICE_TABLE(of, dc9x00_of_match);

static struct platform_driver dc9x00_platform_driver = {
	.probe = dc9x00_probe,
	.remove = dc9x00_remove,
	.shutdown = dc9x00_shutdown,
	.driver = {
		.name = "dc9x00",
		.of_match_table = dc9x00_of_match,
	},
};

static int __init dc9x00_module_init(void)
{
	int ret;

	ret = platform_driver_register(&dw_cdphy_g301_driver);
	if (ret)
		return ret;

	ret = mipi_dsi_driver_register(&laguna_panel_driver);
	if (ret)
		goto err_panel;

	ret = platform_driver_register(&dw_dsi2h_driver);
	if (ret)
		goto err_dsi2h;

	ret = platform_driver_register(&dc9x00_gpu_probe_driver);
	if (ret)
		goto err_gpu;

	ret = platform_driver_register(&dc9x00_platform_driver);
	if (ret)
		goto err_dc;

	return 0;

err_dc:
	platform_driver_unregister(&dc9x00_gpu_probe_driver);
err_gpu:
	platform_driver_unregister(&dw_dsi2h_driver);
err_dsi2h:
	mipi_dsi_driver_unregister(&laguna_panel_driver);
err_panel:
	platform_driver_unregister(&dw_cdphy_g301_driver);
	return ret;
}

static void __exit dc9x00_module_exit(void)
{
	platform_driver_unregister(&dc9x00_platform_driver);
	platform_driver_unregister(&dc9x00_gpu_probe_driver);
	platform_driver_unregister(&dw_dsi2h_driver);
	mipi_dsi_driver_unregister(&laguna_panel_driver);
	platform_driver_unregister(&dw_cdphy_g301_driver);
}

module_init(dc9x00_module_init);
module_exit(dc9x00_module_exit);

MODULE_DESCRIPTION("Clean-room VeriSilicon DC9x00 / DW-DSI2H / G301 DRM Driver for Laguna");
MODULE_LICENSE("GPL");
