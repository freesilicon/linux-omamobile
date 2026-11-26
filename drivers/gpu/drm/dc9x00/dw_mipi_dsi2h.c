// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room DesignWare MIPI DSI-2 Host driver for Tensor G5 Laguna.
 */

#include <drm/display/drm_dsc.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <video/mipi_display.h>

#include "dc9x00_internal.h"
#include "dc9x00_regs.h"

#define DSI2H_POLL_INTERVAL_US		50
#define DSI2H_POLL_TIMEOUT_US		50000

struct dw_dsi2h {
	struct device *dev;
	void __iomem *regs;
	struct phy *phy;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct mipi_dsi_device *dsi_dev;
	struct drm_display_mode mode;
	struct mutex lock;

	u32 datarate_mbps;
	u32 sys_clk_khz;
	u32 ipi_clk_khz;
	u32 lptx_clk_khz;
	u32 lp2hs_time;
	u32 hs2lp_time;
	u32 lanes;
	u32 ppi_width;
	bool dsc_enabled;
	bool host_ready;
};

static inline struct dw_dsi2h *host_to_dsi2h(struct mipi_dsi_host *host)
{
	return container_of(host, struct dw_dsi2h, dsi_host);
}

static inline struct dw_dsi2h *bridge_to_dsi2h(struct drm_bridge *bridge)
{
	return container_of(bridge, struct dw_dsi2h, bridge);
}

static void dw_dsi2h_dump_diag(struct dw_dsi2h *ctx, const char *ctx_str)
{
	dev_err(ctx->dev,
		"DSI2H %s: core=%#x mode=%#x phy=%#x int_main=%#x int_phy=%#x int_to=%#x int_cri=%#x\n",
		ctx_str,
		readl(ctx->regs + DSI2H_CORE_STATUS),
		readl(ctx->regs + DSI2H_MODE_STATUS),
		readl(ctx->regs + DSI2H_PHY_STATUS),
		readl(ctx->regs + DSI2H_INT_ST_MAIN),
		readl(ctx->regs + DSI2H_INT_ST_PHY),
		readl(ctx->regs + DSI2H_INT_ST_TO),
		readl(ctx->regs + DSI2H_INT_ST_CRI));
}

static int dw_dsi2h_set_op_mode(struct dw_dsi2h *ctx, u32 mode)
{
	u32 sts;
	int ret;

	writel(mode & 0x7, ctx->regs + DSI2H_MODE_CTRL);
	ret = readl_poll_timeout(ctx->regs + DSI2H_MODE_STATUS, sts,
				 (sts & 0x7) == mode, DSI2H_POLL_INTERVAL_US,
				 DSI2H_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(ctx->dev, "DSI2H mode %u timeout (status=%#x)\n", mode, sts);
		dw_dsi2h_dump_diag(ctx, "mode_timeout");
	}
	return ret;
}

static int dw_dsi2h_wait_cri_idle(struct dw_dsi2h *ctx)
{
	u32 sts;
	int ret;

	ret = readl_poll_timeout(ctx->regs + DSI2H_CORE_STATUS, sts,
				 !(sts & DSI2H_CORE_CRI_BUSY),
				 DSI2H_POLL_INTERVAL_US, DSI2H_POLL_TIMEOUT_US);
	if (ret)
		dw_dsi2h_dump_diag(ctx, "cri_idle_timeout");
	return ret;
}

static ssize_t dw_dsi2h_cri_read(struct dw_dsi2h *ctx, const struct mipi_dsi_msg *msg)
{
	u32 sts, hdr, word;
	u8 dt, lsb, msb;
	u8 *dst = msg->rx_buf;
	size_t remain, rcv = 0;
	int ret;

	ret = dw_dsi2h_wait_cri_idle(ctx);
	if (ret)
		return ret;

	ret = readl_poll_timeout(ctx->regs + DSI2H_CORE_STATUS, sts,
				 sts & DSI2H_CORE_CRI_RD_DATA_AVAIL,
				 DSI2H_POLL_INTERVAL_US, DSI2H_POLL_TIMEOUT_US);
	if (ret) {
		dw_dsi2h_dump_diag(ctx, "cri_rx_timeout");
		return ret;
	}

	while (readl(ctx->regs + DSI2H_CORE_STATUS) & DSI2H_CORE_CRI_RD_DATA_AVAIL) {
		hdr = readl(ctx->regs + DSI2H_CRI_RX_HDR);
		dt = hdr & 0x3f;
		lsb = (hdr >> 8) & 0xff;
		msb = (hdr >> 16) & 0xff;

		switch (dt) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (rcv + 1 < msg->rx_len) {
				dst[rcv++] = lsb;
				dst[rcv++] = msb;
			} else if (rcv < msg->rx_len) {
				dst[rcv++] = lsb;
			}
			break;
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			if (rcv < msg->rx_len)
				dst[rcv++] = lsb;
			break;
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
			remain = ((size_t)msb << 8) | lsb;
			while (remain > 0 &&
			       (readl(ctx->regs + DSI2H_CORE_STATUS) & DSI2H_CORE_CRI_RD_DATA_AVAIL)) {
				size_t copy = min_t(size_t, 4, msg->rx_len - rcv);

				word = readl(ctx->regs + DSI2H_CRI_RX_PLD);
				if (copy) {
					memcpy(dst + rcv, &word, copy);
					rcv += copy;
				}
				remain = (remain > 4) ? remain - 4 : 0;
			}
			break;
		default:
			break;
		}
	}

	return rcv;
}

static ssize_t dw_dsi2h_host_transfer(struct mipi_dsi_host *host,
				      const struct mipi_dsi_msg *msg)
{
	struct dw_dsi2h *ctx = host_to_dsi2h(host);
	struct mipi_dsi_packet pkt;
	const u8 *tx;
	size_t len;
	u32 word;
	ssize_t ret;

	if (!msg->tx_buf && msg->tx_len)
		return -EINVAL;
	if (!msg->tx_len)
		return 0;

	ret = mipi_dsi_create_packet(&pkt, msg);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	if (!ctx->host_ready) {
		ret = -EPERM;
		goto out_unlock;
	}

	ret = dw_dsi2h_wait_cri_idle(ctx);
	if (ret)
		goto out_unlock;

	if (msg->rx_len) {
		word = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE |
		       ((msg->rx_len & 0xff) << 8) |
		       (((msg->rx_len >> 8) & 0xff) << 16);
		if (msg->flags & MIPI_DSI_MSG_USE_LPM)
			word |= DSI2H_CRI_TX_MODE_LP;
		writel(word, ctx->regs + DSI2H_CRI_TX_HDR);
	}

	tx = pkt.payload;
	len = pkt.payload_length;
	while (len) {
		size_t chunk = min_t(size_t, len, 4);

		word = 0;
		memcpy(&word, tx, chunk);
		writel(word, ctx->regs + DSI2H_CRI_TX_PLD);
		tx += chunk;
		len -= chunk;
	}

	word = (u32)pkt.header[0] | ((u32)pkt.header[1] << 8) |
	       ((u32)pkt.header[2] << 16);
	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		word |= DSI2H_CRI_TX_MODE_LP;
	if (msg->rx_len)
		word |= DSI2H_CRI_HDR_RD;
	if (mipi_dsi_packet_format_is_long(msg->type))
		word |= DSI2H_CRI_HDR_LONG;

	writel(word, ctx->regs + DSI2H_CRI_TX_HDR);

	if (msg->rx_len) {
		ret = dw_dsi2h_cri_read(ctx, msg);
	} else {
		ret = dw_dsi2h_wait_cri_idle(ctx);
		if (!ret)
			ret = msg->tx_len;
	}

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int dw_dsi2h_host_attach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dw_dsi2h *ctx = host_to_dsi2h(host);
	struct drm_bridge *next;

	if (device->lanes > 4)
		return -EINVAL;

	next = devm_drm_of_get_bridge(ctx->dev, ctx->dev->of_node, 1, 0);
	if (IS_ERR(next))
		return PTR_ERR(next);

	ctx->next_bridge = next;
	ctx->dsi_dev = device;
	ctx->lanes = device->lanes ? device->lanes : 4;
	ctx->dsc_enabled = (device->dsc != NULL);

	return devm_drm_bridge_add(ctx->dev, &ctx->bridge);
}

static int dw_dsi2h_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct dw_dsi2h *ctx = host_to_dsi2h(host);

	ctx->dsi_dev = NULL;
	return 0;
}

static const struct mipi_dsi_host_ops dw_dsi2h_host_ops = {
	.attach = dw_dsi2h_host_attach,
	.detach = dw_dsi2h_host_detach,
	.transfer = dw_dsi2h_host_transfer,
};

static u32 dw_dsi2h_fixed_ratio(u32 num, u32 den, u32 max_val)
{
	u64 q = ((u64)num << 16) + (den >> 1);

	return min_t(u64, div64_u64(q, den), max_val);
}

static void dw_dsi2h_program_regs(struct dw_dsi2h *ctx)
{
	u32 byte_clk_khz = (ctx->datarate_mbps * 1000) / (ctx->ppi_width == 1 ? 16 : 8);
	u32 lptx_div = clamp_t(u32, ctx->sys_clk_khz / ctx->lptx_clk_khz / 2, 1, 63);
	u32 color_cfg;

	writel(0, ctx->regs + DSI2H_PWR_UP);
	readl(ctx->regs + DSI2H_PWR_UP);
	udelay(1);
	writel(DSI2H_MODE_IDLE, ctx->regs + DSI2H_MODE_CTRL);
	writel(0, ctx->regs + DSI2H_CRI_TX_CTRL);

	writel(0x0, ctx->regs + DSI2H_TO_HSTX_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_HSTXRDY_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_LPRX_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_LPTXRDY_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_LPTXTRIG_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_LPTXULPS_CFG);
	writel(0xffff, ctx->regs + DSI2H_TO_BTA_CFG);
	writel(1, ctx->regs + DSI2H_MANUAL_MODE_CFG);

	writel(((ctx->lanes - 1) << DSI2H_PHY_LANES_SHIFT) |
	       (ctx->ppi_width << DSI2H_PHY_PPI_WIDTH_SHIFT) |
	       DSI2H_PHY_HS_TRANSFER_EN,
	       ctx->regs + DSI2H_PHY_MODE_CFG);
	writel(DSI2H_PHY_CLK_TYPE_NONCONT | (lptx_div << DSI2H_PHY_LPTX_CLK_DIV_SHIFT),
	       ctx->regs + DSI2H_PHY_CLK_CFG);
	writel(ctx->lp2hs_time, ctx->regs + DSI2H_PHY_LP2HS_CFG);
	writel(ctx->hs2lp_time, ctx->regs + DSI2H_PHY_HS2LP_CFG);
	writel(8 * byte_clk_khz / 1000, ctx->regs + DSI2H_PHY_MAX_RD_TIME_CFG);
	writel(8645076, ctx->regs + DSI2H_PHY_ESC_CMD_T_CFG);
	writel(4715496, ctx->regs + DSI2H_PHY_ESC_BYTE_T_CFG);
	writel(dw_dsi2h_fixed_ratio(byte_clk_khz, ctx->ipi_clk_khz, 0x3fffff),
	       ctx->regs + DSI2H_PHY_IPI_RATIO_CFG);
	writel(dw_dsi2h_fixed_ratio(byte_clk_khz, ctx->sys_clk_khz, 0x3ffff),
	       ctx->regs + DSI2H_PHY_SYS_RATIO_CFG);

	writel(DSI2H_GENERAL_EOTP_TX_EN | DSI2H_GENERAL_BTA_EN,
	       ctx->regs + DSI2H_DSI_GENERAL_CFG);
	writel(0, ctx->regs + DSI2H_DSI_VCID_CFG);
	writel(0, ctx->regs + DSI2H_DSI_SCRAMBLING_CFG);
	writel(0x2 | BIT(4) | BIT(5) | BIT(12) | BIT(13) | BIT(14),
	       ctx->regs + DSI2H_DSI_VID_TX_CFG);
	writel(1000, ctx->regs + DSI2H_DSI_MAX_RPS_CFG);
	writel(1, ctx->regs + DSI2H_DSI_TEAR_EFFECT_CFG);

	color_cfg = (ctx->dsc_enabled ? DSI2H_IPI_FMT_COMPRESSED : 0) | DSI2H_IPI_DEPTH_8BIT;
	writel(color_cfg, ctx->regs + DSI2H_IPI_COLOR_MAN_CFG);
	writel(ctx->mode.hdisplay ? ctx->mode.hdisplay : 1344,
	       ctx->regs + DSI2H_IPI_PIX_PKT_CFG);
	writel(0, ctx->regs + DSI2H_IPI_HIBERNATE_CFG);

	/* Mask expected/noisy interrupts so IRQ line stays clean */
	writel(0x3001f, ctx->regs + DSI2H_INT_MASK_PHY);
	writel(0x0, ctx->regs + DSI2H_INT_MASK_TO);
	writel(0xffff, ctx->regs + DSI2H_INT_MASK_ACK);
	writel(0xf010f, ctx->regs + DSI2H_INT_MASK_IPI);
	writel(0x3, ctx->regs + DSI2H_INT_MASK_PRI);
	writel(0x3f007f, ctx->regs + DSI2H_INT_MASK_CRI);
}

static int dw_dsi2h_bridge_attach(struct drm_bridge *bridge,
				  struct drm_encoder *encoder,
				  enum drm_bridge_attach_flags flags)
{
	struct dw_dsi2h *ctx = bridge_to_dsi2h(bridge);

	encoder->encoder_type = DRM_MODE_ENCODER_DSI;
	return drm_bridge_attach(encoder, ctx->next_bridge, bridge, flags);
}

static enum drm_mode_status
dw_dsi2h_bridge_mode_valid(struct drm_bridge *bridge,
			   const struct drm_display_info *info,
			   const struct drm_display_mode *mode)
{
	if (!mode->hdisplay || !mode->vdisplay)
		return MODE_BAD;
	return MODE_OK;
}

static void dw_dsi2h_bridge_mode_set(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode,
				     const struct drm_display_mode *adjusted_mode)
{
	struct dw_dsi2h *ctx = bridge_to_dsi2h(bridge);

	drm_mode_copy(&ctx->mode, adjusted_mode);
}

static void dw_dsi2h_bridge_atomic_pre_enable(struct drm_bridge *bridge,
					      struct drm_atomic_commit *state)
{
	struct dw_dsi2h *ctx = bridge_to_dsi2h(bridge);
	union phy_configure_opts opts = {};
	u32 phy_sts = 0;
	int ret;

	mutex_lock(&ctx->lock);
	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to resume DSI2H PD: %d\n", ret);
		mutex_unlock(&ctx->lock);
		return;
	}

	dev_info(ctx->dev, "DSI2H CORE_ID=%#x VERSION=%#x\n",
		 readl(ctx->regs + DSI2H_CORE_ID),
		 readl(ctx->regs + DSI2H_VERSION));

	dw_dsi2h_program_regs(ctx);

	opts.mipi_dphy.hs_clk_rate = (unsigned long)ctx->datarate_mbps * 1000000UL;
	opts.mipi_dphy.lanes = ctx->lanes;

	ret = phy_init(ctx->phy);
	if (ret) {
		dev_err(ctx->dev, "phy_init failed: %d\n", ret);
		goto err_pm;
	}
	ret = phy_configure(ctx->phy, &opts);
	if (ret) {
		dev_err(ctx->dev, "phy_configure failed: %d\n", ret);
		goto err_phy_exit;
	}

	writel(1, ctx->regs + DSI2H_PWR_UP);

	ret = phy_power_on(ctx->phy);
	if (ret) {
		dev_err(ctx->dev, "phy_power_on failed: %d\n", ret);
		goto err_phy_exit;
	}

	if (readl_poll_timeout(ctx->regs + DSI2H_PHY_STATUS, phy_sts,
			       (phy_sts & DSI2H_PHY_STOPSTATE_ALL) == DSI2H_PHY_STOPSTATE_ALL,
			       DSI2H_POLL_INTERVAL_US, 10000)) {
		dev_warn(ctx->dev, "DSI2H PHY stopstate incomplete (phy=%#x)\n", phy_sts);
		dw_dsi2h_dump_diag(ctx, "phy_stopstate_timeout");
	}

	dw_dsi2h_set_op_mode(ctx, DSI2H_MODE_DATA_STREAM);
	ctx->host_ready = true;
	mutex_unlock(&ctx->lock);
	return;

err_phy_exit:
	phy_exit(ctx->phy);
err_pm:
	pm_runtime_put_sync(ctx->dev);
	mutex_unlock(&ctx->lock);
}

static void dw_dsi2h_bridge_atomic_enable(struct drm_bridge *bridge,
					  struct drm_atomic_commit *state)
{
	struct dw_dsi2h *ctx = bridge_to_dsi2h(bridge);

	mutex_lock(&ctx->lock);
	if (ctx->host_ready &&
	    (readl(ctx->regs + DSI2H_MODE_STATUS) & 0x7) != DSI2H_MODE_DATA_STREAM)
		dw_dsi2h_set_op_mode(ctx, DSI2H_MODE_DATA_STREAM);
	mutex_unlock(&ctx->lock);
}

static void dw_dsi2h_bridge_atomic_post_disable(struct drm_bridge *bridge,
						struct drm_atomic_commit *state)
{
	struct dw_dsi2h *ctx = bridge_to_dsi2h(bridge);
	u32 sts;

	mutex_lock(&ctx->lock);
	ctx->host_ready = false;
	dw_dsi2h_set_op_mode(ctx, DSI2H_MODE_IDLE);
	if (readl_poll_timeout(ctx->regs + DSI2H_CORE_STATUS, sts, !sts,
			       DSI2H_POLL_INTERVAL_US, DSI2H_POLL_TIMEOUT_US))
		dw_dsi2h_dump_diag(ctx, "post_disable_idle_timeout");
	writel(0, ctx->regs + DSI2H_PWR_UP);
	phy_power_off(ctx->phy);
	phy_exit(ctx->phy);
	pm_runtime_put_sync(ctx->dev);
	mutex_unlock(&ctx->lock);
}

static const struct drm_bridge_funcs dw_dsi2h_bridge_funcs = {
	.attach = dw_dsi2h_bridge_attach,
	.mode_valid = dw_dsi2h_bridge_mode_valid,
	.mode_set = dw_dsi2h_bridge_mode_set,
	.atomic_pre_enable = dw_dsi2h_bridge_atomic_pre_enable,
	.atomic_enable = dw_dsi2h_bridge_atomic_enable,
	.atomic_post_disable = dw_dsi2h_bridge_atomic_post_disable,
	.atomic_create_state = drm_atomic_helper_bridge_create_state,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
};

static irqreturn_t dw_dsi2h_irq(int irq, void *data)
{
	struct dw_dsi2h *ctx = data;
	u32 st = readl(ctx->regs + DSI2H_INT_ST_MAIN);

	if (!st)
		return IRQ_NONE;

	dev_dbg_ratelimited(ctx->dev, "DSI2H INT_ST_MAIN=%#x\n", st);
	return IRQ_HANDLED;
}

static void dw_dsi2h_host_unregister_action(void *data)
{
	mipi_dsi_host_unregister(data);
}

static int dw_dsi2h_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_dsi2h *ctx;
	int irq, ret;

	ctx = devm_drm_bridge_alloc(dev, struct dw_dsi2h, bridge, &dw_dsi2h_bridge_funcs);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dev = dev;
	mutex_init(&ctx->lock);

	ctx->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);

	ctx->phy = devm_phy_get(dev, "dw-mipi-cdphy");
	if (IS_ERR(ctx->phy))
		return dev_err_probe(dev, PTR_ERR(ctx->phy), "failed to acquire CDPHY\n");

	of_property_read_u32(dev->of_node, "datarate", &ctx->datarate_mbps);
	if (!ctx->datarate_mbps)
		ctx->datarate_mbps = 1368;
	of_property_read_u32(dev->of_node, "sys_clk", &ctx->sys_clk_khz);
	if (!ctx->sys_clk_khz)
		ctx->sys_clk_khz = 400000;
	of_property_read_u32(dev->of_node, "ipi_clk", &ctx->ipi_clk_khz);
	if (!ctx->ipi_clk_khz)
		ctx->ipi_clk_khz = 75000;
	of_property_read_u32(dev->of_node, "lptx_clk", &ctx->lptx_clk_khz);
	if (!ctx->lptx_clk_khz)
		ctx->lptx_clk_khz = 20000;
	of_property_read_u32(dev->of_node, "lp2hs_time", &ctx->lp2hs_time);
	if (!ctx->lp2hs_time)
		ctx->lp2hs_time = 0x009b00d4;
	of_property_read_u32(dev->of_node, "hs2lp_time", &ctx->hs2lp_time);
	if (!ctx->hs2lp_time)
		ctx->hs2lp_time = 0x006a80cc;
	of_property_read_u32(dev->of_node, "ppi_width", &ctx->ppi_width);
	if (!ctx->ppi_width)
		ctx->ppi_width = 1;
	ctx->lanes = 4;

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		ret = devm_request_irq(dev, irq, dw_dsi2h_irq, 0, dev_name(dev), ctx);
		if (ret)
			return ret;
	}

	ctx->bridge.of_node = dev->of_node;
	ctx->dsi_host.dev = dev;
	ctx->dsi_host.ops = &dw_dsi2h_host_ops;

	pm_runtime_enable(dev);
	platform_set_drvdata(pdev, ctx);

	ret = mipi_dsi_host_register(&ctx->dsi_host);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, dw_dsi2h_host_unregister_action, &ctx->dsi_host);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	return 0;
}

static void dw_dsi2h_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id dw_dsi2h_of_match[] = {
	{ .compatible = "verisilicon,dc9x00-mipi-dsi2h" },
	{}
};
MODULE_DEVICE_TABLE(of, dw_dsi2h_of_match);

struct platform_driver dw_dsi2h_driver = {
	.probe = dw_dsi2h_probe,
	.remove = dw_dsi2h_remove,
	.driver = {
		.name = "dw-mipi-dsi2h",
		.of_match_table = dw_dsi2h_of_match,
	},
};
