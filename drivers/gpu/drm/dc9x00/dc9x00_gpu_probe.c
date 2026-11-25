// SPDX-License-Identifier: GPL-2.0-only
/*
 * Opt-in Stage G0 PowerVR Volcanic identity probe for Tensor G5 Laguna.
 * Kept powered OFF at boot; triggered only via debugfs or explicit module param.
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "dc9x00_internal.h"
#include "dc9x00_regs.h"

static bool gpu_probe_on_boot;
module_param(gpu_probe_on_boot, bool, 0444);
MODULE_PARM_DESC(gpu_probe_on_boot,
		 "Run Stage G0 PowerVR RGX_CR_CORE_ID probe during driver init (default: N)");

struct dc9x00_gpu_probe {
	struct device *dev;
	struct mutex lock;
	void __iomem *gpu_control;
	void __iomem *pvtc;
	void __iomem *pvtc_pwroff;
	void __iomem *dts;
	void __iomem *rgx;
	struct dentry *debug_dir;
	u64 last_core_id;
	int last_status;
	u32 last_ack;
};

static struct dc9x00_gpu_probe *g_gpu_probe;

static int gpu_pvtc_sdif_cmd(struct dc9x00_gpu_probe *ctx, u8 addr, u32 data)
{
	u32 status;
	u32 cmd = BIT(31) | BIT(27) | ((u32)(addr & 0x7) << 24) | (data & 0xffffff);

	writel(cmd, ctx->pvtc + GPU_PVTC_SDIF);
	return readl_poll_timeout(ctx->pvtc + GPU_PVTC_SDIF_STATUS, status,
				  (status & BIT(0)) == 0, 0, 1000);
}

static int gpu_set_pvtc_pwroff(struct dc9x00_gpu_probe *ctx, bool power_on)
{
	u32 ack, req = power_on ? 0 : 1;

	writel(req, ctx->pvtc_pwroff + GPU_PVTC_PWROFF_REQ);
	return readl_poll_timeout(ctx->pvtc_pwroff + GPU_PVTC_PWROFF_ACK, ack,
				  ack == req, 0, 1000);
}

static int gpu_core_logic_power_on(struct dc9x00_gpu_probe *ctx)
{
	u32 ack, pwr_status;
	int ret;

	writel(GPU_CTRL_POWER_ON_VAL, ctx->gpu_control);
	ret = readl_poll_timeout(ctx->gpu_control, ack, ack & GPU_CTRL_ACK_MASK,
				 0, 10000);
	if (ret)
		return ret;
	ctx->last_ack = ack;

	if (ctx->dts) {
		writel(0x1, ctx->dts + GPU_DTS_PWRSWITCH_CTRL);
		writel(0x0, ctx->dts + GPU_DTS_ISO_EN);
	}

	if ((ack & GPU_CTRL_POWER_OFF_ACK) && ctx->pvtc) {
		writel(BIT(24) | (1U << 16) | (1U << 8) | 1U,
		       ctx->pvtc + GPU_PVTC_CLK_SYNTH);
		writel(0x7E, ctx->pvtc + GPU_PVTC_SDIF_DISABLE);
		ret = readl_poll_timeout(ctx->pvtc + GPU_PVTC_PWR_STATUS, pwr_status,
					 pwr_status == 1, 10, 1000);
		if (ret)
			return ret;

		ret = gpu_pvtc_sdif_cmd(ctx, 0x1, 0xF0000);
		if (ret)
			return ret;
		ret = gpu_pvtc_sdif_cmd(ctx, 0x4, 0x7);
		if (ret)
			return ret;
		ret = gpu_pvtc_sdif_cmd(ctx, 0x5, 0x200);
		if (ret)
			return ret;
		ret = gpu_pvtc_sdif_cmd(ctx, 0x0, 0x508);
		if (ret)
			return ret;
	} else if ((ack & GPU_CTRL_POWER_GATE_ACK) && ctx->pvtc_pwroff) {
		ret = gpu_set_pvtc_pwroff(ctx, true);
		if (ret)
			return ret;
	}

	return 0;
}

static void gpu_core_logic_power_off(struct dc9x00_gpu_probe *ctx)
{
	u32 ack;

	if (ctx->pvtc_pwroff)
		gpu_set_pvtc_pwroff(ctx, false);
	if (ctx->dts)
		writel(0x1, ctx->dts + GPU_DTS_PWRSTATE);

	writel(GPU_CTRL_POWER_OFF_VAL, ctx->gpu_control);
	readl_poll_timeout(ctx->gpu_control, ack, ack & GPU_CTRL_ACK_MASK, 0, 10000);
}

static int dc9x00_gpu_probe_run(struct dc9x00_gpu_probe *ctx)
{
	int ret;

	mutex_lock(&ctx->lock);
	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret < 0) {
		ctx->last_status = ret;
		mutex_unlock(&ctx->lock);
		dev_err(ctx->dev, "GPU probe failed to power sswrp_gpu_pd: %d\n", ret);
		return ret;
	}

	ret = gpu_core_logic_power_on(ctx);
	if (ret) {
		ctx->last_status = ret;
		dev_err(ctx->dev, "GPU core logic power-on failed: %d (ack=%#x)\n",
			ret, ctx->last_ack);
		goto out_put;
	}

	ctx->last_core_id = readq(ctx->rgx + RGX_CR_CORE_ID_OFF);
	ctx->last_status = 0;
	dev_info(ctx->dev,
		 "Stage G0 PowerVR probe: RGX_CR_CORE_ID=%#018llx (ack=%#x)\n",
		 ctx->last_core_id, ctx->last_ack);

	gpu_core_logic_power_off(ctx);

out_put:
	pm_runtime_put_sync(ctx->dev);
	mutex_unlock(&ctx->lock);
	return ret;
}

static int gpu_probe_run_set(void *data, u64 val)
{
	struct dc9x00_gpu_probe *ctx = data;

	if (!val)
		return 0;
	return dc9x00_gpu_probe_run(ctx);
}

static int gpu_probe_run_get(void *data, u64 *val)
{
	struct dc9x00_gpu_probe *ctx = data;

	*val = (ctx->last_status == 0 && ctx->last_core_id != 0) ? 1 : 0;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gpu_probe_run_fops, gpu_probe_run_get,
			 gpu_probe_run_set, "%llu\n");

static int gpu_probe_id_get(void *data, u64 *val)
{
	struct dc9x00_gpu_probe *ctx = data;

	*val = ctx->last_core_id;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gpu_probe_id_fops, gpu_probe_id_get, NULL, "%#llx\n");

void dc9x00_gpu_probe_debugfs_init(struct dentry *root)
{
	struct dc9x00_gpu_probe *ctx = g_gpu_probe;

	if (!ctx || !root)
		return;

	ctx->debug_dir = root;
	debugfs_create_file("gpu_probe_run", 0600, root, ctx, &gpu_probe_run_fops);
	debugfs_create_file("gpu_probe_id", 0400, root, ctx, &gpu_probe_id_fops);
}

void dc9x00_gpu_probe_debugfs_cleanup(void)
{
	struct dc9x00_gpu_probe *ctx = g_gpu_probe;

	if (ctx)
		ctx->debug_dir = NULL;
}

static int dc9x00_gpu_probe_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dc9x00_gpu_probe *ctx;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->last_status = -EAGAIN;
	mutex_init(&ctx->lock);

	ctx->gpu_control = devm_platform_ioremap_resource_byname(pdev, "gpu_control");
	if (IS_ERR(ctx->gpu_control))
		return PTR_ERR(ctx->gpu_control);

	ctx->rgx = devm_platform_ioremap_resource_byname(pdev, "rgx");
	if (IS_ERR(ctx->rgx))
		return PTR_ERR(ctx->rgx);

	ctx->pvtc = devm_platform_ioremap_resource_byname(pdev, "pvtc");
	if (IS_ERR(ctx->pvtc))
		ctx->pvtc = NULL;

	ctx->pvtc_pwroff = devm_platform_ioremap_resource_byname(pdev, "pvtc_pwroff");
	if (IS_ERR(ctx->pvtc_pwroff))
		ctx->pvtc_pwroff = NULL;

	ctx->dts = devm_platform_ioremap_resource_byname(pdev, "dts");
	if (IS_ERR(ctx->dts))
		ctx->dts = NULL;

	platform_set_drvdata(pdev, ctx);
	pm_runtime_enable(dev);
	g_gpu_probe = ctx;

	if (gpu_probe_on_boot)
		dc9x00_gpu_probe_run(ctx);

	return 0;
}

static void dc9x00_gpu_probe_driver_remove(struct platform_device *pdev)
{
	if (g_gpu_probe == platform_get_drvdata(pdev))
		g_gpu_probe = NULL;
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id dc9x00_gpu_probe_of_match[] = {
	{ .compatible = "google,laguna-gpu-stage0-probe" },
	{}
};
MODULE_DEVICE_TABLE(of, dc9x00_gpu_probe_of_match);

struct platform_driver dc9x00_gpu_probe_driver = {
	.probe = dc9x00_gpu_probe_driver_probe,
	.remove = dc9x00_gpu_probe_driver_remove,
	.driver = {
		.name = "laguna-gpu-probe",
		.of_match_table = dc9x00_gpu_probe_of_match,
	},
};
