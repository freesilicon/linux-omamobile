// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room platform driver for Google Tensor G5 (Laguna / buenos) EdgeTPU.
 *
 * Probes "google,edgetpu-laguna", maps CSRs/mailboxes, binds arm,smmu-v3
 * and GSA security authentication, enables runtime PM, and registers
 * /dev/edgetpu-soc.
 */

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/google/gsa.h>

#include "edgetpu_laguna.h"

static void laguna_put_gsa(void *data)
{
	struct device *gsa = data;

	if (gsa)
		gsa_put_device(gsa);
}

static int laguna_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct edgetpu_laguna_dev *etdev;
	struct resource *res;
	u32 lpb_base = (u32)LAGUNA_LPB_TPU_DEFAULT_BASE;
	u32 lpb_size = LAGUNA_LPB_TPU_DEFAULT_SIZE;
	int ret;

	etdev = devm_kzalloc(dev, sizeof(*etdev), GFP_KERNEL);
	if (!etdev)
		return -ENOMEM;

	etdev->dev = dev;
	mutex_init(&etdev->pm_lock);
	platform_set_drvdata(pdev, etdev);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "top");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	etdev->top_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(etdev->top_regs))
		return PTR_ERR(etdev->top_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lpcm");
	if (res) {
		etdev->lpcm_regs = devm_ioremap_resource(dev, res);
		if (IS_ERR(etdev->lpcm_regs))
			return PTR_ERR(etdev->lpcm_regs);
	}

	of_property_read_u32_index(dev->of_node, "lpb-sswrp-base", 0, &lpb_base);
	of_property_read_u32_index(dev->of_node, "lpb-sswrp-size", 0, &lpb_size);
	if (lpb_base && lpb_size)
		etdev->lpb_regs = devm_ioremap(dev, lpb_base, lpb_size);

	etdev->mbox_irqs[0] = platform_get_irq_byname(pdev, "mbox_0");
	if (etdev->mbox_irqs[0] < 0)
		return dev_err_probe(dev, etdev->mbox_irqs[0],
				     "failed to get mbox_0 IRQ\n");

	etdev->mbox_irqs[1] = platform_get_irq_byname(pdev, "mbox_1");
	if (etdev->mbox_irqs[1] < 0)
		return dev_err_probe(dev, etdev->mbox_irqs[1],
				     "failed to get mbox_1 IRQ\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set 36-bit DMA mask\n");

	etdev->gsa_dev = gsa_get_device(dev);
	if (IS_ERR(etdev->gsa_dev)) {
		ret = PTR_ERR(etdev->gsa_dev);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "GSA device unavailable (%d); only non-secure FW possible\n",
			 ret);
		etdev->gsa_dev = NULL;
	} else {
		ret = devm_add_action_or_reset(dev, laguna_put_gsa, etdev->gsa_dev);
		if (ret)
			return ret;
	}

	ret = edgetpu_laguna_iommu_init(etdev);
	if (ret)
		return ret;

	ret = edgetpu_laguna_fw_init(etdev);
	if (ret)
		goto err_iommu;

	pm_runtime_set_suspended(dev);
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = edgetpu_laguna_fs_register(etdev);
	if (ret)
		goto err_pm;

	dev_info(dev, "Google Laguna EdgeTPU initialized (/dev/%s, mbox IRQs %d/%d)\n",
		 etdev->mdev.name, etdev->mbox_irqs[0], etdev->mbox_irqs[1]);
	return 0;

err_pm:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	edgetpu_laguna_fw_exit(etdev);
err_iommu:
	edgetpu_laguna_iommu_exit(etdev);
	return ret;
}

static void laguna_remove(struct platform_device *pdev)
{
	struct edgetpu_laguna_dev *etdev = platform_get_drvdata(pdev);

	edgetpu_laguna_fs_unregister(etdev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	edgetpu_laguna_fw_exit(etdev);
	edgetpu_laguna_iommu_exit(etdev);
}

static int laguna_runtime_suspend(struct device *dev)
{
	struct edgetpu_laguna_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_laguna_runtime_suspend(etdev);
}

static int laguna_runtime_resume(struct device *dev)
{
	struct edgetpu_laguna_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_laguna_runtime_resume(etdev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(laguna_pm_ops, laguna_runtime_suspend,
				 laguna_runtime_resume, NULL);

static const struct of_device_id laguna_of_match[] = {
	{ .compatible = "google,edgetpu-laguna" },
	{ }
};
MODULE_DEVICE_TABLE(of, laguna_of_match);

static struct platform_driver edgetpu_laguna_driver = {
	.probe = laguna_probe,
	.remove = laguna_remove,
	.driver = {
		.name = "edgetpu-laguna",
		.of_match_table = laguna_of_match,
		.pm = pm_ptr(&laguna_pm_ops),
	},
};
module_platform_driver(edgetpu_laguna_driver);

MODULE_DESCRIPTION("Clean-room Google Tensor G5 (Laguna) EdgeTPU NPU driver");
MODULE_LICENSE("GPL");
