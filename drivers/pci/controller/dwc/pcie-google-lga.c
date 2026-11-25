// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for Google Laguna (Tensor G5) SoCs.
 *
 * Clean-room DesignWare PCIe Root Complex glue managing CPM clocks/resets,
 * HSIO_S power domains, PHY SRAM initialization handshake, endpoint pwrctrl,
 * and LTSSM bring-up.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci-pwrctrl.h>
#include <linux/pci_regs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <soc/google/clk-cpm.h>

#include "pcie-designware.h"

#define LGA_PCIE_SII_PWR_CTRL		0x2c
#define LGA_PCIE_SII_L1_PWRGATE		BIT(11)

#define LGA_PCIE_TOP_CTRL		0x00
#define LGA_PCIE_TOP_PHY_RST_DONE	BIT(0)
#define LGA_PCIE_TOP_EXT_LD_DONE	BIT(23)

#define LGA_PCIE_TOP_STATUS		0x10
#define LGA_PCIE_TOP_PHY_INIT_DONE	BIT(2)

#define LGA_PCIE_TOP_LTSSM_CTRL		0x14
#define LGA_PCIE_TOP_LTSSM_EN		BIT(0)

#define LGA_PCIE_TOP_LINK_STS		0x18
#define LGA_PCIE_TOP_LINK_UP_MASK	GENMASK(7, 6)

#define LGA_PCIE_TOP_AXI_CTRL		0x24
#define LGA_PCIE_TOP_AXI_MASK		GENMASK(5, 0)
#define LGA_PCIE_TOP_AXI_VAL		0x9

#define LGA_PCIE_DBI_RX_Q_CTRL		0x748
#define LGA_PCIE_DBI_RX_Q_CREDITS	GENMASK(11, 0)
#define LGA_PCIE_DBI_LINK_TIMEOUT	0x8d4

#define LGA_PCIE_L1SS_MASK		0xff0400ffU
#define LGA_PCIE_L1SS_VAL		0x00513c00U

#define LGA_PCIE_PHY_INIT_POLL_US	100
#define LGA_PCIE_PHY_INIT_TIMEOUT_US	USEC_PER_SEC
#define LGA_PCIE_DEFAULT_PERST_US	20000

struct google_lga_pcie {
	struct dw_pcie pci;
	void __iomem *sii;
	void __iomem *top;
	struct clk *phy_fw_clk;
	struct clk *aux_clk;
	struct reset_control *rst_init;
	struct reset_control *rst_perst;
	struct reset_control *rst_app_hold;
	struct reset_control *rst_power_up;
	struct gpio_desc *perstn_gpio;
	u32 perst_delay_us;
	u32 posted_rx_q_credits;
	u32 link_timeout_ms;
	bool has_link_timeout;
	bool has_phy_sram;
	bool skip_link_eq;
	bool l1_pwrgate_disable;
	bool auto_gate_enabled;
	bool clocks_enabled;
	bool irqs_enabled;
	int cpl_timeout_irq;
	int link_down_irq;
	atomic_t cpl_timeout_cnt;
	atomic_t link_down_cnt;
};

#define to_google_lga_pcie(x) container_of((x), struct google_lga_pcie, pci)

static irqreturn_t google_lga_pcie_cpl_timeout_isr(int irq, void *data)
{
	struct google_lga_pcie *lga = data;
	int cnt = atomic_inc_return(&lga->cpl_timeout_cnt);

	dev_warn_ratelimited(lga->pci.dev, "PCIe completion timeout event (%d)\n", cnt);
	return IRQ_HANDLED;
}

static irqreturn_t google_lga_pcie_link_down_isr(int irq, void *data)
{
	struct google_lga_pcie *lga = data;
	int cnt = atomic_inc_return(&lga->link_down_cnt);

	dev_warn_ratelimited(lga->pci.dev, "PCIe link down event (%d)\n", cnt);
	return IRQ_HANDLED;
}

static void google_lga_pcie_set_auto_gate(struct google_lga_pcie *lga, bool enable)
{
	int ret;

	if (!IS_ENABLED(CONFIG_GOOGLE_CPM_CLK) || !lga->phy_fw_clk)
		return;
	if (lga->auto_gate_enabled == enable)
		return;

	ret = goog_cpm_clk_toggle_auto_gate(lga->phy_fw_clk, enable);
	if (ret) {
		dev_warn(lga->pci.dev, "CPM auto-gate(%d) failed: %d\n",
			 enable, ret);
		return;
	}

	lga->auto_gate_enabled = enable;
}

static void google_lga_pcie_disable_irqs(struct google_lga_pcie *lga)
{
	if (!lga->irqs_enabled)
		return;

	if (lga->cpl_timeout_irq > 0)
		disable_irq(lga->cpl_timeout_irq);
	if (lga->link_down_irq > 0)
		disable_irq(lga->link_down_irq);
	lga->irqs_enabled = false;
}

static void google_lga_pcie_assert_resets(struct google_lga_pcie *lga)
{
	reset_control_assert(lga->rst_app_hold);
	reset_control_assert(lga->rst_perst);
	reset_control_assert(lga->rst_power_up);
	reset_control_assert(lga->rst_init);
}

static void google_lga_pcie_disable_clocks(struct google_lga_pcie *lga)
{
	if (!lga->clocks_enabled)
		return;

	clk_disable_unprepare(lga->phy_fw_clk);
	clk_disable_unprepare(lga->aux_clk);
	lga->clocks_enabled = false;
}

static void google_lga_pcie_configure_dbi(struct google_lga_pcie *lga)
{
	struct dw_pcie *pci = &lga->pci;
	u16 l1ss;
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);

	if (lga->has_link_timeout)
		dw_pcie_writel_dbi(pci, LGA_PCIE_DBI_LINK_TIMEOUT,
				   lga->link_timeout_ms);

	l1ss = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_L1SS);
	if (l1ss) {
		val = dw_pcie_readl_dbi(pci, l1ss + PCI_L1SS_CTL1);
		val = (val & LGA_PCIE_L1SS_MASK) | LGA_PCIE_L1SS_VAL;
		dw_pcie_writel_dbi(pci, l1ss + PCI_L1SS_CTL1, val);
	}

	if (lga->skip_link_eq) {
		val = dw_pcie_readl_dbi(pci, GEN3_RELATED_OFF);
		val |= GEN3_RELATED_OFF_GEN3_EQ_DISABLE;
		dw_pcie_writel_dbi(pci, GEN3_RELATED_OFF, val);
	}

	if (lga->posted_rx_q_credits) {
		val = dw_pcie_readl_dbi(pci, LGA_PCIE_DBI_RX_Q_CTRL);
		val &= ~LGA_PCIE_DBI_RX_Q_CREDITS;
		val |= FIELD_PREP(LGA_PCIE_DBI_RX_Q_CREDITS,
				  lga->posted_rx_q_credits);
		dw_pcie_writel_dbi(pci, LGA_PCIE_DBI_RX_Q_CTRL, val);
	}

	dw_pcie_dbi_ro_wr_dis(pci);
}

static int google_lga_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	/* Hold PERST# active (low) while bringing up supplies, clocks, and PHY. */
	if (lga->perstn_gpio)
		gpiod_set_value_cansleep(lga->perstn_gpio, 1);

	ret = pci_pwrctrl_create_devices(dev);
	if (ret)
		return ret;

	if (!pp->skip_pwrctrl_off) {
		ret = pci_pwrctrl_power_on_devices(dev);
		if (ret)
			goto err_pwrctrl_destroy;
	}

	ret = clk_prepare_enable(lga->aux_clk);
	if (ret)
		goto err_pwrctrl_off;

	ret = clk_prepare_enable(lga->phy_fw_clk);
	if (ret) {
		clk_disable_unprepare(lga->aux_clk);
		goto err_pwrctrl_off;
	}
	lga->clocks_enabled = true;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		goto err_disable_clks;

	google_lga_pcie_assert_resets(lga);

	ret = reset_control_deassert(lga->rst_init);
	if (ret)
		goto err_assert_resets;

	ret = reset_control_deassert(lga->rst_power_up);
	if (ret)
		goto err_assert_resets;

	ret = reset_control_deassert(lga->rst_perst);
	if (ret)
		goto err_assert_resets;

	if (lga->has_phy_sram) {
		ret = readl_poll_timeout(lga->top + LGA_PCIE_TOP_STATUS, val,
					 val & LGA_PCIE_TOP_PHY_INIT_DONE,
					 LGA_PCIE_PHY_INIT_POLL_US,
					 LGA_PCIE_PHY_INIT_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "PHY SRAM init timeout (status=%#x)\n", val);
			goto err_assert_resets;
		}

		val = readl(lga->top + LGA_PCIE_TOP_CTRL);
		val |= LGA_PCIE_TOP_EXT_LD_DONE;
		writel(val, lga->top + LGA_PCIE_TOP_CTRL);
	}

	val = readl(lga->top + LGA_PCIE_TOP_CTRL);
	val |= LGA_PCIE_TOP_PHY_RST_DONE;
	writel(val, lga->top + LGA_PCIE_TOP_CTRL);

	ret = reset_control_deassert(lga->rst_app_hold);
	if (ret)
		goto err_assert_resets;

	val = readl(lga->top + LGA_PCIE_TOP_AXI_CTRL);
	val = (val & ~LGA_PCIE_TOP_AXI_MASK) | LGA_PCIE_TOP_AXI_VAL;
	writel(val, lga->top + LGA_PCIE_TOP_AXI_CTRL);

	if (lga->l1_pwrgate_disable) {
		val = readl(lga->sii + LGA_PCIE_SII_PWR_CTRL);
		val &= ~LGA_PCIE_SII_L1_PWRGATE;
		writel(val, lga->sii + LGA_PCIE_SII_PWR_CTRL);
	}

	google_lga_pcie_configure_dbi(lga);

	return 0;

err_assert_resets:
	google_lga_pcie_assert_resets(lga);
	pinctrl_pm_select_idle_state(dev);
err_disable_clks:
	google_lga_pcie_disable_clocks(lga);
err_pwrctrl_off:
	if (!pp->skip_pwrctrl_off)
		pci_pwrctrl_power_off_devices(dev);
err_pwrctrl_destroy:
	if (ret != -EPROBE_DEFER)
		pci_pwrctrl_destroy_devices(dev);
	return ret;
}

static void google_lga_pcie_host_post_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);

	google_lga_pcie_set_auto_gate(lga, true);

	if (!lga->irqs_enabled) {
		if (lga->cpl_timeout_irq > 0) {
			irq_set_irqchip_state(lga->cpl_timeout_irq,
					      IRQCHIP_STATE_PENDING, false);
			enable_irq(lga->cpl_timeout_irq);
		}
		if (lga->link_down_irq > 0) {
			irq_set_irqchip_state(lga->link_down_irq,
					      IRQCHIP_STATE_PENDING, false);
			enable_irq(lga->link_down_irq);
		}
		lga->irqs_enabled = true;
	}
}

static void google_lga_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	struct device *dev = pci->dev;

	google_lga_pcie_disable_irqs(lga);
	google_lga_pcie_set_auto_gate(lga, false);

	if (lga->perstn_gpio)
		gpiod_set_value_cansleep(lga->perstn_gpio, 1);

	pinctrl_pm_select_idle_state(dev);
	google_lga_pcie_assert_resets(lga);
	google_lga_pcie_disable_clocks(lga);

	if (!pp->skip_pwrctrl_off)
		pci_pwrctrl_power_off_devices(dev);
	pci_pwrctrl_destroy_devices(dev);
}

static bool google_lga_pcie_link_up(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	u32 top_sts = readl(lga->top + LGA_PCIE_TOP_LINK_STS);
	u32 dbg1;

	if ((top_sts & LGA_PCIE_TOP_LINK_UP_MASK) != LGA_PCIE_TOP_LINK_UP_MASK)
		return false;

	dbg1 = dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG1);
	return !(dbg1 & PCIE_PORT_DEBUG1_LINK_IN_TRAINING);
}

static int google_lga_pcie_start_link(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	u32 val;

	if (lga->perstn_gpio) {
		gpiod_set_value_cansleep(lga->perstn_gpio, 0);
		usleep_range(lga->perst_delay_us, lga->perst_delay_us + 2000);
	}

	val = readl(lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
	val |= LGA_PCIE_TOP_LTSSM_EN;
	writel(val, lga->top + LGA_PCIE_TOP_LTSSM_CTRL);

	return 0;
}

static void google_lga_pcie_stop_link(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	u32 val;

	val = readl(lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
	val &= ~LGA_PCIE_TOP_LTSSM_EN;
	writel(val, lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
}

static const struct dw_pcie_ops google_lga_pcie_ops = {
	.link_up = google_lga_pcie_link_up,
	.start_link = google_lga_pcie_start_link,
	.stop_link = google_lga_pcie_stop_link,
};

static const struct dw_pcie_host_ops google_lga_pcie_host_ops = {
	.init = google_lga_pcie_host_init,
	.post_init = google_lga_pcie_host_post_init,
	.deinit = google_lga_pcie_host_deinit,
};

static int google_lga_pcie_request_irqs(struct platform_device *pdev,
					struct google_lga_pcie *lga)
{
	struct device *dev = &pdev->dev;
	int ret;

	lga->cpl_timeout_irq = platform_get_irq_byname_optional(pdev, "cpl_timeout");
	if (lga->cpl_timeout_irq > 0) {
		ret = devm_request_irq(dev, lga->cpl_timeout_irq,
				       google_lga_pcie_cpl_timeout_isr,
				       IRQF_NO_AUTOEN, "lga-pcie-cpl-timeout", lga);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request cpl_timeout irq\n");
	}

	lga->link_down_irq = platform_get_irq_byname_optional(pdev, "link_down");
	if (lga->link_down_irq > 0) {
		ret = devm_request_irq(dev, lga->link_down_irq,
				       google_lga_pcie_link_down_isr,
				       IRQF_NO_AUTOEN, "lga-pcie-link-down", lga);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request link_down irq\n");
	}

	return 0;
}

static int google_lga_pcie_get_resources(struct platform_device *pdev,
					 struct google_lga_pcie *lga)
{
	struct device *dev = &pdev->dev;

	lga->sii = devm_platform_ioremap_resource_byname(pdev, "sii");
	if (IS_ERR(lga->sii))
		return PTR_ERR(lga->sii);

	lga->top = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(lga->top))
		return PTR_ERR(lga->top);

	lga->has_phy_sram = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							 "phy_sram") != NULL;

	lga->aux_clk = devm_clk_get(dev, "aux_clk");
	if (IS_ERR(lga->aux_clk))
		return dev_err_probe(dev, PTR_ERR(lga->aux_clk),
				     "failed to get aux_clk\n");

	lga->phy_fw_clk = devm_clk_get(dev, "phy_fw");
	if (IS_ERR(lga->phy_fw_clk))
		return dev_err_probe(dev, PTR_ERR(lga->phy_fw_clk),
				     "failed to get phy_fw clock\n");

	lga->rst_init = devm_reset_control_get_exclusive(dev, "init");
	if (IS_ERR(lga->rst_init))
		return dev_err_probe(dev, PTR_ERR(lga->rst_init),
				     "failed to get init reset\n");

	lga->rst_perst = devm_reset_control_get_exclusive(dev, "perst");
	if (IS_ERR(lga->rst_perst))
		return dev_err_probe(dev, PTR_ERR(lga->rst_perst),
				     "failed to get perst reset\n");

	lga->rst_app_hold = devm_reset_control_get_exclusive(dev, "app_hold_phy");
	if (IS_ERR(lga->rst_app_hold))
		return dev_err_probe(dev, PTR_ERR(lga->rst_app_hold),
				     "failed to get app_hold_phy reset\n");

	lga->rst_power_up = devm_reset_control_get_exclusive(dev, "power_up");
	if (IS_ERR(lga->rst_power_up))
		return dev_err_probe(dev, PTR_ERR(lga->rst_power_up),
				     "failed to get power_up reset\n");

	lga->perstn_gpio = devm_gpiod_get_optional(dev, "perstn", GPIOD_OUT_HIGH);
	if (IS_ERR(lga->perstn_gpio))
		return dev_err_probe(dev, PTR_ERR(lga->perstn_gpio),
				     "failed to get perstn gpio\n");

	return 0;
}

static void google_lga_pcie_parse_dt(struct device *dev,
				     struct google_lga_pcie *lga)
{
	lga->perst_delay_us = LGA_PCIE_DEFAULT_PERST_US;
	device_property_read_u32(dev, "google,perst-delay-us", &lga->perst_delay_us);
	device_property_read_u32(dev, "google,posted-rx-q-data-credits",
				 &lga->posted_rx_q_credits);
	lga->has_link_timeout = !device_property_read_u32(dev, "link-timeout-ms",
							  &lga->link_timeout_ms);
	lga->skip_link_eq = device_property_read_bool(dev, "google,skip-link-equalization");
	lga->l1_pwrgate_disable = device_property_read_bool(dev, "google,l1-pwrgate-disable");
}

static int google_lga_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct google_lga_pcie *lga;
	int ret;

	lga = devm_kzalloc(dev, sizeof(*lga), GFP_KERNEL);
	if (!lga)
		return -ENOMEM;

	lga->pci.dev = dev;
	lga->pci.ops = &google_lga_pcie_ops;
	lga->pci.pp.ops = &google_lga_pcie_host_ops;
	platform_set_drvdata(pdev, lga);

	ret = google_lga_pcie_get_resources(pdev, lga);
	if (ret)
		return ret;

	google_lga_pcie_parse_dt(dev, lga);

	ret = google_lga_pcie_request_irqs(pdev, lga);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "failed to runtime-resume PCIe RC\n");
	}

	ret = dw_pcie_host_init(&lga->pci.pp);
	if (ret) {
		pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	return 0;
}

static void google_lga_pcie_remove(struct platform_device *pdev)
{
	struct google_lga_pcie *lga = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dw_pcie_host_deinit(&lga->pci.pp);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

static const struct of_device_id google_lga_pcie_of_match[] = {
	{ .compatible = "google,lga-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, google_lga_pcie_of_match);

static struct platform_driver google_lga_pcie_driver = {
	.probe = google_lga_pcie_probe,
	.remove = google_lga_pcie_remove,
	.driver = {
		.name = "google-lga-pcie",
		.of_match_table = google_lga_pcie_of_match,
	},
};
module_platform_driver(google_lga_pcie_driver);

MODULE_AUTHOR("Bobby Bones <saariuscrypto@gmail.com>");
MODULE_DESCRIPTION("Google Laguna (Tensor G5) PCIe host controller driver");
MODULE_LICENSE("GPL");
