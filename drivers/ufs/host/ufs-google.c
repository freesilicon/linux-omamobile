// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS host controller driver for Google Pixel Laguna (Tensor G5 / lga).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <ufs/unipro.h>
#include "ufshcd-pltfrm.h"

/* UFS top wrapper registers */
#define UFS_TOP_ENC_STAT			0x0008
#define UFS_TOP_CRYPTO_KAT_STAT			BIT(4)
#define UFS_TOP_PHY_CTRL			0x0100
#define UFS_TOP_PHY_PATCH_MODE_MASK		GENMASK(19, 16)
#define UFS_TOP_PHY_PATCH_MODE_ROM		0xf
#define UFS_TOP_PHY_SRAM_INIT_DONE		BIT(21)
#define UFS_TOP_REXT_CTRL			0x0110
#define UFS_TOP_REXT_EN				BIT(6)
#define UFS_TOP_AWUSER_VC			0x0200
#define UFS_TOP_PHYCLK_SEL			0x0208
#define UFS_TOP_CFG_CLKSEL			0x0210
#define UFS_TOP_MPHY_CFG			0x0214

/* Laguna HCI vendor registers */
#define UFS_HC_VENDOR_CTRL			0x00c0
#define UFS_HC_VENDOR_CTRL_CLR_MASK		GENMASK(17, 16)
#define UFS_HC_HCLK_DIV				0x00f8
#define UFS_HC_HCLK_DIV_200MHZ			0x00c8

/* Laguna MCQ queue and runtime offsets */
#define MCQ_QUEUE_OFFSET(c)			(FIELD_GET(GENMASK(23, 16), (c)) * 0x200)
#define UFS_GOOGLE_MCQ_SIZE			0x1000
#define UFS_GOOGLE_MCQ_OPR_SIZE			0x0200
#define UFS_GOOGLE_MCQ_OPR_SQD_OFF		0x4000
#define UFS_GOOGLE_MCQ_OPR_SQIS_OFF		0x4200
#define UFS_GOOGLE_MCQ_OPR_CQD_OFF		0x4400
#define UFS_GOOGLE_MCQ_OPR_CQIS_OFF		0x4600
#define UFS_GOOGLE_MCQ_SQD_STRIDE		0x0014
#define UFS_GOOGLE_MCQ_SQIS_STRIDE		0x0008
#define UFS_GOOGLE_MCQ_CQD_STRIDE		0x0008
#define UFS_GOOGLE_MCQ_CQIS_STRIDE		0x000c
#define UFS_GOOGLE_MCQ_QCFG_STRIDE		0x0040
#define UFS_GOOGLE_MCQ_ROUTE_REG		0x0034
#define UFS_GOOGLE_MCQ_ROUTE_EN			BIT(8)
#define UFS_GOOGLE_MCQIACR_CFG			0x81010100
#define UFS_GOOGLE_MAX_MCQ_IRQS			16

/* M-PHY indirect APB attributes over DME */
#define MPHY_APB_ADDR_LSB			0x810e0000
#define MPHY_APB_ADDR_MSB			0x811e0000
#define MPHY_APB_WDATA_LSB			0x812e0000
#define MPHY_APB_WDATA_MSB			0x813e0000
#define MPHY_APB_RDATA_LSB			0x814e0000
#define MPHY_APB_RDATA_MSB			0x815e0000
#define MPHY_APB_EXEC				0x816e0000

#define UFS_GOOGLE_DME_GEAR_TUNE		0x15d4
#define UFS_GOOGLE_UIC_POLL_TIMEOUT_US		4000000
#define UFS_GOOGLE_TOP_TIMEOUT_US		400000
#define UFS_GOOGLE_SRAM_TIMEOUT_US		570000
#define UFS_GOOGLE_PHY_READY_TIMEOUT_US		1000000

struct ufs_google_dme_attr {
	u32 sel;
	u32 val;
	u8 peer;
};

struct ufs_google_mcq_irq {
	struct ufs_hba *hba;
	unsigned int irq;
	u32 qid;
	bool requested;
};

struct ufs_google_host {
	struct ufs_hba *hba;
	void __iomem *ufs_top;
	phys_addr_t hc_phys;

	struct clk *ref_clk;
	struct clk *aux_clk;
	struct reset_control *phy_rst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *refclk_on;
	struct pinctrl_state *refclk_off;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct regulator *vdd0p75;
	struct regulator *vdd1p2;

	struct device *ufs_prep_pd_dev;
	struct device *ufs_pd_dev;
	struct notifier_block pd_nb;
	/* Serializes genpd/power-up transitions across init, exit, and notifiers */
	struct mutex pd_lock;
	/* Serializes indirect M-PHY register accesses and IE masking */
	struct mutex mphy_lock;
	bool pd_on;
	bool need_phy_init;

	bool enable_cal;
	u32 *phy_cal_data;
	int phy_cal_cnt;

	bool mcq_mapped;
	void __iomem *mcq_opr_base[OPR_MAX];
	struct ufs_google_mcq_irq mcq_irqs[UFS_GOOGLE_MAX_MCQ_IRQS];
	u32 nr_mcq_irqs;
};

static const struct ufs_google_dme_attr ufs_google_rmmi_attrs[] = {
	{ 0x81010000, 0x00000005, DME_LOCAL },
	{ 0x81150000, 0x00000001, DME_LOCAL },
	{ 0x80190004, 0x00000015, DME_LOCAL },
	{ 0x80190005, 0x00000015, DME_LOCAL },
	{ 0x80130004, 0x00000001, DME_LOCAL },
	{ 0x80130005, 0x00000001, DME_LOCAL },
};

static const struct ufs_google_dme_attr ufs_google_link_dme_seq[] = {
	{ UIC_ARG_MIB(T_CONNECTIONSTATE), 0x0, DME_LOCAL },
	{ UIC_ARG_MIB(N_DEVICEID), 0x0, DME_LOCAL },
	{ UIC_ARG_MIB(N_DEVICEID_VALID), 0x1, DME_LOCAL },
	{ UIC_ARG_MIB(T_PEERDEVICEID), 0x1, DME_LOCAL },
	{ UIC_ARG_MIB(T_PEERCPORTID), 0x0, DME_LOCAL },
	{ UIC_ARG_MIB(T_TRAFFICCLASS), 0x0, DME_LOCAL },
	{ UIC_ARG_MIB(T_CPORTFLAGS), 0x6, DME_LOCAL },
	{ UIC_ARG_MIB(T_CPORTMODE), 0x1, DME_LOCAL },
	{ UIC_ARG_MIB(T_CONNECTIONSTATE), 0x1, DME_LOCAL },
	{ UIC_ARG_MIB(T_CONNECTIONSTATE), 0x0, DME_PEER },
	{ UIC_ARG_MIB(N_DEVICEID), 0x1, DME_PEER },
	{ UIC_ARG_MIB(N_DEVICEID_VALID), 0x1, DME_PEER },
	{ UIC_ARG_MIB(T_PEERDEVICEID), 0x0, DME_PEER },
	{ UIC_ARG_MIB(T_PEERCPORTID), 0x0, DME_PEER },
	{ UIC_ARG_MIB(T_TRAFFICCLASS), 0x0, DME_PEER },
	{ UIC_ARG_MIB(T_CPORTFLAGS), 0x6, DME_PEER },
	{ UIC_ARG_MIB(T_CPORTMODE), 0x1, DME_PEER },
	{ UIC_ARG_MIB(T_CONNECTIONSTATE), 0x1, DME_PEER },
};

static const struct {
	u16 addr;
	u16 or_mask;
} ufs_google_cal_rmw[] = {
	{ 0x3128, 0x000c },
	{ 0x3328, 0x000c },
	{ 0x3101, 0x0002 },
	{ 0x3301, 0x0002 },
	{ 0x3103, 0x0060 },
	{ 0x3303, 0x0060 },
};

static inline void ufs_google_rmwl_top(struct ufs_google_host *ufs,
				       u32 mask, u32 val, u32 reg)
{
	u32 tmp = readl(ufs->ufs_top + reg);

	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, ufs->ufs_top + reg);
}

static int ufs_google_dme_set_seq(struct ufs_hba *hba,
				  const struct ufs_google_dme_attr *seq,
				  size_t count)
{
	size_t i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = ufshcd_dme_set_attr(hba, seq[i].sel, ATTR_SET_NOR,
					  seq[i].val, seq[i].peer);
		if (ret) {
			dev_err(hba->dev, "dme_set sel=0x%x val=0x%x failed (%d)\n",
				seq[i].sel, seq[i].val, ret);
			return ret;
		}
	}

	return 0;
}

static int ufs_google_poll_uic_cmd(struct ufs_hba *hba, u32 cmd, u32 sel,
				   u32 wval, u32 *rval)
{
	u32 status;
	int ret;

	mutex_lock(&hba->uic_cmd_mutex);
	ufshcd_writel(hba, sel, REG_UIC_COMMAND_ARG_1);
	ufshcd_writel(hba, 0, REG_UIC_COMMAND_ARG_2);
	ufshcd_writel(hba, wval, REG_UIC_COMMAND_ARG_3);
	ufshcd_writel(hba, cmd, REG_UIC_COMMAND);

	ret = readl_poll_timeout(hba->mmio_base + REG_INTERRUPT_STATUS, status,
				 status & UIC_COMMAND_COMPL, 3,
				 UFS_GOOGLE_UIC_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(hba->dev, "polled UIC cmd 0x%x sel 0x%x timed out\n",
			cmd, sel);
		goto out_unlock;
	}

	ufshcd_writel(hba, UIC_COMMAND_COMPL, REG_INTERRUPT_STATUS);
	ret = ufshcd_readl(hba, REG_UIC_COMMAND_ARG_2) & CONFIG_RESULT_CODE_MASK;
	if (ret) {
		dev_err(hba->dev, "polled UIC cmd 0x%x sel 0x%x err %d\n",
			cmd, sel, ret);
		ret = -EIO;
		goto out_unlock;
	}

	if (rval)
		*rval = ufshcd_readl(hba, REG_UIC_COMMAND_ARG_3);

out_unlock:
	mutex_unlock(&hba->uic_cmd_mutex);
	return ret;
}

static int ufs_google_mphy_indirect_xfer(struct ufs_hba *hba, u16 addr,
					 u16 *val, bool is_write)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	u32 ie_saved, lsb = 0, msb = 0;
	int ret;

	mutex_lock(&ufs->mphy_lock);

	/* Mask UIC_COMMAND_COMPL around polled calibration bursts to avoid IRQ races. */
	ie_saved = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);
	if (ie_saved & UIC_COMMAND_COMPL)
		ufshcd_writel(hba, ie_saved & ~UIC_COMMAND_COMPL, REG_INTERRUPT_ENABLE);

	ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET, MPHY_APB_ADDR_LSB,
				      addr & 0xff, NULL);
	if (ret)
		goto out_restore;

	ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET, MPHY_APB_ADDR_MSB,
				      (addr >> 8) & 0xff, NULL);
	if (ret)
		goto out_restore;

	if (is_write) {
		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET,
					      MPHY_APB_WDATA_LSB, *val & 0xff, NULL);
		if (ret)
			goto out_restore;

		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET,
					      MPHY_APB_WDATA_MSB, (*val >> 8) & 0xff,
					      NULL);
		if (ret)
			goto out_restore;

		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET, MPHY_APB_EXEC,
					      1, NULL);
	} else {
		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_SET, MPHY_APB_EXEC,
					      0, NULL);
		if (ret)
			goto out_restore;

		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_GET,
					      MPHY_APB_RDATA_LSB, 0, &lsb);
		if (ret)
			goto out_restore;

		ret = ufs_google_poll_uic_cmd(hba, UIC_CMD_DME_GET,
					      MPHY_APB_RDATA_MSB, 0, &msb);
		if (!ret)
			*val = (lsb & 0xff) | ((msb & 0xff) << 8);
	}

out_restore:
	if (ie_saved & UIC_COMMAND_COMPL)
		ufshcd_writel(hba, ufshcd_readl(hba, REG_INTERRUPT_ENABLE) |
			      UIC_COMMAND_COMPL, REG_INTERRUPT_ENABLE);
	mutex_unlock(&ufs->mphy_lock);

	return ret;
}

static int ufs_google_apply_calibration(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	u16 addr, val;
	size_t i;
	int ret;

	for (i = 0; i < (size_t)ufs->phy_cal_cnt; i++) {
		addr = ufs->phy_cal_data[i] & 0xffff;
		val = (ufs->phy_cal_data[i] >> 16) & 0xffff;
		ret = ufs_google_mphy_indirect_xfer(hba, addr, &val, true);
		if (ret)
			return ret;
	}

	val = 0x0002;
	ret = ufs_google_mphy_indirect_xfer(hba, 0x017c, &val, true);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ufs_google_cal_rmw); i++) {
		ret = ufs_google_mphy_indirect_xfer(hba,
						    ufs_google_cal_rmw[i].addr,
						    &val, false);
		if (ret)
			return ret;

		val |= ufs_google_cal_rmw[i].or_mask;
		ret = ufs_google_mphy_indirect_xfer(hba,
						    ufs_google_cal_rmw[i].addr,
						    &val, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int ufs_google_power_up(struct ufs_google_host *ufs, bool reset_hw)
{
	struct ufs_hba *hba = ufs->hba;
	u32 val;
	int ret;

	mutex_lock(&ufs->pd_lock);
	if (ufs->pd_on) {
		mutex_unlock(&ufs->pd_lock);
		return 0;
	}

	if (ufs->vdd1p2) {
		ret = regulator_enable(ufs->vdd1p2);
		if (ret)
			goto err_unlock;
	}

	if (ufs->vdd0p75) {
		ret = regulator_enable(ufs->vdd0p75);
		if (ret)
			goto err_dis_vdd1p2;
	}

	if (reset_hw) {
		ufs->need_phy_init = true;

		ufs_google_rmwl_top(ufs, UFS_TOP_PHY_PATCH_MODE_MASK,
				    FIELD_PREP(UFS_TOP_PHY_PATCH_MODE_MASK,
					       UFS_TOP_PHY_PATCH_MODE_ROM),
				    UFS_TOP_PHY_CTRL);
		ufs_google_rmwl_top(ufs, BIT(1), BIT(1), UFS_TOP_PHYCLK_SEL);
		ufs_google_rmwl_top(ufs, BIT(0), BIT(0), UFS_TOP_MPHY_CFG);
		ufs_google_rmwl_top(ufs, BIT(1), BIT(1), UFS_TOP_CFG_CLKSEL);
		ufs_google_rmwl_top(ufs, UFS_TOP_REXT_EN, 0, UFS_TOP_REXT_CTRL);

		ret = readl_poll_timeout(ufs->ufs_top + UFS_TOP_ENC_STAT, val,
					 val & UFS_TOP_CRYPTO_KAT_STAT, 10,
					 UFS_GOOGLE_TOP_TIMEOUT_US);
		if (ret) {
			dev_err(hba->dev, "ufs_top KAT timeout (0x%x)\n", val);
			goto err_dis_vdd0p75;
		}
	}

	if (ufs->refclk_on) {
		ret = pinctrl_select_state(ufs->pinctrl, ufs->refclk_on);
		if (ret)
			goto err_dis_vdd0p75;
	}

	if (reset_hw) {
		if (ufs->reset_gpio)
			gpiod_set_value_cansleep(ufs->reset_gpio, 0);
		if (ufs->power_gpio)
			gpiod_set_value_cansleep(ufs->power_gpio, 1);

		usleep_range(1500, 2000);
	}

	ret = clk_prepare_enable(ufs->ref_clk);
	if (ret)
		goto err_refclk_off;

	if (ufs->aux_clk) {
		ret = clk_prepare_enable(ufs->aux_clk);
		if (ret)
			goto err_dis_refclk;
	}

	usleep_range(300, 310);

	if (reset_hw) {
		if (ufs->reset_gpio)
			gpiod_set_value_cansleep(ufs->reset_gpio, 1);

		ufshcd_writel(hba,
			      ufshcd_readl(hba, REG_INTERRUPT_STATUS) & 0xe0000000U,
			      REG_INTERRUPT_STATUS);
	}

	ufs->pd_on = true;
	mutex_unlock(&ufs->pd_lock);

	return 0;

err_dis_refclk:
	clk_disable_unprepare(ufs->ref_clk);
err_refclk_off:
	if (ufs->refclk_off)
		pinctrl_select_state(ufs->pinctrl, ufs->refclk_off);
err_dis_vdd0p75:
	if (ufs->vdd0p75)
		regulator_disable(ufs->vdd0p75);
err_dis_vdd1p2:
	if (ufs->vdd1p2)
		regulator_disable(ufs->vdd1p2);
err_unlock:
	mutex_unlock(&ufs->pd_lock);
	return ret;
}

static void ufs_google_pd_off(struct ufs_google_host *ufs)
{
	mutex_lock(&ufs->pd_lock);
	if (!ufs->pd_on) {
		mutex_unlock(&ufs->pd_lock);
		return;
	}

	if (ufs->reset_gpio)
		gpiod_set_value_cansleep(ufs->reset_gpio, 0);

	if (ufs->aux_clk)
		clk_disable_unprepare(ufs->aux_clk);
	clk_disable_unprepare(ufs->ref_clk);

	if (ufs->refclk_off)
		pinctrl_select_state(ufs->pinctrl, ufs->refclk_off);

	if (ufs->power_gpio)
		gpiod_set_value_cansleep(ufs->power_gpio, 0);

	if (ufs->vdd0p75)
		regulator_disable(ufs->vdd0p75);
	if (ufs->vdd1p2)
		regulator_disable(ufs->vdd1p2);

	ufs->pd_on = false;
	mutex_unlock(&ufs->pd_lock);
}

static int ufs_google_pd_notifier(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct ufs_google_host *ufs = container_of(nb, struct ufs_google_host,
						   pd_nb);

	if (action == GENPD_NOTIFY_PRE_ON)
		ufs->need_phy_init = true;
	else if (action == GENPD_NOTIFY_ON)
		ufs_google_power_up(ufs, true);
	else if (action == GENPD_NOTIFY_OFF)
		ufs_google_pd_off(ufs);

	return NOTIFY_OK;
}

static int ufs_google_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct ufs_google_host *ufs;
	struct resource *hc_res;
	int ret, cnt;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	ufs->hba = hba;
	mutex_init(&ufs->pd_lock);
	mutex_init(&ufs->mphy_lock);
	ufshcd_set_variant(hba, ufs);

	if (device_property_read_bool(dev, "google,enable-phy-patching")) {
		dev_err(dev, "proprietary SRAM phy patching is unsupported; use ROM mode\n");
		return -EOPNOTSUPP;
	}

	hc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ufs_hc");
	if (!hc_res)
		hc_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!hc_res)
		return -ENODEV;
	ufs->hc_phys = hc_res->start;

	ufs->ufs_top = devm_platform_ioremap_resource_byname(pdev, "ufs_top");
	if (IS_ERR(ufs->ufs_top))
		return PTR_ERR(ufs->ufs_top);

	ufs->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(ufs->ref_clk))
		return dev_err_probe(dev, PTR_ERR(ufs->ref_clk),
				     "failed to get ref_clk\n");

	ufs->aux_clk = devm_clk_get_optional(dev, "aux_clk");
	if (IS_ERR(ufs->aux_clk))
		return dev_err_probe(dev, PTR_ERR(ufs->aux_clk),
				     "failed to get aux_clk\n");

	ufs->phy_rst = devm_reset_control_get(dev, "phy_rst");
	if (IS_ERR(ufs->phy_rst))
		return dev_err_probe(dev, PTR_ERR(ufs->phy_rst),
				     "failed to get phy_rst\n");

	ufs->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ufs->pinctrl))
		return dev_err_probe(dev, PTR_ERR(ufs->pinctrl),
				     "failed to get pinctrl\n");

	ufs->refclk_on = pinctrl_lookup_state(ufs->pinctrl, "refclk_on");
	if (IS_ERR(ufs->refclk_on))
		ufs->refclk_on = NULL;

	ufs->refclk_off = pinctrl_lookup_state(ufs->pinctrl, "refclk_off");
	if (IS_ERR(ufs->refclk_off))
		ufs->refclk_off = NULL;

	ufs->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ufs->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ufs->reset_gpio),
				     "failed to get reset gpio\n");

	ufs->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(ufs->power_gpio))
		return dev_err_probe(dev, PTR_ERR(ufs->power_gpio),
				     "failed to get power gpio\n");

	ufs->vdd0p75 = devm_regulator_get_optional(dev, "vdd0p75");
	if (IS_ERR(ufs->vdd0p75)) {
		if (PTR_ERR(ufs->vdd0p75) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ufs->vdd0p75),
					     "failed to get vdd0p75 regulator\n");
		ufs->vdd0p75 = NULL;
	}

	ufs->vdd1p2 = devm_regulator_get_optional(dev, "vdd1p2");
	if (IS_ERR(ufs->vdd1p2)) {
		if (PTR_ERR(ufs->vdd1p2) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ufs->vdd1p2),
					     "failed to get vdd1p2 regulator\n");
		ufs->vdd1p2 = NULL;
	}

	ufs->enable_cal = device_property_read_bool(dev, "google,enable-phy-calibration");
	if (ufs->enable_cal) {
		cnt = device_property_count_u32(dev, "phy-cal-data");
		if (cnt > 0) {
			ufs->phy_cal_data = devm_kcalloc(dev, cnt, sizeof(u32),
							 GFP_KERNEL);
			if (!ufs->phy_cal_data)
				return -ENOMEM;

			ret = device_property_read_u32_array(dev, "phy-cal-data",
							     ufs->phy_cal_data,
							     cnt);
			if (ret)
				return ret;
			ufs->phy_cal_cnt = cnt;
		}
	}

	ufs->ufs_prep_pd_dev = dev_pm_domain_attach_by_name(dev, "ufs_prep_pd");
	if (IS_ERR(ufs->ufs_prep_pd_dev)) {
		if (PTR_ERR(ufs->ufs_prep_pd_dev) != -ENOENT)
			return dev_err_probe(dev, PTR_ERR(ufs->ufs_prep_pd_dev),
					     "failed to attach ufs_prep_pd\n");
		ufs->ufs_prep_pd_dev = NULL;
	}

	ufs->ufs_pd_dev = dev_pm_domain_attach_by_name(dev, "ufs_pd");
	if (IS_ERR(ufs->ufs_pd_dev)) {
		ret = PTR_ERR(ufs->ufs_pd_dev);
		ufs->ufs_pd_dev = NULL;
		if (ret != -ENOENT)
			goto err_detach_prep_pd;
	}

	if (ufs->ufs_prep_pd_dev) {
		ret = pm_runtime_resume_and_get(ufs->ufs_prep_pd_dev);
		if (ret)
			goto err_detach_prep_pd;
	}

	if (ufs->ufs_pd_dev) {
		ufs->pd_nb.notifier_call = ufs_google_pd_notifier;
		ret = dev_pm_genpd_add_notifier(ufs->ufs_pd_dev, &ufs->pd_nb);
		if (ret)
			goto err_put_prep_pd;

		ret = pm_runtime_resume_and_get(ufs->ufs_pd_dev);
		if (ret)
			goto err_rm_notifier;
	}

	if (!ufs->pd_on) {
		u32 enc_stat = readl(ufs->ufs_top + UFS_TOP_ENC_STAT);
		u32 pwr_ctrl = readl(ufs->ufs_top + UFS_TOP_PHY_CTRL);
		u32 rext_ctrl = readl(ufs->ufs_top + UFS_TOP_REXT_CTRL);

		/*
		 * ABL leaves hsio_s_ufs_prep_pd and hsio_s_ufs_pd on-at-init
		 * with the UFS PHY SRAM FW and REXT calibration active, so
		 * pm_runtime_resume_and_get() does not fire GENPD_NOTIFY_ON.
		 * Preserve that bootloader state at cold boot: REXT_CTRL[6]
		 * is REXT_EN (1 internal, 0 external), not an isolation bit,
		 * and forcing a reset without the proprietary SRAM FW patch
		 * destroys ABL's calibrated PHY state.
		 */
		dev_info(dev, "on-at-init PHY (enc=0x%08x pwr=0x%08x rext=0x%08x)\n",
			 enc_stat, pwr_ctrl, rext_ctrl);
		ufs->need_phy_init = false;

		ret = ufs_google_power_up(ufs, false);
		if (ret)
			goto err_put_pd;
	}

	hba->quirks |= UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8 |
		       UFSHCD_QUIRK_SET_IID_TO_ONE;

	return 0;

err_put_pd:
	if (ufs->ufs_pd_dev)
		pm_runtime_put_sync(ufs->ufs_pd_dev);
err_rm_notifier:
	if (ufs->ufs_pd_dev) {
		dev_pm_genpd_remove_notifier(ufs->ufs_pd_dev);
		dev_pm_domain_detach(ufs->ufs_pd_dev, true);
	}
err_put_prep_pd:
	if (ufs->ufs_prep_pd_dev)
		pm_runtime_put_sync(ufs->ufs_prep_pd_dev);
err_detach_prep_pd:
	if (ufs->ufs_prep_pd_dev)
		dev_pm_domain_detach(ufs->ufs_prep_pd_dev, true);
	return ret;
}

static void ufs_google_exit(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);

	ufs_google_pd_off(ufs);

	if (ufs->ufs_pd_dev) {
		pm_runtime_put_sync(ufs->ufs_pd_dev);
		dev_pm_genpd_remove_notifier(ufs->ufs_pd_dev);
		dev_pm_domain_detach(ufs->ufs_pd_dev, true);
	}

	if (ufs->ufs_prep_pd_dev) {
		pm_runtime_put_sync(ufs->ufs_prep_pd_dev);
		dev_pm_domain_detach(ufs->ufs_prep_pd_dev, true);
	}
}

static int ufs_google_phy_initialization(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	u32 val;
	int ret;

	ret = ufshcd_dme_set(hba, UIC_ARG_MIB(0x8118), 0);
	if (ret)
		return ret;

	ret = ufshcd_dme_set(hba, UIC_ARG_MIB(0xd085), 1);
	if (ret)
		return ret;

	if (!ufs->need_phy_init) {
		dev_info(hba->dev, "phy init skipped\n");
		return 0;
	}

	if (device_property_read_bool(hba->dev, "google,skip-ufs-phy-init"))
		return ufshcd_dme_set(hba, UIC_ARG_MIB(0xd0c1), 0);

	ret = ufs_google_dme_set_seq(hba, ufs_google_rmmi_attrs,
				     ARRAY_SIZE(ufs_google_rmmi_attrs));
	if (ret)
		return ret;

	ret = reset_control_deassert(ufs->phy_rst);
	if (ret)
		return ret;

	ret = readl_poll_timeout(ufs->ufs_top + UFS_TOP_PHY_CTRL, val,
				 val & UFS_TOP_PHY_SRAM_INIT_DONE, 10,
				 UFS_GOOGLE_SRAM_TIMEOUT_US);
	if (ret) {
		dev_err(hba->dev, "M-PHY init done timeout (0x%x)\n", val);
		return ret;
	}

	if (ufs->enable_cal && ufs->phy_cal_cnt > 0) {
		ret = ufs_google_apply_calibration(hba);
		if (ret)
			return ret;
	}

	ret = ufshcd_dme_set(hba, UIC_ARG_MIB(0xd0c1), 0);
	if (ret)
		return ret;

	ret = read_poll_timeout(ufshcd_dme_get, ret,
				ret || val == 1, 100,
				UFS_GOOGLE_PHY_READY_TIMEOUT_US, false,
				hba, UIC_ARG_MIB_SEL(0x0041, 0), &val);
	if (ret || val != 1)
		return ret ? ret : -ETIMEDOUT;

	ret = read_poll_timeout(ufshcd_dme_get, ret,
				ret || val == 1, 100,
				UFS_GOOGLE_PHY_READY_TIMEOUT_US, false,
				hba, UIC_ARG_MIB_SEL(0x0041, 1), &val);
	if (ret || val != 1)
		return ret ? ret : -ETIMEDOUT;

	ret = read_poll_timeout(ufshcd_dme_get, ret,
				ret || (val & 0x7), 100,
				UFS_GOOGLE_PHY_READY_TIMEOUT_US, false,
				hba, UIC_ARG_MIB_SEL(0x00c1, 4), &val);
	if (ret || !(val & 0x7))
		return ret ? ret : -ETIMEDOUT;

	ret = read_poll_timeout(ufshcd_dme_get, ret,
				ret || (val & 0x7), 100,
				UFS_GOOGLE_PHY_READY_TIMEOUT_US, false,
				hba, UIC_ARG_MIB_SEL(0x00c1, 5), &val);
	if (ret || !(val & 0x7))
		return ret ? ret : -ETIMEDOUT;

	ufs->need_phy_init = false;

	return 0;
}

static int ufs_google_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status status)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	u32 val;
	int ret;

	if (status == PRE_CHANGE) {
		ufshcd_writel(hba, UFS_HC_HCLK_DIV_200MHZ, UFS_HC_HCLK_DIV);
		writel(0x101, ufs->ufs_top + UFS_TOP_AWUSER_VC);

		return ufshcd_vops_phy_initialization(hba);
	}

	if (status != POST_CHANGE)
		return 0;

	/* Allow link and peer UniPro stack to settle before CPORT setup. */
	usleep_range(500, 600);

	ret = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_TACTIVATE), &val);
	if (!ret) {
		ret = ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE), val + 1);
		if (ret)
			dev_warn(hba->dev, "failed to bump PA_TACTIVATE (%d)\n", ret);
	}

	ret = ufs_google_dme_set_seq(hba, ufs_google_link_dme_seq,
				     ARRAY_SIZE(ufs_google_link_dme_seq));
	if (ret)
		return ret;

	ufshcd_rmwl(hba, UFS_HC_VENDOR_CTRL_CLR_MASK, 0, UFS_HC_VENDOR_CTRL);

	return 0;
}

static int ufs_google_negotiate_pwr_mode(struct ufs_hba *hba,
					 const struct ufs_pa_layer_attr *dev_max_params,
					 struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_host_params host_params;

	ufshcd_init_host_params(&host_params);
	host_params.hs_rx_gear = UFS_HS_G5;
	host_params.hs_tx_gear = UFS_HS_G5;
	ufshcd_parse_gear_limits(hba, &host_params);

	return ufshcd_negotiate_pwr_params(&host_params, dev_max_params,
					   dev_req_params);
}

static int ufs_google_pwr_change_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status,
					struct ufs_pa_layer_attr *final_params)
{
	u32 val;

	if (status != PRE_CHANGE || !final_params)
		return 0;

	dev_info(hba->dev, "hs_gear=%u hs_series=%u hs_rate=%u\n",
		 final_params->gear_rx, final_params->pwr_rx,
		 final_params->hs_rate);

	val = (ufshcd_is_hs_mode(final_params) &&
	       final_params->gear_rx >= UFS_HS_G4) ?
		PA_INITIAL_ADAPT : PA_NO_ADAPT;

	return ufshcd_dme_set(hba, UIC_ARG_MIB(UFS_GOOGLE_DME_GEAR_TUNE), val);
}

static int ufs_google_device_reset(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);

	if (!ufs->reset_gpio || hba->dev_info.wmanufacturerid != UFS_VENDOR_MICRON)
		return 0;

	gpiod_set_value_cansleep(ufs->reset_gpio, 0);
	usleep_range(200, 300);
	gpiod_set_value_cansleep(ufs->reset_gpio, 1);
	usleep_range(10000, 15000);

	return 0;
}

static int ufs_google_get_hba_mac(struct ufs_hba *hba)
{
	return 64;
}

static int ufs_google_mcq_config_resource(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	static const unsigned long opr_offsets[OPR_MAX] = {
		[OPR_SQD] = UFS_GOOGLE_MCQ_OPR_SQD_OFF,
		[OPR_SQIS] = UFS_GOOGLE_MCQ_OPR_SQIS_OFF,
		[OPR_CQD] = UFS_GOOGLE_MCQ_OPR_CQD_OFF,
		[OPR_CQIS] = UFS_GOOGLE_MCQ_OPR_CQIS_OFF,
	};
	int i;

	if (ufs->mcq_mapped)
		return 0;

	hba->mcq_base = devm_ioremap(dev,
				     ufs->hc_phys +
				     MCQ_QUEUE_OFFSET(hba->mcq_capabilities),
				     UFS_GOOGLE_MCQ_SIZE);
	if (!hba->mcq_base)
		return -ENOMEM;

	for (i = 0; i < OPR_MAX; i++) {
		ufs->mcq_opr_base[i] = devm_ioremap(dev,
						    ufs->hc_phys + opr_offsets[i],
						    UFS_GOOGLE_MCQ_OPR_SIZE);
		if (!ufs->mcq_opr_base[i])
			return -ENOMEM;
	}

	ufs->mcq_mapped = true;
	return 0;
}

static int ufs_google_op_runtime_config(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	static const struct {
		unsigned long offset;
		unsigned long stride;
	} opr_cfg[OPR_MAX] = {
		[OPR_SQD] = { UFS_GOOGLE_MCQ_OPR_SQD_OFF, UFS_GOOGLE_MCQ_SQD_STRIDE },
		[OPR_SQIS] = { UFS_GOOGLE_MCQ_OPR_SQIS_OFF, UFS_GOOGLE_MCQ_SQIS_STRIDE },
		[OPR_CQD] = { UFS_GOOGLE_MCQ_OPR_CQD_OFF, UFS_GOOGLE_MCQ_CQD_STRIDE },
		[OPR_CQIS] = { UFS_GOOGLE_MCQ_OPR_CQIS_OFF, UFS_GOOGLE_MCQ_CQIS_STRIDE },
	};
	int i;

	if (!ufs->mcq_mapped)
		return -EINVAL;

	for (i = 0; i < OPR_MAX; i++) {
		hba->mcq_opr[i].offset = opr_cfg[i].offset;
		hba->mcq_opr[i].stride = opr_cfg[i].stride;
		hba->mcq_opr[i].base = ufs->mcq_opr_base[i];
	}

	return 0;
}

static irqreturn_t ufs_google_mcq_intr(int irq, void *data)
{
	struct ufs_google_mcq_irq *m = data;
	struct ufs_hba *hba = m->hba;
	struct ufs_hw_queue *hwq = &hba->uhq[m->qid];
	u32 events;

	events = ufshcd_mcq_read_cqis(hba, m->qid);
	if (events)
		ufshcd_mcq_write_cqis(hba, events, m->qid);

	writel(UFS_GOOGLE_MCQIACR_CFG,
	       hba->mcq_opr[OPR_CQIS].base +
	       hba->mcq_opr[OPR_CQIS].stride * m->qid + REG_MCQIACR);

	if (events & UFSHCD_MCQ_CQIS_TAIL_ENT_PUSH_STS)
		ufshcd_mcq_poll_cqe_lock(hba, hwq);

	return IRQ_HANDLED;
}

static int ufs_google_config_esi(struct ufs_hba *hba)
{
	struct ufs_google_host *ufs = ufshcd_get_variant(hba);
	struct platform_device *pdev = to_platform_device(hba->dev);
	u32 nr_irqs;
	int i, irq, ret;

	if (device_property_read_bool(hba->dev, "google,skip-mcq-multi-intr"))
		return -EOPNOTSUPP;

	nr_irqs = hba->nr_hw_queues - hba->nr_queues[HCTX_TYPE_POLL];
	if (!nr_irqs || nr_irqs > UFS_GOOGLE_MAX_MCQ_IRQS)
		return -EOPNOTSUPP;

	for (i = 0; i < nr_irqs; i++) {
		if (ufs->mcq_irqs[i].requested)
			continue;

		irq = platform_get_irq(pdev, 1 + i);
		if (irq < 0)
			return irq;

		ufs->mcq_irqs[i].hba = hba;
		ufs->mcq_irqs[i].irq = irq;
		ufs->mcq_irqs[i].qid = i;

		ret = devm_request_irq(hba->dev, irq, ufs_google_mcq_intr, 0,
				       devm_kasprintf(hba->dev, GFP_KERNEL,
						      "ufs-google-mcq-%d", i),
				       &ufs->mcq_irqs[i]);
		if (ret)
			return ret;

		ufs->mcq_irqs[i].requested = true;
	}
	ufs->nr_mcq_irqs = nr_irqs;

	ufshcd_rmwl(hba, MCQ_IAG_EVENT_STATUS, 0, REG_INTERRUPT_ENABLE);
	hba->quirks |= UFSHCD_QUIRK_MCQ_BROKEN_INTR;

	for (i = 0; i < nr_irqs; i++) {
		u32 cqis_off = hba->mcq_opr[OPR_CQIS].offset +
			       hba->mcq_opr[OPR_CQIS].stride * i;

		writel(UFS_GOOGLE_MCQ_ROUTE_EN | i,
		       hba->mcq_base + UFS_GOOGLE_MCQ_QCFG_STRIDE * i +
		       UFS_GOOGLE_MCQ_ROUTE_REG);
		writel_relaxed(cqis_off,
			       hba->mcq_base + UFS_GOOGLE_MCQ_QCFG_STRIDE * i +
			       REG_CQISAO);
		writel(UFS_GOOGLE_MCQIACR_CFG,
		       hba->mcq_opr[OPR_CQIS].base +
		       hba->mcq_opr[OPR_CQIS].stride * i + REG_MCQIACR);
	}

	return 0;
}

static int ufs_google_set_dma_mask(struct ufs_hba *hba)
{
	return dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(36));
}

static const struct ufs_hba_variant_ops ufs_hba_google_vops = {
	.name			= "google-ufshc",
	.init			= ufs_google_init,
	.exit			= ufs_google_exit,
	.set_dma_mask		= ufs_google_set_dma_mask,
	.phy_initialization	= ufs_google_phy_initialization,
	.link_startup_notify	= ufs_google_link_startup_notify,
	.negotiate_pwr_mode	= ufs_google_negotiate_pwr_mode,
	.pwr_change_notify	= ufs_google_pwr_change_notify,
	.device_reset		= ufs_google_device_reset,
	.get_hba_mac		= ufs_google_get_hba_mac,
	.mcq_config_resource	= ufs_google_mcq_config_resource,
	.op_runtime_config	= ufs_google_op_runtime_config,
	.config_esi		= ufs_google_config_esi,
};

static int ufs_google_probe(struct platform_device *pdev)
{
	return ufshcd_pltfrm_init(pdev, &ufs_hba_google_vops);
}

static void ufs_google_remove(struct platform_device *pdev)
{
	ufshcd_pltfrm_remove(pdev);
}

static const struct of_device_id ufs_google_of_match[] = {
	{ .compatible = "google,ufshc" },
	{},
};
MODULE_DEVICE_TABLE(of, ufs_google_of_match);

static const struct dev_pm_ops ufs_google_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare	= ufshcd_suspend_prepare,
	.complete	= ufshcd_resume_complete,
};

static struct platform_driver ufs_google_pltform = {
	.probe		= ufs_google_probe,
	.remove		= ufs_google_remove,
	.driver		= {
		.name		= "google-ufshc",
		.pm		= pm_ptr(&ufs_google_pm_ops),
		.of_match_table	= ufs_google_of_match,
	},
};
module_platform_driver(ufs_google_pltform);

MODULE_DESCRIPTION("Google Tensor Laguna UFS Host Controller Driver");
MODULE_LICENSE("GPL");
