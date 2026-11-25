// SPDX-License-Identifier: GPL-2.0-only
/*
 * DA9188 / DA9189 PMIC regulator driver over Google CPM MBA.
 */

#include <linux/mfd/da9188.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

enum da9188_regulator_id {
	/* DA9188 (M) bucks */
	DA9188_REGL_BUCK1M = 0,
	DA9188_REGL_BUCK2M,
	DA9188_REGL_BUCK3M,
	DA9188_REGL_BUCK4M,
	DA9188_REGL_BUCK5M,
	DA9188_REGL_BUCK6M,
	DA9188_REGL_BUCK7M,
	DA9188_REGL_BUCK8M,
	DA9188_REGL_BUCK9M,
	DA9188_REGL_BUCK10M,
	DA9188_REGL_BUCK11M,
	DA9188_REGL_BUCK12M,
	DA9188_REGL_BUCK13M,
	/* DA9188 (M) LDOs */
	DA9188_REGL_LDO1M,
	DA9188_REGL_LDO2M,
	DA9188_REGL_LDO3M,
	DA9188_REGL_LDO4M,
	DA9188_REGL_LDO5M,
	DA9188_REGL_LDO6M,
	DA9188_REGL_LDO7M,
	DA9188_REGL_LDO8M,
	DA9188_REGL_LDO9M,
	DA9188_REGL_LDO10M,
	DA9188_REGL_LDO11M,
	DA9188_REGL_LDO12M,
	DA9188_REGL_LDO13M,
	DA9188_REGL_LDO14M,
	DA9188_REGL_LDO15M,
	DA9188_REGL_LDO16M,
	DA9188_REGL_LDO17M,
	DA9188_REGL_LDO18M,
	DA9188_REGL_LDO19M,
	DA9188_REGL_LDO20M,
	DA9188_REGL_LDO21M,
	DA9188_REGL_LDO22M,
	DA9188_REGL_LDO23M,
	DA9188_REGL_LDO24M,
	DA9188_REGL_LDO25M,
	DA9188_REGL_LDO26M,
	DA9188_REGL_LDO27M,
	DA9188_REGL_LDO28M,
	/* DA9189 (S) bucks + buck-boost */
	DA9188_REGL_BUCK1S,
	DA9188_REGL_BUCK2S,
	DA9188_REGL_BUCK3S,
	DA9188_REGL_BUCK4S,
	DA9188_REGL_BUCK5S,
	DA9188_REGL_BUCK6S,
	DA9188_REGL_BUCK7S,
	DA9188_REGL_BUCK8S,
	DA9188_REGL_BUCK9S,
	DA9188_REGL_BUCK10S,
	DA9188_REGL_BUCK11S,
	DA9188_REGL_BUCK12S,
	DA9188_REGL_BUCK13S,
	DA9188_REGL_BUBO,
	/* DA9189 (S) LDOs */
	DA9188_REGL_LDO1S,
	DA9188_REGL_LDO2S,
	DA9188_REGL_LDO3S,
	DA9188_REGL_LDO4S,
	DA9188_REGL_LDO5S,
	DA9188_REGL_LDO6S,
	DA9188_REGL_LDO7S,
	DA9188_REGL_LDO8S,
	DA9188_REGL_LDO9S,
	DA9188_REGL_LDO10S,
	DA9188_REGL_LDO11S,
	DA9188_REGL_LDO12S,
	DA9188_REGL_LDO13S,
	DA9188_REGL_LDO14S,
	DA9188_REGL_LDO15S,
	DA9188_REGL_LDO16S,
	DA9188_REGL_LDO17S,
	DA9188_REGL_LDO18S,
	DA9188_REGL_LDO19S,
	DA9188_REGL_LDO20S,
	DA9188_REGL_LDO21S,
	DA9188_REGL_LDO22S,
	DA9188_REGL_LDO23S,
	DA9188_REGL_LDO24S,
	DA9188_REGL_LDO25S,
	DA9188_REGL_LDO26S,
	DA9188_REGL_LDO27S,
	DA9188_REGL_LDO28S,
	DA9188_REGL_MAX,
};

#define DA9188_NUM_M_REGS	(DA9188_REGL_BUCK1S - DA9188_REGL_BUCK1M)
#define DA9188_NUM_S_REGS	(DA9188_REGL_MAX - DA9188_REGL_BUCK1S)

struct da9188_regulator_priv {
	struct device *dev;
	struct da9188_mbox mbox;
	u32 dst_channel;
};

static int da9188_regl_get_u32(struct regulator_dev *rdev, u8 cmd, u32 *val)
{
	struct da9188_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct da9188_mbox_data req = {};
	struct da9188_mbox_data resp;
	int ret;

	ret = da9188_mbox_xfer(priv->dev, &priv->mbox, priv->dst_channel,
			       DA9188_MB_TARGET_REGULATOR, cmd, rdev->desc->id,
			       &req, &resp);
	if (!ret)
		*val = resp.data[0];

	return ret;
}

static int da9188_regl_set_u32(struct regulator_dev *rdev, u8 cmd, u32 val)
{
	struct da9188_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct da9188_mbox_data req = { .data = { val, 0 } };

	return da9188_mbox_xfer(priv->dev, &priv->mbox, priv->dst_channel,
				DA9188_MB_TARGET_REGULATOR, cmd, rdev->desc->id,
				&req, NULL);
}

static int da9188_regl_get_voltage_sel(struct regulator_dev *rdev)
{
	u32 sel = 0;
	int ret;

	ret = da9188_regl_get_u32(rdev, DA9188_REGL_CMD_GET_VOLTAGE_SEL, &sel);
	return ret ? ret : (int)sel;
}

static int da9188_regl_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	return da9188_regl_set_u32(rdev, DA9188_REGL_CMD_SET_VOLTAGE_SEL, sel);
}

static int da9188_regl_is_enabled(struct regulator_dev *rdev)
{
	u32 en = 0;
	int ret;

	ret = da9188_regl_get_u32(rdev, DA9188_REGL_CMD_GET_IS_ENABLED, &en);
	return ret ? ret : !!en;
}

static int da9188_regl_enable(struct regulator_dev *rdev)
{
	return da9188_regl_set_u32(rdev, DA9188_REGL_CMD_SET_ENABLE_DISABLE, 1);
}

static int da9188_regl_disable(struct regulator_dev *rdev)
{
	return da9188_regl_set_u32(rdev, DA9188_REGL_CMD_SET_ENABLE_DISABLE, 0);
}

static int da9188_regl_get_bypass(struct regulator_dev *rdev, bool *enable)
{
	u32 bypass = 0;
	int ret;

	ret = da9188_regl_get_u32(rdev, DA9188_REGL_CMD_GET_BYPASS, &bypass);
	if (!ret)
		*enable = !!bypass;

	return ret;
}

static int da9188_regl_set_bypass(struct regulator_dev *rdev, bool enable)
{
	return da9188_regl_set_u32(rdev, DA9188_REGL_CMD_SET_BYPASS, enable);
}

static unsigned int da9188_regl_get_mode(struct regulator_dev *rdev)
{
	u32 mode = 0;
	int ret;

	ret = da9188_regl_get_u32(rdev, DA9188_REGL_CMD_GET_MODE, &mode);
	if (ret)
		return REGULATOR_MODE_INVALID;

	return mode ? REGULATOR_MODE_STANDBY : REGULATOR_MODE_NORMAL;
}

static int da9188_regl_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	u32 raw;

	if (mode == REGULATOR_MODE_NORMAL)
		raw = 0;
	else if (mode == REGULATOR_MODE_STANDBY)
		raw = 1;
	else
		return -EINVAL;

	return da9188_regl_set_u32(rdev, DA9188_REGL_CMD_SET_MODE, raw);
}

static unsigned int da9188_regl_of_map_mode(unsigned int mode)
{
	if (mode == REGULATOR_MODE_NORMAL || mode == REGULATOR_MODE_STANDBY)
		return mode;

	return REGULATOR_MODE_INVALID;
}

static const struct regulator_ops da9188_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= da9188_regl_get_voltage_sel,
	.set_voltage_sel	= da9188_regl_set_voltage_sel,
	.is_enabled		= da9188_regl_is_enabled,
	.enable			= da9188_regl_enable,
	.disable		= da9188_regl_disable,
	.get_mode		= da9188_regl_get_mode,
	.set_mode		= da9188_regl_set_mode,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static const struct regulator_ops da9188_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= da9188_regl_get_voltage_sel,
	.set_voltage_sel	= da9188_regl_set_voltage_sel,
	.is_enabled		= da9188_regl_is_enabled,
	.enable			= da9188_regl_enable,
	.disable		= da9188_regl_disable,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static const struct regulator_ops da9188_ldo_bypass_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= da9188_regl_get_voltage_sel,
	.set_voltage_sel	= da9188_regl_set_voltage_sel,
	.is_enabled		= da9188_regl_is_enabled,
	.enable			= da9188_regl_enable,
	.disable		= da9188_regl_disable,
	.get_bypass		= da9188_regl_get_bypass,
	.set_bypass		= da9188_regl_set_bypass,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static const struct regulator_ops da9188_monitor_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= da9188_regl_get_voltage_sel,
	.is_enabled		= da9188_regl_is_enabled,
	.get_mode		= da9188_regl_get_mode,
};

#define DA9188_BUCK_DESC(_name, _id, _min, _step) {		\
	.name			= #_name,			\
	.of_match		= of_match_ptr(#_name),		\
	.regulators_node	= of_match_ptr("regulators"),	\
	.ops			= &da9188_buck_ops,		\
	.type			= REGULATOR_VOLTAGE,		\
	.id			= (_id),			\
	.owner			= THIS_MODULE,			\
	.n_voltages		= 256,				\
	.min_uV			= (_min),			\
	.uV_step		= (_step),			\
	.of_map_mode		= da9188_regl_of_map_mode,	\
}

#define DA9188_LDO_DESC(_name, _id, _ops, _min, _step) {	\
	.name			= #_name,			\
	.of_match		= of_match_ptr(#_name),		\
	.regulators_node	= of_match_ptr("regulators"),	\
	.ops			= &(_ops),			\
	.type			= REGULATOR_VOLTAGE,		\
	.id			= (_id),			\
	.owner			= THIS_MODULE,			\
	.n_voltages		= 256,				\
	.min_uV			= (_min),			\
	.uV_step		= (_step),			\
}

static const struct regulator_desc da9188_regulators_desc[DA9188_REGL_MAX] = {
	/* DA9188 (M) */
	[DA9188_REGL_BUCK1M]	= DA9188_BUCK_DESC(buck_1m, DA9188_REGL_BUCK1M, 240000, 5000),
	[DA9188_REGL_BUCK2M]	= DA9188_BUCK_DESC(buck_2m, DA9188_REGL_BUCK2M, 240000, 5000),
	[DA9188_REGL_BUCK3M]	= DA9188_BUCK_DESC(buck_3m, DA9188_REGL_BUCK3M, 240000, 5000),
	[DA9188_REGL_BUCK4M]	= DA9188_BUCK_DESC(buck_4m, DA9188_REGL_BUCK4M, 240000, 5000),
	[DA9188_REGL_BUCK5M]	= DA9188_BUCK_DESC(buck_5m, DA9188_REGL_BUCK5M, 240000, 5000),
	[DA9188_REGL_BUCK6M]	= DA9188_BUCK_DESC(buck_6m, DA9188_REGL_BUCK6M, 240000, 5000),
	[DA9188_REGL_BUCK7M]	= DA9188_BUCK_DESC(buck_7m, DA9188_REGL_BUCK7M, 240000, 5000),
	[DA9188_REGL_BUCK8M]	= DA9188_BUCK_DESC(buck_8m, DA9188_REGL_BUCK8M, 240000, 5000),
	[DA9188_REGL_BUCK9M]	= DA9188_BUCK_DESC(buck_9m, DA9188_REGL_BUCK9M, 240000, 5000),
	[DA9188_REGL_BUCK10M]	= DA9188_BUCK_DESC(buck_10m, DA9188_REGL_BUCK10M, 240000, 5000),
	[DA9188_REGL_BUCK11M]	= DA9188_BUCK_DESC(buck_11m, DA9188_REGL_BUCK11M, 240000, 5000),
	[DA9188_REGL_BUCK12M]	= DA9188_BUCK_DESC(buck_12m, DA9188_REGL_BUCK12M, 240000, 5000),
	[DA9188_REGL_BUCK13M]	= DA9188_BUCK_DESC(buck_13m, DA9188_REGL_BUCK13M, 240000, 5000),
	[DA9188_REGL_LDO1M]	= DA9188_LDO_DESC(ldo_1m, DA9188_REGL_LDO1M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO2M]	= DA9188_LDO_DESC(ldo_2m, DA9188_REGL_LDO2M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO3M]	= DA9188_LDO_DESC(ldo_3m, DA9188_REGL_LDO3M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO4M]	= DA9188_LDO_DESC(ldo_4m, DA9188_REGL_LDO4M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO5M]	= DA9188_LDO_DESC(ldo_5m, DA9188_REGL_LDO5M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO6M]	= DA9188_LDO_DESC(ldo_6m, DA9188_REGL_LDO6M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO7M]	= DA9188_LDO_DESC(ldo_7m, DA9188_REGL_LDO7M, da9188_ldo_ops, 960000, 15000),
	[DA9188_REGL_LDO8M]	= DA9188_LDO_DESC(ldo_8m, DA9188_REGL_LDO8M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO9M]	= DA9188_LDO_DESC(ldo_9m, DA9188_REGL_LDO9M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO10M]	= DA9188_LDO_DESC(ldo_10m, DA9188_REGL_LDO10M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO11M]	= DA9188_LDO_DESC(ldo_11m, DA9188_REGL_LDO11M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO12M]	= DA9188_LDO_DESC(ldo_12m, DA9188_REGL_LDO12M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO13M]	= DA9188_LDO_DESC(ldo_13m, DA9188_REGL_LDO13M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO14M]	= DA9188_LDO_DESC(ldo_14m, DA9188_REGL_LDO14M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO15M]	= DA9188_LDO_DESC(ldo_15m, DA9188_REGL_LDO15M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO16M]	= DA9188_LDO_DESC(ldo_16m, DA9188_REGL_LDO16M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO17M]	= DA9188_LDO_DESC(ldo_17m, DA9188_REGL_LDO17M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO18M]	= DA9188_LDO_DESC(ldo_18m, DA9188_REGL_LDO18M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO19M]	= DA9188_LDO_DESC(ldo_19m, DA9188_REGL_LDO19M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO20M]	= DA9188_LDO_DESC(ldo_20m, DA9188_REGL_LDO20M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO21M]	= DA9188_LDO_DESC(ldo_21m, DA9188_REGL_LDO21M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO22M]	= DA9188_LDO_DESC(ldo_22m, DA9188_REGL_LDO22M, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO23M]	= DA9188_LDO_DESC(ldo_23m, DA9188_REGL_LDO23M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO24M]	= DA9188_LDO_DESC(ldo_24m, DA9188_REGL_LDO24M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO25M]	= DA9188_LDO_DESC(ldo_25m, DA9188_REGL_LDO25M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO26M]	= DA9188_LDO_DESC(ldo_26m, DA9188_REGL_LDO26M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO27M]	= DA9188_LDO_DESC(ldo_27m, DA9188_REGL_LDO27M, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO28M]	= DA9188_LDO_DESC(ldo_28m, DA9188_REGL_LDO28M, da9188_ldo_ops, 400000, 5000),
	/* DA9189 (S) */
	[DA9188_REGL_BUCK1S]	= DA9188_BUCK_DESC(buck_1s, DA9188_REGL_BUCK1S, 240000, 5000),
	[DA9188_REGL_BUCK2S]	= DA9188_BUCK_DESC(buck_2s, DA9188_REGL_BUCK2S, 240000, 5000),
	[DA9188_REGL_BUCK3S]	= DA9188_BUCK_DESC(buck_3s, DA9188_REGL_BUCK3S, 240000, 5000),
	[DA9188_REGL_BUCK4S]	= DA9188_BUCK_DESC(buck_4s, DA9188_REGL_BUCK4S, 240000, 5000),
	[DA9188_REGL_BUCK5S]	= DA9188_BUCK_DESC(buck_5s, DA9188_REGL_BUCK5S, 240000, 5000),
	[DA9188_REGL_BUCK6S]	= DA9188_BUCK_DESC(buck_6s, DA9188_REGL_BUCK6S, 240000, 5000),
	[DA9188_REGL_BUCK7S]	= DA9188_BUCK_DESC(buck_7s, DA9188_REGL_BUCK7S, 800000, 10000),
	[DA9188_REGL_BUCK8S]	= DA9188_BUCK_DESC(buck_8s, DA9188_REGL_BUCK8S, 240000, 5000),
	[DA9188_REGL_BUCK9S]	= DA9188_BUCK_DESC(buck_9s, DA9188_REGL_BUCK9S, 240000, 5000),
	[DA9188_REGL_BUCK10S]	= DA9188_BUCK_DESC(buck_10s, DA9188_REGL_BUCK10S, 240000, 5000),
	[DA9188_REGL_BUCK11S]	= DA9188_BUCK_DESC(buck_11s, DA9188_REGL_BUCK11S, 800000, 10000),
	[DA9188_REGL_BUCK12S]	= DA9188_BUCK_DESC(buck_12s, DA9188_REGL_BUCK12S, 800000, 10000),
	[DA9188_REGL_BUCK13S]	= DA9188_BUCK_DESC(buck_13s, DA9188_REGL_BUCK13S, 240000, 5000),
	[DA9188_REGL_BUBO]	= DA9188_BUCK_DESC(buck_boost, DA9188_REGL_BUBO, 1300000, 10000),
	[DA9188_REGL_LDO1S]	= DA9188_LDO_DESC(ldo_1s, DA9188_REGL_LDO1S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO2S]	= DA9188_LDO_DESC(ldo_2s, DA9188_REGL_LDO2S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO3S]	= DA9188_LDO_DESC(ldo_3s, DA9188_REGL_LDO3S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO4S]	= DA9188_LDO_DESC(ldo_4s, DA9188_REGL_LDO4S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO5S]	= DA9188_LDO_DESC(ldo_5s, DA9188_REGL_LDO5S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO6S]	= DA9188_LDO_DESC(ldo_6s, DA9188_REGL_LDO6S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO7S]	= DA9188_LDO_DESC(ldo_7s, DA9188_REGL_LDO7S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO8S]	= DA9188_LDO_DESC(ldo_8s, DA9188_REGL_LDO8S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO9S]	= DA9188_LDO_DESC(ldo_9s, DA9188_REGL_LDO9S, da9188_ldo_bypass_ops, 400000, 5000),
	[DA9188_REGL_LDO10S]	= DA9188_LDO_DESC(ldo_10s, DA9188_REGL_LDO10S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO11S]	= DA9188_LDO_DESC(ldo_11s, DA9188_REGL_LDO11S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO12S]	= DA9188_LDO_DESC(ldo_12s, DA9188_REGL_LDO12S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO13S]	= DA9188_LDO_DESC(ldo_13s, DA9188_REGL_LDO13S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO14S]	= DA9188_LDO_DESC(ldo_14s, DA9188_REGL_LDO14S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO15S]	= DA9188_LDO_DESC(ldo_15s, DA9188_REGL_LDO15S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO16S]	= DA9188_LDO_DESC(ldo_16s, DA9188_REGL_LDO16S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO17S]	= DA9188_LDO_DESC(ldo_17s, DA9188_REGL_LDO17S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO18S]	= DA9188_LDO_DESC(ldo_18s, DA9188_REGL_LDO18S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO19S]	= DA9188_LDO_DESC(ldo_19s, DA9188_REGL_LDO19S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO20S]	= DA9188_LDO_DESC(ldo_20s, DA9188_REGL_LDO20S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO21S]	= DA9188_LDO_DESC(ldo_21s, DA9188_REGL_LDO21S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO22S]	= DA9188_LDO_DESC(ldo_22s, DA9188_REGL_LDO22S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO23S]	= DA9188_LDO_DESC(ldo_23s, DA9188_REGL_LDO23S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO24S]	= DA9188_LDO_DESC(ldo_24s, DA9188_REGL_LDO24S, da9188_ldo_ops, 400000, 5000),
	[DA9188_REGL_LDO25S]	= DA9188_LDO_DESC(ldo_25s, DA9188_REGL_LDO25S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO26S]	= DA9188_LDO_DESC(ldo_26s, DA9188_REGL_LDO26S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO27S]	= DA9188_LDO_DESC(ldo_27s, DA9188_REGL_LDO27S, da9188_ldo_ops, 1200000, 10000),
	[DA9188_REGL_LDO28S]	= DA9188_LDO_DESC(ldo_28s, DA9188_REGL_LDO28S, da9188_ldo_ops, 1200000, 10000),
};

static int da9188_register_chip_regulators(struct da9188_regulator_priv *priv,
					   struct device_node *chip_np,
					   unsigned int start_idx,
					   unsigned int count)
{
	struct device *dev = priv->dev;
	struct of_regulator_match *matches;
	unsigned int i;
	int matched;

	matches = devm_kcalloc(dev, count, sizeof(*matches), GFP_KERNEL);
	if (!matches)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		matches[i].name = da9188_regulators_desc[start_idx + i].name;
		matches[i].desc = &da9188_regulators_desc[start_idx + i];
	}

	matched = of_regulator_match(dev, chip_np, matches, count);
	if (matched < 0)
		return matched;

	for (i = 0; i < count; i++) {
		struct regulator_config cfg = {
			.dev = dev,
			.driver_data = priv,
			.of_node = matches[i].of_node,
			.init_data = matches[i].init_data,
		};
		const struct regulator_desc *desc = matches[i].desc;
		struct regulator_dev *rdev;

		if (!matches[i].init_data)
			continue;

		if (of_property_read_bool(matches[i].of_node, "google,monitor-only")) {
			struct regulator_desc *mon_desc;

			matches[i].init_data->constraints.apply_uV = 0;
			mon_desc = devm_kmemdup(dev, desc, sizeof(*mon_desc),
						GFP_KERNEL);
			if (!mon_desc)
				return -ENOMEM;
			mon_desc->ops = &da9188_monitor_ops;
			desc = mon_desc;
		}

		rdev = devm_regulator_register(dev, desc, &cfg);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     matches[i].name);
	}

	return matched;
}

static int da9188_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct da9188_regulator_priv *priv;
	struct device_node *regs_np, *m_np, *s_np;
	int ret, total = 0;

	if (!dev->of_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->dst_channel = CPM_COMMON_PMIC_SERVICE;
	of_property_read_u32(dev->of_node, "mba-dest-channel", &priv->dst_channel);
	platform_set_drvdata(pdev, priv);

	ret = da9188_devm_mbox_request(dev, &priv->mbox);
	if (ret)
		return ret;

	regs_np = of_get_child_by_name(dev->of_node, "regulators");
	if (!regs_np)
		return -ENODEV;

	m_np = of_get_child_by_name(regs_np, "da9188");
	if (m_np) {
		ret = da9188_register_chip_regulators(priv, m_np,
						      DA9188_REGL_BUCK1M,
						      DA9188_NUM_M_REGS);
		of_node_put(m_np);
		if (ret < 0)
			goto out_put_regs;
		total += ret;
	}

	s_np = of_get_child_by_name(regs_np, "da9189");
	if (s_np) {
		ret = da9188_register_chip_regulators(priv, s_np,
						      DA9188_REGL_BUCK1S,
						      DA9188_NUM_S_REGS);
		of_node_put(s_np);
		if (ret < 0)
			goto out_put_regs;
		total += ret;
	}

	of_node_put(regs_np);
	return total > 0 ? 0 : -ENODEV;

out_put_regs:
	of_node_put(regs_np);
	return ret;
}

static const struct platform_device_id da9188_regulator_id_table[] = {
	{ "da9188-regulator", 0 },
	{}
};
MODULE_DEVICE_TABLE(platform, da9188_regulator_id_table);

static struct platform_driver da9188_regulator_driver = {
	.probe = da9188_regulator_probe,
	.driver = {
		.name = "da9188-regulator",
	},
	.id_table = da9188_regulator_id_table,
};
module_platform_driver(da9188_regulator_driver);

MODULE_DESCRIPTION("DA9188 / DA9189 PMIC regulator driver");
MODULE_LICENSE("GPL");
