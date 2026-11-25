// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2DOS07X02 sub-PMIC buck regulator driver.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define S2DOS07X02_REG_BUCK_VOUT	0x03
#define S2DOS07X02_REG_BUCK_EN		0x04

#define S2DOS07X02_MIN_UV		850000
#define S2DOS07X02_MAX_UV		2100000
#define S2DOS07X02_STEP_UV		12500
#define S2DOS07X02_N_VOUTS		(((S2DOS07X02_MAX_UV - S2DOS07X02_MIN_UV) / \
					  S2DOS07X02_STEP_UV) + 1)
#define S2DOS07X02_VOUT_BASE		0x9b
#define S2DOS07X02_BUCK_EN_ON		0x01
#define S2DOS07X02_BUCK_EN_OFF		0x00

struct s2dos07x02_pmic {
	struct device *dev;
	struct i2c_client *i2c;
	const struct regulator_desc *desc;
	bool enabled;
	u8 vout;
};

static int s2dos07x02_is_enabled(struct regulator_dev *rdev)
{
	struct s2dos07x02_pmic *pmic = rdev_get_drvdata(rdev);

	return pmic->enabled;
}

static int s2dos07x02_enable(struct regulator_dev *rdev)
{
	struct s2dos07x02_pmic *pmic = rdev_get_drvdata(rdev);
	int ret;

	if (pmic->enabled)
		return 0;

	pmic->enabled = true;
	ret = i2c_smbus_write_byte_data(pmic->i2c, S2DOS07X02_REG_BUCK_EN,
					S2DOS07X02_BUCK_EN_ON);
	if (ret < 0) {
		dev_warn(pmic->dev, "enable write failed (%d)\n", ret);
		if (ret == -ETIMEDOUT)
			return 0;
		return ret;
	}

	usleep_range(200, 210);
	ret = i2c_smbus_write_byte_data(pmic->i2c, S2DOS07X02_REG_BUCK_VOUT,
					pmic->vout);
	if (ret < 0 && ret != -ETIMEDOUT)
		return ret;

	return 0;
}

static int s2dos07x02_disable(struct regulator_dev *rdev)
{
	struct s2dos07x02_pmic *pmic = rdev_get_drvdata(rdev);
	int ret;

	if (!pmic->enabled)
		return 0;

	pmic->enabled = false;
	ret = i2c_smbus_write_byte_data(pmic->i2c, S2DOS07X02_REG_BUCK_EN,
					S2DOS07X02_BUCK_EN_OFF);
	if (ret < 0) {
		dev_warn(pmic->dev, "disable write failed (%d)\n", ret);
		if (ret == -ETIMEDOUT)
			return 0;
	}

	return ret;
}

static int s2dos07x02_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2dos07x02_pmic *pmic = rdev_get_drvdata(rdev);
	int vout, uV;

	vout = i2c_smbus_read_byte_data(pmic->i2c, S2DOS07X02_REG_BUCK_VOUT);
	if (vout < 0) {
		dev_warn(pmic->dev, "vout read failed (%d)\n", vout);
		if (vout != -ETIMEDOUT)
			return vout;
		vout = pmic->vout;
	}

	uV = ((vout - S2DOS07X02_VOUT_BASE) * S2DOS07X02_STEP_UV) +
	     S2DOS07X02_MIN_UV;

	return regulator_map_voltage_linear(rdev, uV, uV);
}

static int s2dos07x02_set_voltage_sel(struct regulator_dev *rdev,
				      unsigned int sel)
{
	struct s2dos07x02_pmic *pmic = rdev_get_drvdata(rdev);
	int uV, ret;
	u8 vout;

	uV = regulator_list_voltage_linear(rdev, sel);
	if (uV < 0)
		return uV;

	vout = S2DOS07X02_VOUT_BASE + ((uV - S2DOS07X02_MIN_UV) /
				       S2DOS07X02_STEP_UV);
	pmic->vout = vout;

	ret = i2c_smbus_write_byte_data(pmic->i2c, S2DOS07X02_REG_BUCK_VOUT, vout);
	if (ret < 0) {
		dev_warn(pmic->dev, "vout write failed (%d)\n", ret);
		if (ret == -ETIMEDOUT)
			return 0;
	}

	return ret;
}

static const struct regulator_ops s2dos07x02_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2dos07x02_is_enabled,
	.enable			= s2dos07x02_enable,
	.disable		= s2dos07x02_disable,
	.get_voltage_sel	= s2dos07x02_get_voltage_sel,
	.set_voltage_sel	= s2dos07x02_set_voltage_sel,
};

static const struct regulator_desc s2dos07x02_desc = {
	.name		= "s2dos07x02-pmic",
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.min_uV		= S2DOS07X02_MIN_UV,
	.uV_step	= S2DOS07X02_STEP_UV,
	.n_voltages	= S2DOS07X02_N_VOUTS,
	.ops		= &s2dos07x02_buck_ops,
};

static struct regulator_init_data s2dos07x02_default_init_data = {
	.constraints = {
		.min_uV = S2DOS07X02_MIN_UV,
		.max_uV = S2DOS07X02_MAX_UV,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
	},
};

static int s2dos07x02_probe(struct i2c_client *i2c)
{
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct s2dos07x02_pmic *pmic;
	struct regulator_dev *rdev;
	int val;

	pmic = devm_kzalloc(&i2c->dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->dev = &i2c->dev;
	pmic->i2c = i2c;
	pmic->desc = &s2dos07x02_desc;
	i2c_set_clientdata(i2c, pmic);

	i2c->adapter->timeout = 50;
	i2c->adapter->retries = 2;

	val = i2c_smbus_read_byte_data(i2c, S2DOS07X02_REG_BUCK_EN);
	if (val < 0)
		return val;
	pmic->enabled = !!val;

	val = i2c_smbus_read_byte_data(i2c, S2DOS07X02_REG_BUCK_VOUT);
	if (val < 0)
		return val;
	pmic->vout = val;

	init_data = of_get_regulator_init_data(&i2c->dev, i2c->dev.of_node,
					       &s2dos07x02_desc);
	if (!init_data)
		init_data = &s2dos07x02_default_init_data;

	cfg.dev = &i2c->dev;
	cfg.of_node = i2c->dev.of_node;
	cfg.driver_data = pmic;
	cfg.init_data = init_data;

	rdev = devm_regulator_register(&i2c->dev, pmic->desc, &cfg);
	if (IS_ERR(rdev))
		return dev_err_probe(&i2c->dev, PTR_ERR(rdev),
				     "failed to register s2dos07x02 regulator\n");

	return 0;
}

static const struct of_device_id s2dos07x02_of_match[] = {
	{ .compatible = "samsung,s2dos07x02" },
	{}
};
MODULE_DEVICE_TABLE(of, s2dos07x02_of_match);

static struct i2c_driver s2dos07x02_driver = {
	.driver = {
		.name = "s2dos07x02",
		.of_match_table = s2dos07x02_of_match,
	},
	.probe = s2dos07x02_probe,
};
module_i2c_driver(s2dos07x02_driver);

MODULE_DESCRIPTION("Samsung S2DOS07X02 sub-PMIC regulator driver");
MODULE_LICENSE("GPL");
