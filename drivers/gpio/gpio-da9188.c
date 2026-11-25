// SPDX-License-Identifier: GPL-2.0-only
/*
 * DA9188 / DA9189 PMIC GPIO and pinconf driver over CPM MBA.
 */

#include <linux/gpio/driver.h>
#include <linux/mfd/da9188.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

enum da9188_gpio_dir {
	DA9188_GPIO_DIR_IN	= 0,
	DA9188_GPIO_DIR_OUT	= 1,
};

enum da9188_gpio_out_drive {
	DA9188_GPIO_DRIVE_PP	= 0,
	DA9188_GPIO_DRIVE_OD	= 1,
};

enum da9188_gpio_drv_str {
	DA9188_GPIO_DRV_2MA	= 0,
	DA9188_GPIO_DRV_4MA	= 1,
	DA9188_GPIO_DRV_6MA	= 2,
	DA9188_GPIO_DRV_8MA	= 3,
};

enum da9188_gpio_pull {
	DA9188_GPIO_PULL_OFF	= 0,
	DA9188_GPIO_PULL_UP_20K	= 1,
	DA9188_GPIO_PULL_UP_800K = 2,
	DA9188_GPIO_PULL_DN_20K	= 3,
	DA9188_GPIO_PULL_DN_800K = 4,
};

static const u32 da9188_drv_ma_table[] = { 2, 4, 6, 8 };

/* DA9188 (0..7) + DA9189 (8..18) = 19 GPIOs */
static const struct pinctrl_pin_desc da9188_pins[] = {
	PINCTRL_PIN(0,  "gpio0"),  PINCTRL_PIN(1,  "gpio1"),
	PINCTRL_PIN(2,  "gpio2"),  PINCTRL_PIN(3,  "gpio3"),
	PINCTRL_PIN(4,  "gpio4"),  PINCTRL_PIN(5,  "gpio5"),
	PINCTRL_PIN(6,  "gpio6"),  PINCTRL_PIN(7,  "gpio7"),
	PINCTRL_PIN(8,  "gpio8"),  PINCTRL_PIN(9,  "gpio9"),
	PINCTRL_PIN(10, "gpio10"), PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"), PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"), PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"), PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
};

#define DA9188_VGPIO_NGPIOS	32

static unsigned int vgpio_target = DA9188_MB_TARGET_VGPIO;
module_param(vgpio_target, uint, 0444);
MODULE_PARM_DESC(vgpio_target,
		 "CPM MBA target ID for DA9188 virtual GPIOs (default 4)");

struct da9188_gpio_priv {
	struct device *dev;
	struct pinctrl_desc pdesc;
	struct pinctrl_dev *pctl;
	struct gpio_chip gc;
	struct da9188_mbox mbox;
	u32 dst_channel;
	u8 mb_target;
	bool is_vgpio;
};

static int da9188_gpio_mb_cmd(struct da9188_gpio_priv *priv, u8 cmd,
			      unsigned int pin, u32 val, u32 *out)
{
	struct da9188_mbox_data req = { .data = { val, 0 } };
	struct da9188_mbox_data resp;
	int ret;

	ret = da9188_mbox_xfer(priv->dev, &priv->mbox, priv->dst_channel,
			       priv->mb_target, cmd, pin,
			       &req, out ? &resp : NULL);
	if (!ret && out)
		*out = resp.data[0];

	return ret;
}

static int da9188_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *config)
{
	struct da9188_gpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg = 0, raw = 0;
	int ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_OUTPUT_DRIVE,
					 pin, 0, &raw);
		if (ret)
			return ret;
		if ((param == PIN_CONFIG_DRIVE_OPEN_DRAIN) !=
		    (raw == DA9188_GPIO_DRIVE_OD))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_DIRECTION,
					 pin, 0, &raw);
		if (ret)
			return ret;
		if (raw != DA9188_GPIO_DIR_OUT)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_LEVEL:
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_DIRECTION,
					 pin, 0, &raw);
		if (ret)
			return ret;
		if (raw == DA9188_GPIO_DIR_IN)
			return -EINVAL;
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_VALUE,
					 pin, 0, &arg);
		if (ret)
			return ret;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_DRIVE_STRENGTH,
					 pin, 0, &raw);
		if (ret)
			return ret;
		if (raw >= ARRAY_SIZE(da9188_drv_ma_table))
			return -EINVAL;
		arg = da9188_drv_ma_table[raw];
		break;
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_PULL_CONFIG,
					 pin, 0, &raw);
		if (ret)
			return ret;
		if (param == PIN_CONFIG_BIAS_DISABLE) {
			if (raw != DA9188_GPIO_PULL_OFF)
				return -EINVAL;
			arg = 1;
		} else if (param == PIN_CONFIG_BIAS_PULL_UP) {
			if (raw == DA9188_GPIO_PULL_UP_20K)
				arg = 20000;
			else if (raw == DA9188_GPIO_PULL_UP_800K)
				arg = 800000;
			else
				return -EINVAL;
		} else {
			if (raw == DA9188_GPIO_PULL_DN_20K)
				arg = 20000;
			else if (raw == DA9188_GPIO_PULL_DN_800K)
				arg = 800000;
			else
				return -EINVAL;
		}
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int da9188_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct da9188_gpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);
		u32 val = 0;
		u8 cmd;
		int ret;

		switch (param) {
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			cmd = DA9188_GPIO_CMD_SET_OUTPUT_DRIVE;
			val = DA9188_GPIO_DRIVE_OD;
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			cmd = DA9188_GPIO_CMD_SET_OUTPUT_DRIVE;
			val = DA9188_GPIO_DRIVE_PP;
			break;
		case PIN_CONFIG_LEVEL:
			ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_SET_DIRECTION,
						 pin, DA9188_GPIO_DIR_OUT, NULL);
			if (ret)
				return ret;
			cmd = DA9188_GPIO_CMD_SET_VALUE;
			val = !!arg;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			cmd = DA9188_GPIO_CMD_SET_DIRECTION;
			val = arg ? DA9188_GPIO_DIR_OUT : DA9188_GPIO_DIR_IN;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH: {
			unsigned int idx;

			cmd = DA9188_GPIO_CMD_SET_DRIVE_STRENGTH;
			for (idx = 0; idx < ARRAY_SIZE(da9188_drv_ma_table); idx++) {
				if (da9188_drv_ma_table[idx] == arg) {
					val = idx;
					break;
				}
			}
			if (idx == ARRAY_SIZE(da9188_drv_ma_table))
				return -ENOTSUPP;
			break;
		}
		case PIN_CONFIG_BIAS_DISABLE:
			cmd = DA9188_GPIO_CMD_SET_PULL_CONFIG;
			val = DA9188_GPIO_PULL_OFF;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			cmd = DA9188_GPIO_CMD_SET_PULL_CONFIG;
			if (arg == 20000)
				val = DA9188_GPIO_PULL_UP_20K;
			else if (arg == 800000)
				val = DA9188_GPIO_PULL_UP_800K;
			else
				return -ENOTSUPP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			cmd = DA9188_GPIO_CMD_SET_PULL_CONFIG;
			if (arg == 20000)
				val = DA9188_GPIO_PULL_DN_20K;
			else if (arg == 800000)
				val = DA9188_GPIO_PULL_DN_800K;
			else
				return -ENOTSUPP;
			break;
		default:
			return -ENOTSUPP;
		}

		ret = da9188_gpio_mb_cmd(priv, cmd, pin, val, NULL);
		if (ret)
			return ret;
	}

	return 0;
}

static int da9188_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *da9188_pctl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int selector)
{
	return NULL;
}

static int da9188_pinmux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *da9188_pinmux_get_func_name(struct pinctrl_dev *pctldev,
					       unsigned int selector)
{
	return NULL;
}

static int da9188_pinmux_get_func_groups(struct pinctrl_dev *pctldev,
					 unsigned int selector,
					 const char * const **groups,
					 unsigned int *num_groups)
{
	return -ENOTSUPP;
}

static int da9188_pinmux_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int func_selector,
				 unsigned int group_selector)
{
	return -ENOTSUPP;
}

static int da9188_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
					    struct pinctrl_gpio_range *range,
					    unsigned int offset, bool input)
{
	unsigned long cfg = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, !input);

	return da9188_pinconf_set(pctldev, offset, &cfg, 1);
}

static const struct pinctrl_ops da9188_pinctrl_ops = {
	.get_groups_count = da9188_pctl_get_groups_count,
	.get_group_name = da9188_pctl_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinmux_ops da9188_pinmux_ops = {
	.get_functions_count = da9188_pinmux_get_funcs_count,
	.get_function_name = da9188_pinmux_get_func_name,
	.get_function_groups = da9188_pinmux_get_func_groups,
	.set_mux = da9188_pinmux_set_mux,
	.gpio_set_direction = da9188_pinmux_gpio_set_direction,
};

static const struct pinconf_ops da9188_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = da9188_pinconf_get,
	.pin_config_set = da9188_pinconf_set,
};

static int da9188_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct da9188_gpio_priv *priv = gpiochip_get_data(chip);
	u32 val = 0;
	int ret;

	ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_VALUE, offset, 0, &val);
	return ret ? ret : !!val;
}

static int da9188_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct da9188_gpio_priv *priv = gpiochip_get_data(chip);

	return da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_SET_VALUE,
				  offset, !!value, NULL);
}

static int da9188_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct da9188_gpio_priv *priv = gpiochip_get_data(chip);
	u32 val = 0;
	int ret;

	ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_GET_DIRECTION,
				 offset, 0, &val);
	if (ret)
		return ret;

	return (val == DA9188_GPIO_DIR_OUT) ? GPIO_LINE_DIRECTION_OUT :
					      GPIO_LINE_DIRECTION_IN;
}

static int da9188_vgpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct da9188_gpio_priv *priv = gpiochip_get_data(chip);

	return da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_SET_DIRECTION,
				  offset, DA9188_GPIO_DIR_IN, NULL);
}

static int da9188_vgpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct da9188_gpio_priv *priv = gpiochip_get_data(chip);
	int ret;

	ret = da9188_gpio_mb_cmd(priv, DA9188_GPIO_CMD_SET_DIRECTION,
				 offset, DA9188_GPIO_DIR_OUT, NULL);
	if (ret)
		return ret;

	return da9188_gpio_set(chip, offset, value);
}

static int da9188_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	int ret;

	ret = pinctrl_gpio_direction_output(chip, offset);
	if (ret)
		return ret;

	return da9188_gpio_set(chip, offset, value);
}

static int da9188_gpio_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct device *dev = &pdev->dev;
	struct da9188_gpio_priv *priv;
	u32 ngpios;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->is_vgpio = id && id->driver_data;
	priv->mb_target = priv->is_vgpio ? (u8)vgpio_target :
					   DA9188_MB_TARGET_GPIO;
	priv->dst_channel = CPM_COMMON_PMIC_SERVICE;
	of_property_read_u32(dev->of_node, "mba-dest-channel", &priv->dst_channel);
	platform_set_drvdata(pdev, priv);

	ret = da9188_devm_mbox_request(dev, &priv->mbox);
	if (ret)
		return ret;

	priv->gc.label = pdev->name;
	priv->gc.parent = dev;
	priv->gc.owner = THIS_MODULE;
	priv->gc.get_direction = da9188_gpio_get_direction;
	priv->gc.get = da9188_gpio_get;
	priv->gc.set = da9188_gpio_set;
	priv->gc.base = -1;
	priv->gc.can_sleep = true;
	priv->gc.fwnode = dev_fwnode(dev);

	if (priv->is_vgpio) {
		ngpios = DA9188_VGPIO_NGPIOS;
		of_property_read_u32(dev->of_node, "ngpios", &ngpios);
		if (!ngpios || ngpios > DA9188_VGPIO_NGPIOS)
			ngpios = DA9188_VGPIO_NGPIOS;

		priv->gc.direction_input = da9188_vgpio_direction_input;
		priv->gc.direction_output = da9188_vgpio_direction_output;
		priv->gc.ngpio = ngpios;
		return devm_gpiochip_add_data(dev, &priv->gc, priv);
	}

	priv->pdesc.name = dev_name(dev);
	priv->pdesc.pins = da9188_pins;
	priv->pdesc.npins = ARRAY_SIZE(da9188_pins);
	priv->pdesc.pctlops = &da9188_pinctrl_ops;
	priv->pdesc.pmxops = &da9188_pinmux_ops;
	priv->pdesc.confops = &da9188_pinconf_ops;
	priv->pdesc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &priv->pdesc, priv, &priv->pctl);
	if (ret)
		return ret;

	ret = pinctrl_enable(priv->pctl);
	if (ret)
		return ret;

	priv->gc.request = gpiochip_generic_request;
	priv->gc.free = gpiochip_generic_free;
	priv->gc.direction_input = pinctrl_gpio_direction_input;
	priv->gc.direction_output = da9188_gpio_direction_output;
	priv->gc.set_config = gpiochip_generic_config;
	priv->gc.ngpio = ARRAY_SIZE(da9188_pins);

	return devm_gpiochip_add_data(dev, &priv->gc, priv);
}

static const struct platform_device_id da9188_gpio_id_table[] = {
	{ "da9188-gpio", 0 },
	{ "da9188-vgpio", 1 },
	{}
};
MODULE_DEVICE_TABLE(platform, da9188_gpio_id_table);

static struct platform_driver da9188_gpio_driver = {
	.driver = {
		.name = "da9188-gpio",
	},
	.probe = da9188_gpio_probe,
	.id_table = da9188_gpio_id_table,
};
module_platform_driver(da9188_gpio_driver);

MODULE_DESCRIPTION("DA9188 / DA9189 PMIC GPIO driver");
MODULE_LICENSE("GPL");
