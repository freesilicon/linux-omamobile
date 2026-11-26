// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core pin controller and GPIO driver for Google Tensor SoCs.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/soc/google/lga-aoc.h>
#include <linux/spinlock.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "pinctrl-google.h"

#define GOOGLE_RPM_AUTOSUSPEND_MS	10

struct google_pinctrl_irq_map {
	struct google_pinctrl *gctl;
	int pad;
	unsigned int virq;
	bool shared_gia;
	bool client_enabled;
	bool wakeup;
};

struct google_pinctrl {
	struct device *dev;
	const struct google_pinctrl_soc_info *info;
	void __iomem *csr_base;
	void __iomem *secondary_csr_base;
	u16 *secondary_pad_idx;

	raw_spinlock_t lock;

	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl;
	struct gpio_chip chip;

	struct device *pad_function_pd;
	struct device *csr_access_pd;
	struct device_link *csr_link;
	struct clk_bulk_data *clks;
	int nr_clks;

	struct google_pinctrl_irq_map *irq_map;
	int num_irqs;
	bool rpm_capable;
	bool rpm_ref_held;
	bool wakeup_capable;
	bool suspended;
	bool aoc_ssr_active;
};

static DEFINE_MUTEX(google_aoc_ssr_lock);
static struct google_pinctrl *google_aoc_pinctrl;

static bool google_is_shared_reg(const struct google_pinctrl *gctl,
				 unsigned int pad, unsigned int reg_off)
{
	return gctl->secondary_csr_base &&
	       gctl->secondary_pad_idx[pad] > 0 &&
	       reg_off > GOOGLE_REG_DMUX;
}

static void __iomem *google_pad_reg(const struct google_pinctrl *gctl,
				    unsigned int pad, unsigned int reg_off)
{
	size_t stride = gctl->info->pin_stride;

	if (google_is_shared_reg(gctl, pad, reg_off)) {
		unsigned int sec_idx = gctl->secondary_pad_idx[pad] - 1;

		return gctl->secondary_csr_base + stride * sec_idx + reg_off;
	}

	return gctl->csr_base + stride * pad + reg_off;
}

static u32 google_pad_read(const struct google_pinctrl *gctl,
			   unsigned int pad, unsigned int reg_off)
{
	if (READ_ONCE(gctl->aoc_ssr_active) &&
	    !google_is_shared_reg(gctl, pad, reg_off))
		return 0;

	return readl(google_pad_reg(gctl, pad, reg_off));
}

static void google_pad_write(const struct google_pinctrl *gctl,
			     unsigned int pad, unsigned int reg_off, u32 val)
{
	if (READ_ONCE(gctl->aoc_ssr_active) &&
	    !google_is_shared_reg(gctl, pad, reg_off))
		return;

	writel(val, google_pad_reg(gctl, pad, reg_off));
}

static int google_csr_get(struct google_pinctrl *gctl)
{
	if (!gctl->csr_access_pd)
		return 0;

	return pm_runtime_resume_and_get(gctl->dev);
}

static void google_csr_put(struct google_pinctrl *gctl)
{
	if (!gctl->csr_access_pd)
		return;

	pm_runtime_put_autosuspend(gctl->dev);
}

/* Pin controller ops */

static int google_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	return gctl->info->num_groups;
}

static const char *google_pctl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int selector)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= gctl->info->num_groups)
		return NULL;

	return gctl->info->groups[selector].name;
}

static int google_pctl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				      const unsigned int **pins,
				      unsigned int *num_pins)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= gctl->info->num_groups)
		return -EINVAL;

	*pins = gctl->info->groups[selector].pins;
	*num_pins = gctl->info->groups[selector].npins;
	return 0;
}

static const struct pinctrl_ops google_pinctrl_ops = {
	.get_groups_count = google_pctl_get_groups_count,
	.get_group_name = google_pctl_get_group_name,
	.get_group_pins = google_pctl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_group,
	.dt_free_map = pinctrl_utils_free_map,
};

/* Pinmux ops */

static bool google_pad_valid(struct google_pinctrl *gctl, unsigned int pad)
{
	return pad < gctl->info->num_groups &&
	       gpiochip_line_is_valid(&gctl->chip, pad);
}

static int google_pinmux_request(struct pinctrl_dev *pctldev, unsigned int offset)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	return google_pad_valid(gctl, offset) ? 0 : -EINVAL;
}

static int google_pinmux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	return gctl->info->num_funcs;
}

static const char *google_pinmux_get_func_name(struct pinctrl_dev *pctldev,
					       unsigned int selector)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= gctl->info->num_funcs)
		return NULL;

	return gctl->info->funcs[selector].name;
}

static int google_pinmux_get_func_groups(struct pinctrl_dev *pctldev,
					 unsigned int selector,
					 const char * const **groups,
					 unsigned int *const num_groups)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= gctl->info->num_funcs)
		return -EINVAL;

	*groups = gctl->info->funcs[selector].groups;
	*num_groups = gctl->info->funcs[selector].ngroups;
	return 0;
}

static int google_pinmux_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int func_selector,
				 unsigned int group_selector)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);
	const struct google_pingroup *g;
	unsigned long flags;
	unsigned int slot;
	int ret;

	if (!google_pad_valid(gctl, group_selector))
		return -EINVAL;

	g = &gctl->info->groups[group_selector];
	for (slot = 0; slot < g->nfuncs; slot++) {
		if (g->funcs[slot] == (int)func_selector)
			break;
	}
	if (slot == g->nfuncs)
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	google_pad_write(gctl, g->num, GOOGLE_REG_DMUX, BIT(slot));
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return 0;
}

static int google_pinmux_gpio_request_enable(struct pinctrl_dev *pctldev,
					     struct pinctrl_gpio_range *range,
					     unsigned int offset)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);
	const struct google_pingroup *g;
	int func;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	g = &gctl->info->groups[offset];
	if (gctl->secondary_csr_base && gctl->secondary_pad_idx[offset] > 0 &&
	    g->funcs[GOOGLE_GPIO_FUNC_SHARED] >= 0)
		func = g->funcs[GOOGLE_GPIO_FUNC_SHARED];
	else
		func = g->funcs[gctl->info->gpio_func];

	if (func < 0)
		return -EPERM;

	return google_pinmux_set_mux(pctldev, func, offset);
}

static const struct pinmux_ops google_pinmux_ops = {
	.request = google_pinmux_request,
	.get_functions_count = google_pinmux_get_funcs_count,
	.get_function_name = google_pinmux_get_func_name,
	.get_function_groups = google_pinmux_get_func_groups,
	.gpio_request_enable = google_pinmux_gpio_request_enable,
	.set_mux = google_pinmux_set_mux,
};

/* Pinconf ops */

static int google_pinconf_group_get(struct pinctrl_dev *pctldev,
				    unsigned int group, unsigned long *config)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 param_val, tx_val;
	u32 arg = 0;
	int ret;

	if (!google_pad_valid(gctl, group))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	param_val = google_pad_read(gctl, group, GOOGLE_REG_PARAM);
	tx_val = google_pad_read(gctl, group, GOOGLE_REG_TXDATA);

	google_csr_put(gctl);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (FIELD_GET(GOOGLE_TXDATA_PUPD_MASK, tx_val) != GOOGLE_PUPD_OPEN)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (FIELD_GET(GOOGLE_TXDATA_PUPD_MASK, tx_val) != GOOGLE_PUPD_PD)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (FIELD_GET(GOOGLE_TXDATA_PUPD_MASK, tx_val) != GOOGLE_PUPD_PU)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_OUTPUT_ENABLE:
		if (!(tx_val & GOOGLE_TXDATA_OE_BIT))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = FIELD_GET(GOOGLE_PARAM_DRV_MASK, param_val);
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (!(param_val & GOOGLE_PARAM_SLEW_BIT))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(param_val & GOOGLE_PARAM_IE_BIT))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		if (!(param_val & GOOGLE_PARAM_RXMODE_BIT))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_LEVEL:
		if (!(tx_val & GOOGLE_TXDATA_OE_BIT))
			return -EINVAL;
		arg = !!(tx_val & GOOGLE_TXDATA_DOUT_BIT);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int google_pinconf_group_set(struct pinctrl_dev *pctldev,
				    unsigned int group,
				    unsigned long *configs,
				    unsigned int num_configs)
{
	struct google_pinctrl *gctl = pinctrl_dev_get_drvdata(pctldev);
	u32 param_orig, tx_orig, param_val, tx_val;
	unsigned long flags;
	unsigned int i;
	int ret;

	if (!google_pad_valid(gctl, group))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);

	param_orig = google_pad_read(gctl, group, GOOGLE_REG_PARAM);
	tx_orig = google_pad_read(gctl, group, GOOGLE_REG_TXDATA);
	param_val = param_orig;
	tx_val = tx_orig;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			tx_val &= ~GOOGLE_TXDATA_PUPD_MASK;
			tx_val |= FIELD_PREP(GOOGLE_TXDATA_PUPD_MASK, GOOGLE_PUPD_OPEN);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
		case PIN_CONFIG_BIAS_PULL_UP: {
			u32 open_val = (tx_val & ~GOOGLE_TXDATA_PUPD_MASK) |
				       FIELD_PREP(GOOGLE_TXDATA_PUPD_MASK, GOOGLE_PUPD_OPEN);
			u32 bias = (param == PIN_CONFIG_BIAS_PULL_UP) ?
				   GOOGLE_PUPD_PU : GOOGLE_PUPD_PD;

			google_pad_write(gctl, group, GOOGLE_REG_TXDATA, open_val);
			udelay(GOOGLE_PUPD_SETTLE_US);
			tx_val = (tx_val & ~GOOGLE_TXDATA_PUPD_MASK) |
				 FIELD_PREP(GOOGLE_TXDATA_PUPD_MASK, bias);
			break;
		}
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			tx_val |= GOOGLE_TXDATA_OE_BIT;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			if (arg > FIELD_MAX(GOOGLE_PARAM_DRV_MASK)) {
				ret = -ENOTSUPP;
				goto unlock;
			}
			param_val &= ~GOOGLE_PARAM_DRV_MASK;
			param_val |= FIELD_PREP(GOOGLE_PARAM_DRV_MASK, arg);
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (arg)
				param_val |= GOOGLE_PARAM_SLEW_BIT;
			else
				param_val &= ~GOOGLE_PARAM_SLEW_BIT;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			if (arg)
				param_val |= GOOGLE_PARAM_IE_BIT;
			else
				param_val &= ~GOOGLE_PARAM_IE_BIT;
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			if (arg)
				param_val |= GOOGLE_PARAM_RXMODE_BIT;
			else
				param_val &= ~GOOGLE_PARAM_RXMODE_BIT;
			break;
		case PIN_CONFIG_LEVEL:
			tx_val |= GOOGLE_TXDATA_OE_BIT;
			if (arg)
				tx_val |= GOOGLE_TXDATA_DOUT_BIT;
			else
				tx_val &= ~GOOGLE_TXDATA_DOUT_BIT;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			if (arg)
				tx_val |= GOOGLE_TXDATA_OE_BIT;
			else
				tx_val &= ~GOOGLE_TXDATA_OE_BIT;
			break;
		case PIN_CONFIG_PERSIST_STATE:
			break;
		default:
			ret = -ENOTSUPP;
			goto unlock;
		}
	}

	if (param_val != param_orig)
		google_pad_write(gctl, group, GOOGLE_REG_PARAM, param_val);
	if (tx_val != tx_orig)
		google_pad_write(gctl, group, GOOGLE_REG_TXDATA, tx_val);

unlock:
	raw_spin_unlock_irqrestore(&gctl->lock, flags);
	google_csr_put(gctl);
	return ret;
}

static const struct pinconf_ops google_pinconf_ops = {
	.is_generic = true,
	.pin_config_group_get = google_pinconf_group_get,
	.pin_config_group_set = google_pinconf_group_set,
};

/* GPIO chip ops */

static int google_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;
	int ret;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	val = google_pad_read(gctl, offset, GOOGLE_REG_PARAM);
	val |= GOOGLE_PARAM_IE_BIT;
	google_pad_write(gctl, offset, GOOGLE_REG_PARAM, val);

	val = google_pad_read(gctl, offset, GOOGLE_REG_TXDATA);
	val &= ~GOOGLE_TXDATA_OE_BIT;
	google_pad_write(gctl, offset, GOOGLE_REG_TXDATA, val);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return 0;
}

static int google_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;
	int ret;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	val = google_pad_read(gctl, offset, GOOGLE_REG_TXDATA);
	val |= GOOGLE_TXDATA_OE_BIT;
	if (value)
		val |= GOOGLE_TXDATA_DOUT_BIT;
	else
		val &= ~GOOGLE_TXDATA_DOUT_BIT;
	google_pad_write(gctl, offset, GOOGLE_REG_TXDATA, val);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return 0;
}

static int google_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;
	int ret;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	val = google_pad_read(gctl, offset, GOOGLE_REG_TXDATA);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return (val & GOOGLE_TXDATA_OE_BIT) ? GPIO_LINE_DIRECTION_OUT :
					      GPIO_LINE_DIRECTION_IN;
}

static int google_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;
	int ret;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	val = google_pad_read(gctl, offset, GOOGLE_REG_RXDATA);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return !!(val & GOOGLE_RXDATA_PAD_BIT);
}

static int google_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);
	unsigned long flags;
	u32 val;
	int ret;

	if (!google_pad_valid(gctl, offset))
		return -EINVAL;

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	val = google_pad_read(gctl, offset, GOOGLE_REG_TXDATA);
	if (value)
		val |= GOOGLE_TXDATA_DOUT_BIT;
	else
		val &= ~GOOGLE_TXDATA_DOUT_BIT;
	google_pad_write(gctl, offset, GOOGLE_REG_TXDATA, val);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return 0;
}

static int google_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
				  unsigned long config)
{
	struct google_pinctrl *gctl = gpiochip_get_data(chip);

	return google_pinconf_group_set(gctl->pctl, offset, &config, 1);
}

static const struct gpio_chip google_gpio_chip_template = {
	.direction_input = google_gpio_direction_input,
	.direction_output = google_gpio_direction_output,
	.get_direction = google_gpio_get_direction,
	.get = google_gpio_get,
	.set = google_gpio_set,
	.set_config = google_gpio_set_config,
	.request = gpiochip_generic_request,
	.free = gpiochip_generic_free,
};

/* GPIO IRQ support */

static struct google_pinctrl_irq_map *google_find_irq_map(struct google_pinctrl *gctl,
							  unsigned int pad)
{
	int i;

	for (i = 0; i < gctl->num_irqs; i++) {
		if (gctl->irq_map[i].pad == (int)pad)
			return &gctl->irq_map[i];
	}

	return NULL;
}

static void google_gpio_irq_mask_pad(struct google_pinctrl *gctl,
				     unsigned int pad, bool unmask)
{
	unsigned long flags;
	u32 rxdata, trig;

	if (!google_pad_valid(gctl, pad))
		return;

	if (google_csr_get(gctl) < 0)
		return;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	rxdata = google_pad_read(gctl, pad, GOOGLE_REG_RXDATA);
	trig = FIELD_GET(GOOGLE_RXDATA_TRIG_MASK, rxdata);
	if (unmask && (trig == GOOGLE_TRIG_LEVEL_HIGH || trig == GOOGLE_TRIG_LEVEL_LOW)) {
		google_pad_write(gctl, pad, GOOGLE_REG_ISR, GOOGLE_IXR_IRQ_BIT);
		google_pad_write(gctl, pad, GOOGLE_REG_ISROVF, GOOGLE_IXR_IRQ_BIT);
	}
	google_pad_write(gctl, pad, GOOGLE_REG_IMR, unmask ? GOOGLE_IXR_IRQ_BIT : 0);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
}

static void google_gpio_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	struct google_pinctrl_irq_map *map = google_find_irq_map(gctl, d->hwirq);
	unsigned long flags;

	gpiochip_enable_irq(gc, d->hwirq);

	if (google_csr_get(gctl) < 0)
		return;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	google_pad_write(gctl, d->hwirq, GOOGLE_REG_IER, GOOGLE_IXR_IRQ_BIT);
	google_pad_write(gctl, d->hwirq, GOOGLE_REG_IMR, GOOGLE_IXR_IRQ_BIT);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	if (map && map->shared_gia && !map->client_enabled) {
		enable_irq(map->virq);
		map->client_enabled = true;
	}

	google_csr_put(gctl);
}

static void google_gpio_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	struct google_pinctrl_irq_map *map = google_find_irq_map(gctl, d->hwirq);
	unsigned long flags;

	if (google_csr_get(gctl) < 0)
		goto out_disable;

	if (!map || !map->shared_gia) {
		raw_spin_lock_irqsave(&gctl->lock, flags);
		google_pad_write(gctl, d->hwirq, GOOGLE_REG_IER, 0);
		google_pad_write(gctl, d->hwirq, GOOGLE_REG_IMR, 0);
		raw_spin_unlock_irqrestore(&gctl->lock, flags);
	} else if (map->client_enabled) {
		disable_irq_nosync(map->virq);
		map->client_enabled = false;
	}

	google_csr_put(gctl);

out_disable:
	gpiochip_disable_irq(gc, d->hwirq);
}

static void google_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);

	google_gpio_irq_mask_pad(gctl, d->hwirq, false);
}

static void google_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);

	google_gpio_irq_mask_pad(gctl, d->hwirq, true);
}

static void google_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	google_pad_write(gctl, d->hwirq, GOOGLE_REG_ISR, GOOGLE_IXR_IRQ_BIT);
	google_pad_write(gctl, d->hwirq, GOOGLE_REG_ISROVF, GOOGLE_IXR_IRQ_BIT);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);
}

static void google_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);

	google_csr_get(gctl);
}

static void google_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);

	google_csr_put(gctl);
}

static int google_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	unsigned long flags;
	u32 trig, rxdata;
	int ret;

	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		trig = GOOGLE_TRIG_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trig = GOOGLE_TRIG_EDGE_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		trig = GOOGLE_TRIG_EDGE_BOTH;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trig = GOOGLE_TRIG_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trig = GOOGLE_TRIG_LEVEL_LOW;
		break;
	default:
		return -EINVAL;
	}

	if (flow_type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	ret = google_csr_get(gctl);
	if (ret < 0)
		return ret;

	raw_spin_lock_irqsave(&gctl->lock, flags);
	rxdata = google_pad_read(gctl, d->hwirq, GOOGLE_REG_RXDATA);
	rxdata &= ~GOOGLE_RXDATA_TRIG_MASK;
	rxdata |= FIELD_PREP(GOOGLE_RXDATA_TRIG_MASK, trig);
	google_pad_write(gctl, d->hwirq, GOOGLE_REG_RXDATA, rxdata);
	raw_spin_unlock_irqrestore(&gctl->lock, flags);

	google_csr_put(gctl);
	return 0;
}

static int google_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	struct google_pinctrl_irq_map *map;

	if (!gctl->wakeup_capable)
		return -EINVAL;

	map = google_find_irq_map(gctl, d->hwirq);
	if (!map)
		return -EINVAL;

	map->wakeup = !!on;
	return 0;
}

static void google_gpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct google_pinctrl_irq_map *map = irq_desc_get_handler_data(desc);
	struct google_pinctrl *gctl;
	u32 rxdata, trig, isr, isr_ovf;
	bool is_level;

	if (!map)
		return;

	gctl = map->gctl;
	chained_irq_enter(chip, desc);

	rxdata = google_pad_read(gctl, map->pad, GOOGLE_REG_RXDATA);
	trig = FIELD_GET(GOOGLE_RXDATA_TRIG_MASK, rxdata);
	is_level = (trig == GOOGLE_TRIG_LEVEL_HIGH || trig == GOOGLE_TRIG_LEVEL_LOW);

	isr = google_pad_read(gctl, map->pad, GOOGLE_REG_ISR);
	isr_ovf = google_pad_read(gctl, map->pad, GOOGLE_REG_ISROVF);

	if (is_level) {
		generic_handle_domain_irq(gctl->chip.irq.domain, map->pad);
		if (isr)
			google_pad_write(gctl, map->pad, GOOGLE_REG_ISR, isr);
		if (isr_ovf)
			google_pad_write(gctl, map->pad, GOOGLE_REG_ISROVF, isr_ovf);
	} else {
		if (isr)
			google_pad_write(gctl, map->pad, GOOGLE_REG_ISR, isr);
		if (isr_ovf)
			google_pad_write(gctl, map->pad, GOOGLE_REG_ISROVF, isr_ovf);
		generic_handle_domain_irq(gctl->chip.irq.domain, map->pad);
	}

	chained_irq_exit(chip, desc);
}

static unsigned int google_gpio_irq_startup(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	struct google_pinctrl_irq_map *map = google_find_irq_map(gctl, d->hwirq);

	if (!map)
		return 0;

	irq_set_chained_handler_and_data(map->virq, google_gpio_irq_handler, map);
	map->client_enabled = true;
	google_gpio_irq_enable(d);
	return 0;
}

static void google_gpio_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct google_pinctrl *gctl = gpiochip_get_data(gc);
	struct google_pinctrl_irq_map *map = google_find_irq_map(gctl, d->hwirq);

	google_gpio_irq_disable(d);
	if (map)
		irq_set_chained_handler_and_data(map->virq, NULL, NULL);
}

static const struct irq_chip google_gpio_irqchip = {
	.name = "google-pinctrl",
	.irq_startup = google_gpio_irq_startup,
	.irq_shutdown = google_gpio_irq_shutdown,
	.irq_enable = google_gpio_irq_enable,
	.irq_disable = google_gpio_irq_disable,
	.irq_ack = google_gpio_irq_ack,
	.irq_mask = google_gpio_irq_mask,
	.irq_unmask = google_gpio_irq_unmask,
	.irq_bus_lock = google_gpio_irq_bus_lock,
	.irq_bus_sync_unlock = google_gpio_irq_bus_sync_unlock,
	.irq_set_type = google_gpio_irq_set_type,
	.irq_set_wake = google_gpio_irq_set_wake,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_ENABLE_WAKEUP_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/* Probe and PM setup */

static void google_pinctrl_detach_pds(void *data)
{
	struct google_pinctrl *gctl = data;

	if (gctl->csr_link)
		device_link_del(gctl->csr_link);

	if (gctl->csr_access_pd)
		dev_pm_domain_detach(gctl->csr_access_pd, true);

	if (gctl->pad_function_pd) {
		if (!READ_ONCE(gctl->aoc_ssr_active))
			pm_runtime_put_sync(gctl->pad_function_pd);
		dev_pm_domain_detach(gctl->pad_function_pd, true);
	}
}

static void google_pinctrl_unregister_aoc_ssr(void *data)
{
	struct google_pinctrl *gctl = data;

	mutex_lock(&google_aoc_ssr_lock);
	if (google_aoc_pinctrl == gctl)
		google_aoc_pinctrl = NULL;
	mutex_unlock(&google_aoc_ssr_lock);
}

int google_pinctrl_aoc_ssr_request(bool active)
{
	struct google_pinctrl *gctl;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&google_aoc_ssr_lock);
	gctl = google_aoc_pinctrl;
	if (!gctl || !gctl->pad_function_pd) {
		mutex_unlock(&google_aoc_ssr_lock);
		return 0;
	}

	if (active) {
		if (!gctl->aoc_ssr_active) {
			raw_spin_lock_irqsave(&gctl->lock, flags);
			WRITE_ONCE(gctl->aoc_ssr_active, true);
			raw_spin_unlock_irqrestore(&gctl->lock, flags);

			ret = pm_runtime_put_sync(gctl->pad_function_pd);
			if (ret < 0) {
				dev_warn(gctl->dev,
					 "failed to release AoC pad_function_pd for SSR: %d\n",
					 ret);
				raw_spin_lock_irqsave(&gctl->lock, flags);
				WRITE_ONCE(gctl->aoc_ssr_active, false);
				raw_spin_unlock_irqrestore(&gctl->lock, flags);
			} else {
				ret = 0;
			}
		}
	} else {
		if (gctl->aoc_ssr_active) {
			ret = pm_runtime_resume_and_get(gctl->pad_function_pd);
			if (ret < 0) {
				dev_warn(gctl->dev,
					 "failed to restore AoC pad_function_pd after SSR: %d\n",
					 ret);
			} else {
				raw_spin_lock_irqsave(&gctl->lock, flags);
				WRITE_ONCE(gctl->aoc_ssr_active, false);
				raw_spin_unlock_irqrestore(&gctl->lock, flags);
				ret = 0;
			}
		}
	}
	mutex_unlock(&google_aoc_ssr_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(google_pinctrl_aoc_ssr_request);

static int google_pinctrl_attach_pds(struct google_pinctrl *gctl)
{
	int ret;

	dev_pm_domain_detach(gctl->dev, false);

	gctl->pad_function_pd = dev_pm_domain_attach_by_name(gctl->dev, "pad_function_pd");
	if (IS_ERR(gctl->pad_function_pd)) {
		ret = PTR_ERR(gctl->pad_function_pd);
		gctl->pad_function_pd = NULL;
		return ret;
	}

	if (gctl->pad_function_pd) {
		ret = pm_runtime_resume_and_get(gctl->pad_function_pd);
		if (ret < 0) {
			dev_pm_domain_detach(gctl->pad_function_pd, true);
			gctl->pad_function_pd = NULL;
			return ret;
		}
	}

	gctl->csr_access_pd = dev_pm_domain_attach_by_name(gctl->dev, "csr_access_pd");
	if (IS_ERR(gctl->csr_access_pd)) {
		ret = PTR_ERR(gctl->csr_access_pd);
		gctl->csr_access_pd = NULL;
		if (gctl->pad_function_pd) {
			pm_runtime_put_sync(gctl->pad_function_pd);
			dev_pm_domain_detach(gctl->pad_function_pd, true);
			gctl->pad_function_pd = NULL;
		}
		return ret;
	}

	if (gctl->csr_access_pd) {
		gctl->csr_link = device_link_add(gctl->dev, gctl->csr_access_pd,
						 DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!gctl->csr_link) {
			google_pinctrl_detach_pds(gctl);
			return -ENOMEM;
		}
	}

	return devm_add_action_or_reset(gctl->dev, google_pinctrl_detach_pds, gctl);
}

static void google_pinctrl_rpm_drop(void *data)
{
	struct google_pinctrl *gctl = data;

	if (gctl->rpm_ref_held) {
		pm_runtime_put_noidle(gctl->dev);
		gctl->rpm_ref_held = false;
	}
}

static void google_pinctrl_clk_unprepare(void *data)
{
	struct google_pinctrl *gctl = data;

	clk_bulk_disable_unprepare(gctl->nr_clks, gctl->clks);
}

static int google_pinctrl_parse_shared_pads(struct platform_device *pdev,
					    struct google_pinctrl *gctl)
{
	struct resource *res;
	int count, i, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return 0;

	gctl->secondary_csr_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gctl->secondary_csr_base))
		return PTR_ERR(gctl->secondary_csr_base);

	gctl->secondary_pad_idx = devm_kcalloc(&pdev->dev, gctl->info->num_pins,
					       sizeof(*gctl->secondary_pad_idx),
					       GFP_KERNEL);
	if (!gctl->secondary_pad_idx)
		return -ENOMEM;

	count = of_property_count_u32_elems(pdev->dev.of_node, "shared-pads");
	if (count < 0)
		return count;

	u32 *pads __free(kfree) = kcalloc(count, sizeof(*pads), GFP_KERNEL);
	if (!pads)
		return -ENOMEM;

	ret = of_property_read_u32_array(pdev->dev.of_node, "shared-pads", pads, count);
	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		if (pads[i] >= gctl->info->num_pins)
			return -EINVAL;
		gctl->secondary_pad_idx[pads[i]] = i + 1;
	}

	return 0;
}

static int google_pinctrl_parse_irqs(struct platform_device *pdev,
				     struct google_pinctrl *gctl)
{
	struct device_node *np = pdev->dev.of_node;
	struct of_phandle_args oirq;
	int count, i, ret;

	count = platform_irq_count(pdev);
	if (count < 0)
		return count;
	if (!count)
		return 0;

	gctl->num_irqs = count;
	gctl->irq_map = devm_kcalloc(&pdev->dev, count, sizeof(*gctl->irq_map),
				     GFP_KERNEL);
	if (!gctl->irq_map)
		return -ENOMEM;

	u32 *pad_map __free(kfree) = kcalloc(count, sizeof(*pad_map), GFP_KERNEL);
	u32 *shared_gia __free(kfree) = kcalloc(count, sizeof(*shared_gia), GFP_KERNEL);
	if (!pad_map || !shared_gia)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "irq-pad-mapping", pad_map, count);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "missing irq-pad-mapping\n");

	/* Optional: controllers routed directly to GIC omit gia-routed-shared-irq */
	of_property_read_u32_array(np, "gia-routed-shared-irq", shared_gia, count);
	gctl->wakeup_capable = of_property_read_bool(np, "sswrp-wakeup-capable");

	for (i = 0; i < count; i++) {
		unsigned int virq;

		if (of_irq_parse_one(np, i, &oirq))
			return -EINVAL;

		virq = irq_create_of_mapping(&oirq);
		if (!virq)
			return -EINVAL;

		gctl->irq_map[i].gctl = gctl;
		gctl->irq_map[i].pad = pad_map[i];
		gctl->irq_map[i].virq = virq;
		gctl->irq_map[i].shared_gia = !!shared_gia[i];
		irq_set_status_flags(virq, IRQ_DISABLE_UNLAZY);
	}

	return 0;
}

static void google_gpio_irq_mask_all(struct google_pinctrl *gctl)
{
	int i;

	for (i = 0; i < gctl->num_irqs; i++) {
		unsigned int pad = gctl->irq_map[i].pad;

		if (google_pad_valid(gctl, pad))
			google_gpio_irq_mask_pad(gctl, pad, false);
	}
}

int google_pinctrl_probe(struct platform_device *pdev,
			 const struct google_pinctrl_soc_info *soc_info)
{
	struct device *dev = &pdev->dev;
	struct google_pinctrl *gctl;
	struct gpio_irq_chip *girq;
	int ret;

	if (!soc_info)
		return -EINVAL;

	gctl = devm_kzalloc(dev, sizeof(*gctl), GFP_KERNEL);
	if (!gctl)
		return -ENOMEM;

	gctl->dev = dev;
	gctl->info = soc_info;
	raw_spin_lock_init(&gctl->lock);
	platform_set_drvdata(pdev, gctl);

	gctl->csr_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gctl->csr_base))
		return PTR_ERR(gctl->csr_base);

	ret = google_pinctrl_parse_shared_pads(pdev, gctl);
	if (ret)
		return ret;

	ret = google_pinctrl_attach_pds(gctl);
	if (ret)
		return ret;

	gctl->rpm_capable = of_property_read_bool(dev->of_node, "runtime-pm-capable");
	pm_runtime_set_autosuspend_delay(dev, GOOGLE_RPM_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;
	pm_runtime_get_noresume(dev);
	gctl->rpm_ref_held = true;
	ret = devm_pm_runtime_enable(dev);
	if (ret) {
		gctl->rpm_ref_held = false;
		pm_runtime_put_noidle(dev);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, google_pinctrl_rpm_drop, gctl);
	if (ret)
		return ret;

	ret = devm_clk_bulk_get_all(dev, &gctl->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	gctl->nr_clks = ret;

	ret = clk_bulk_prepare_enable(gctl->nr_clks, gctl->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");
	ret = devm_add_action_or_reset(dev, google_pinctrl_clk_unprepare, gctl);
	if (ret)
		return ret;

	ret = google_pinctrl_parse_irqs(pdev, gctl);
	if (ret)
		return ret;

	gctl->desc.name = dev_name(dev);
	gctl->desc.pins = soc_info->pins;
	gctl->desc.npins = soc_info->num_pins;
	gctl->desc.pctlops = &google_pinctrl_ops;
	gctl->desc.confops = &google_pinconf_ops;
	gctl->desc.pmxops = &google_pinmux_ops;
	gctl->desc.owner = THIS_MODULE;

	gctl->pctl = devm_pinctrl_register(dev, &gctl->desc, gctl);
	if (IS_ERR(gctl->pctl))
		return dev_err_probe(dev, PTR_ERR(gctl->pctl),
				     "failed to register pinctrl\n");

	gctl->chip = google_gpio_chip_template;
	gctl->chip.label = soc_info->label;
	gctl->chip.parent = dev;
	gctl->chip.base = -1;
	gctl->chip.ngpio = soc_info->num_pins;
	gctl->chip.owner = THIS_MODULE;

	girq = &gctl->chip.irq;
	gpio_irq_chip_set_chip(girq, &google_gpio_irqchip);
	girq->handler = handle_bad_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->fwnode = dev_fwnode(dev);

	ret = devm_gpiochip_add_data(dev, &gctl->chip, gctl);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpiochip\n");

	google_gpio_irq_mask_all(gctl);

	if (gctl->rpm_capable) {
		gctl->rpm_ref_held = false;
		pm_runtime_put_autosuspend(dev);
	}

	if (gctl->pad_function_pd &&
	    of_device_is_compatible(dev->of_node, "google,lga-aoc-pinctrl")) {
		mutex_lock(&google_aoc_ssr_lock);
		google_aoc_pinctrl = gctl;
		mutex_unlock(&google_aoc_ssr_lock);

		ret = devm_add_action_or_reset(dev,
					       google_pinctrl_unregister_aoc_ssr,
					       gctl);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(google_pinctrl_probe);

static void google_wake_irq_mask_update(struct google_pinctrl *gctl, bool unmask)
{
	int i;

	if (!gctl->wakeup_capable)
		return;

	for (i = 0; i < gctl->num_irqs; i++) {
		if (!gctl->irq_map[i].wakeup &&
		    google_pad_valid(gctl, gctl->irq_map[i].pad))
			google_gpio_irq_mask_pad(gctl, gctl->irq_map[i].pad, unmask);
	}
}

static int google_pinctrl_suspend_noirq(struct device *dev)
{
	struct google_pinctrl *gctl = dev_get_drvdata(dev);

	gctl->suspended = true;
	google_wake_irq_mask_update(gctl, false);

	if (gctl->pad_function_pd && !READ_ONCE(gctl->aoc_ssr_active))
		pm_runtime_put_sync(gctl->pad_function_pd);

	return pm_runtime_force_suspend(dev);
}

static int google_pinctrl_resume_noirq(struct device *dev)
{
	struct google_pinctrl *gctl = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	if (gctl->pad_function_pd && !READ_ONCE(gctl->aoc_ssr_active))
		pm_runtime_resume_and_get(gctl->pad_function_pd);

	google_wake_irq_mask_update(gctl, true);
	gctl->suspended = false;
	return 0;
}

static int google_pinctrl_runtime_suspend(struct device *dev)
{
	struct google_pinctrl *gctl = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(gctl->nr_clks, gctl->clks);
	return 0;
}

static int google_pinctrl_runtime_resume(struct device *dev)
{
	struct google_pinctrl *gctl = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(gctl->nr_clks, gctl->clks);
}

const struct dev_pm_ops google_pinctrl_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(google_pinctrl_suspend_noirq,
				  google_pinctrl_resume_noirq)
	RUNTIME_PM_OPS(google_pinctrl_runtime_suspend,
		       google_pinctrl_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(google_pinctrl_pm_ops);

MODULE_DESCRIPTION("Core Google Tensor pin controller and GPIO driver");
MODULE_LICENSE("GPL");
