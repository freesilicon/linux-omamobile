/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Core definitions for Google SoC pin controller and GPIO driver.
 */

#ifndef _PINCTRL_GOOGLE_H_
#define _PINCTRL_GOOGLE_H_

#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

#define GOOGLE_PINMUX_NONE		(-1)
#define google_pinmux_none		GOOGLE_PINMUX_NONE

#define GOOGLE_MAX_FUNCS_PER_PIN	9
#define GOOGLE_GPIO_FUNC_DEFAULT	0
#define GOOGLE_GPIO_FUNC_SHARED		6

/* Standard register offsets per pad block (LGA uses 0x1000 stride) */
#define GOOGLE_REG_PARAM		0x00
#define GOOGLE_REG_DMUX			0x04
#define GOOGLE_REG_TXDATA		0x08
#define GOOGLE_REG_RXGATE		0x0c
#define GOOGLE_REG_RXDATA		0x10
#define GOOGLE_REG_ISR			0x14
#define GOOGLE_REG_ISROVF		0x18
#define GOOGLE_REG_IER			0x1c
#define GOOGLE_REG_IMR			0x20
#define GOOGLE_REG_ITR			0x24

/* PARAM register fields (standard layout) */
#define GOOGLE_PARAM_DRV_MASK		GENMASK(3, 0)
#define GOOGLE_PARAM_IE_BIT		BIT(8)
#define GOOGLE_PARAM_RXMODE_BIT		BIT(12)
#define GOOGLE_PARAM_SLEW_BIT		BIT(16)
#define GOOGLE_PARAM_DEBOUNCE_BIT	BIT(20)

/* TXDATA register fields */
#define GOOGLE_TXDATA_PUPD_MASK		GENMASK(1, 0)
#define GOOGLE_PUPD_OPEN		0
#define GOOGLE_PUPD_PD			1
#define GOOGLE_PUPD_PU			2
#define GOOGLE_PUPD_SPU			3
#define GOOGLE_TXDATA_DOUT_BIT		BIT(2)
#define GOOGLE_TXDATA_OE_BIT		BIT(3)
#define GOOGLE_PUPD_SETTLE_US		10

/* RXDATA register fields */
#define GOOGLE_RXDATA_PAD_BIT		BIT(0)
#define GOOGLE_RXDATA_TRIG_MASK		GENMASK(3, 1)
#define GOOGLE_TRIG_NONE		0
#define GOOGLE_TRIG_EDGE_RISING		1
#define GOOGLE_TRIG_EDGE_FALLING	2
#define GOOGLE_TRIG_EDGE_BOTH		3
#define GOOGLE_TRIG_LEVEL_LOW		4
#define GOOGLE_TRIG_LEVEL_HIGH		5

/* Interrupt status/enable/mask registers share bit 0 */
#define GOOGLE_IXR_IRQ_BIT		BIT(0)

struct google_pingroup {
	const char *name;
	unsigned int num;
	const unsigned int *pins;
	unsigned int npins;
	const int *funcs;
	unsigned int nfuncs;
};

#define GOOGLE_PINS(num)						\
	static const unsigned int google_pin_##num[] = { (num) }

#define GOOGLE_PIN_GROUP(gnum, gname, f0, f1, f2, f3, f4, f5, f6, f7, f8) \
	[gnum] = {							\
		.num = (gnum),						\
		.name = (gname),					\
		.pins = google_pin_##gnum,				\
		.npins = ARRAY_SIZE(google_pin_##gnum),			\
		.funcs = (const int[]){					\
			google_pinmux_##f0,				\
			google_pinmux_##f1,				\
			google_pinmux_##f2,				\
			google_pinmux_##f3,				\
			google_pinmux_##f4,				\
			google_pinmux_##f5,				\
			google_pinmux_##f6,				\
			google_pinmux_##f7,				\
			google_pinmux_##f8,				\
		},							\
		.nfuncs = GOOGLE_MAX_FUNCS_PER_PIN,			\
	}

struct google_pin_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
};

#define GOOGLE_FUNCTION_GROUPS(func, ...)				\
	static const char * const func##_groups[] = { __VA_ARGS__ }

#define GOOGLE_FUNCTION(func)						\
	[google_pinmux_##func] = {					\
		.name = #func,						\
		.groups = func##_groups,				\
		.ngroups = ARRAY_SIZE(func##_groups),			\
	}

struct google_pinctrl_soc_info {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	const struct google_pingroup *groups;
	unsigned int num_groups;
	const struct google_pin_function *funcs;
	unsigned int num_funcs;
	unsigned int gpio_func;
	size_t pin_stride;
	const char *label;
};

int google_pinctrl_probe(struct platform_device *pdev,
			 const struct google_pinctrl_soc_info *soc_info);

extern const struct dev_pm_ops google_pinctrl_pm_ops;

#endif /* _PINCTRL_GOOGLE_H_ */
