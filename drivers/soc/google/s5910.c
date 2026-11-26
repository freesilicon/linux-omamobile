// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5910 SPMI modem reference clock buffer driver.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device/driver.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/s5910.h>
#include <linux/slab.h>
#include <linux/spmi.h>

#define S5910_REG_DCXO_LPM_MODE		0x020e
#define S5910_SEQ_CELLS_PER_STEP	3

struct s5910_seq_step {
	u32 delay_ms;
	u32 reg;
	u32 val;
};

struct s5910_seq {
	struct s5910_seq_step *steps;
	size_t count;
};

struct s5910_info {
	struct device *dev;
	struct regmap *regmap;
	struct s5910_seq off_seq;
	struct s5910_seq on_seq;
};

static const struct regmap_range s5910_wr_range[] = {
	regmap_reg_range(0x0000, 0x0008),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0201, 0x0201),
	regmap_reg_range(0x0207, 0x0207),
	regmap_reg_range(0x0209, 0x020e),
	regmap_reg_range(0x0211, 0x0244),
	regmap_reg_range(0x0275, 0x029a),
};

static const struct regmap_access_table s5910_wr_table = {
	.yes_ranges = s5910_wr_range,
	.n_yes_ranges = ARRAY_SIZE(s5910_wr_range),
};

static const struct regmap_range s5910_rd_range[] = {
	regmap_reg_range(0x0000, 0x0008),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0200, 0x0201),
	regmap_reg_range(0x0207, 0x0207),
	regmap_reg_range(0x0209, 0x020e),
	regmap_reg_range(0x0211, 0x0244),
	regmap_reg_range(0x0275, 0x029a),
};

static const struct regmap_access_table s5910_rd_table = {
	.yes_ranges = s5910_rd_range,
	.n_yes_ranges = ARRAY_SIZE(s5910_rd_range),
};

static const struct regmap_config s5910_regmap_cfg = {
	.name = "s5910_clock_buffer",
	.reg_bits = 16,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = 0x29a,
	.wr_table = &s5910_wr_table,
	.rd_table = &s5910_rd_table,
};

static int s5910_parse_seq(struct device *dev, const char *propname,
			   struct s5910_seq *seq)
{
	int count;
	size_t i;
	u32 *raw;
	int ret;

	count = of_property_count_u32_elems(dev->of_node, propname);
	if (count < 0)
		return count;
	if (count == 0 || (count % S5910_SEQ_CELLS_PER_STEP) != 0)
		return -EINVAL;

	raw = kcalloc(count, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, propname, raw, count);
	if (ret) {
		kfree(raw);
		return ret;
	}

	seq->count = count / S5910_SEQ_CELLS_PER_STEP;
	seq->steps = devm_kcalloc(dev, seq->count, sizeof(*seq->steps),
				  GFP_KERNEL);
	if (!seq->steps) {
		kfree(raw);
		return -ENOMEM;
	}

	for (i = 0; i < seq->count; i++) {
		size_t base = i * S5910_SEQ_CELLS_PER_STEP;

		seq->steps[i].delay_ms = raw[base];
		seq->steps[i].reg = raw[base + 1];
		seq->steps[i].val = raw[base + 2];
	}

	kfree(raw);
	return 0;
}

static int s5910_run_sequence(struct device *dev, const char *name,
			      const struct s5910_seq *seq)
{
	struct s5910_info *info = dev_get_drvdata(dev);
	unsigned int rb_val;
	size_t i;
	int ret;

	if (!info || !info->regmap)
		return -ENODEV;

	for (i = 0; i < seq->count; i++) {
		const struct s5910_seq_step *step = &seq->steps[i];

		if (step->delay_ms)
			msleep(step->delay_ms);

		ret = regmap_write(info->regmap, step->reg, step->val);
		if (ret) {
			dev_err(dev, "%s step %zu write %#x=%#x failed: %d\n",
				name, i, step->reg, step->val, ret);
			return ret;
		}

		ret = regmap_read(info->regmap, step->reg, &rb_val);
		if (ret) {
			dev_err(dev, "%s step %zu readback %#x failed: %d\n",
				name, i, step->reg, ret);
			return ret;
		}
		if (rb_val != step->val)
			dev_dbg(dev, "%s step %zu %#x wrote %#x read %#x\n",
				name, i, step->reg, step->val, rb_val);
	}

	return 0;
}

int s5910_shutdown_sequence(struct device *dev)
{
	struct s5910_info *info = dev ? dev_get_drvdata(dev) : NULL;

	if (!info)
		return -ENODEV;

	return s5910_run_sequence(dev, "off", &info->off_seq);
}
EXPORT_SYMBOL_GPL(s5910_shutdown_sequence);

int s5910_turn_on_sequence(struct device *dev)
{
	struct s5910_info *info = dev ? dev_get_drvdata(dev) : NULL;

	if (!info)
		return -ENODEV;

	return s5910_run_sequence(dev, "on", &info->on_seq);
}
EXPORT_SYMBOL_GPL(s5910_turn_on_sequence);

int s5910_check_lpm_mode(struct device *dev)
{
	struct s5910_info *info = dev ? dev_get_drvdata(dev) : NULL;
	unsigned int val;
	int ret;

	if (!info || !info->regmap)
		return -ENODEV;

	ret = regmap_read(info->regmap, S5910_REG_DCXO_LPM_MODE, &val);
	if (ret) {
		dev_err(dev, "failed to read LPM mode register: %d\n", ret);
		return ret;
	}

	dev_info(dev, "S5910 DCXO LPM mode: %#02x\n", val);
	return 0;
}
EXPORT_SYMBOL_GPL(s5910_check_lpm_mode);

static int s5910_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct s5910_info *info;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	ret = s5910_parse_seq(dev, "s5910,seq", &info->off_seq);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse s5910,seq\n");

	ret = s5910_parse_seq(dev, "s5910,on_seq", &info->on_seq);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse s5910,on_seq\n");

	info->regmap = devm_regmap_init_spmi_ext(sdev, &s5910_regmap_cfg);
	if (IS_ERR(info->regmap))
		return dev_err_probe(dev, PTR_ERR(info->regmap),
				     "failed to initialize S5910 regmap\n");

	spmi_device_set_drvdata(sdev, info);
	return 0;
}

static const struct of_device_id s5910_of_match[] = {
	{ .compatible = "google,s5910-spmi" },
	{}
};
MODULE_DEVICE_TABLE(of, s5910_of_match);

static struct spmi_driver s5910_driver = {
	.probe = s5910_probe,
	.driver = {
		.name = "s5910-spmi",
		.of_match_table = s5910_of_match,
	},
};
module_spmi_driver(s5910_driver);

struct device *s5910_get_device(struct device_node *node)
{
	if (!node)
		return NULL;

	return driver_find_device_by_of_node(&s5910_driver.driver, node);
}
EXPORT_SYMBOL_GPL(s5910_get_device);

struct spmi_device *s5910_get_spmi_device(struct device_node *node)
{
	struct device *dev = s5910_get_device(node);

	return dev ? to_spmi_device(dev) : NULL;
}
EXPORT_SYMBOL_GPL(s5910_get_spmi_device);

MODULE_DESCRIPTION("Google S5910 SPMI modem clock buffer driver");
MODULE_LICENSE("GPL");
