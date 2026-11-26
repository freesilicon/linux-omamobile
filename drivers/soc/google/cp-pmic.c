// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google cellular modem companion PMIC SPMI driver.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device/driver.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <soc/google/cp_pmic.h>

#define CP_PMIC_RETRY_COUNT		2
#define CP_PMIC_SEQ_CELLS_PER_ENTRY	3

struct cp_pmic_reg_entry {
	u32 reg;
	u32 val;
	u32 delay_ms;
};

struct cp_pmic_info {
	struct spmi_device *sdev;
	struct regmap *regmap;
	struct cp_pmic_reg_entry *warm_reset_seq;
	size_t warm_reset_count;
	u32 otp_reg;
};

static const struct regmap_range cp_pmic_wr_range[] = {
	regmap_reg_range(0x0675, 0x067d),
};

static const struct regmap_access_table cp_pmic_wr_table = {
	.yes_ranges = cp_pmic_wr_range,
	.n_yes_ranges = ARRAY_SIZE(cp_pmic_wr_range),
};

static const struct regmap_range cp_pmic_rd_range[] = {
	regmap_reg_range(0x030e, 0x067d),
};

static const struct regmap_access_table cp_pmic_rd_table = {
	.yes_ranges = cp_pmic_rd_range,
	.n_yes_ranges = ARRAY_SIZE(cp_pmic_rd_range),
};

static const struct regmap_config cp_pmic_regmap_cfg = {
	.name = "modem_pmic",
	.reg_bits = 16,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = 0x067d,
	.wr_table = &cp_pmic_wr_table,
	.rd_table = &cp_pmic_rd_table,
};

static ssize_t pmic_read_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct cp_pmic_info *info = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	if (!info || !info->regmap)
		return -ENODEV;

	if (kstrtouint(buf, 16, &reg))
		return -EINVAL;

	ret = regmap_read(info->regmap, reg, &val);
	if (ret) {
		dev_err(dev, "failed to read register %#x: %d\n", reg, ret);
		return ret;
	}

	dev_info(dev, "PMIC register %#06x = %#04x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(pmic_read);

static ssize_t pmic_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cp_pmic_info *info = dev_get_drvdata(dev);
	u32 reg, val;
	int ret;

	if (!info || !info->regmap)
		return -ENODEV;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	ret = regmap_write(info->regmap, reg, val);
	if (ret) {
		dev_err(dev, "failed to write register %#x: %d\n", reg, ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR_WO(pmic_write);

static struct attribute *cp_pmic_attrs[] = {
	&dev_attr_pmic_read.attr,
	&dev_attr_pmic_write.attr,
	NULL,
};

static const struct attribute_group cp_pmic_attr_group = {
	.name = "pmic",
	.attrs = cp_pmic_attrs,
};

void pmic_warm_reset_sequence(struct device *dev)
{
	struct cp_pmic_info *info = dev ? dev_get_drvdata(dev) : NULL;
	size_t i;

	if (!info || !info->regmap) {
		if (dev)
			dev_err(dev, "CP PMIC state not available\n");
		return;
	}

	for (i = 0; i < info->warm_reset_count; i++) {
		const struct cp_pmic_reg_entry *entry = &info->warm_reset_seq[i];
		int retry;
		int ret = -EIO;

		for (retry = 0; retry < CP_PMIC_RETRY_COUNT; retry++) {
			ret = regmap_write(info->regmap, entry->reg, entry->val);
			if (!ret)
				break;
			fsleep(2000);
		}

		if (ret) {
			dev_err(dev, "warm reset write %#x=%#x failed: %d\n",
				entry->reg, entry->val, ret);
			return;
		}

		if (entry->delay_ms)
			msleep(entry->delay_ms);
	}

	dev_info(dev, "CP PMIC warm reset sequence completed\n");
}
EXPORT_SYMBOL_GPL(pmic_warm_reset_sequence);

int pmic_get_otp(struct device *dev)
{
	struct cp_pmic_info *info = dev ? dev_get_drvdata(dev) : NULL;
	unsigned int otp_val = 0;
	int retry;
	int ret = -EIO;

	if (!info || !info->regmap || !info->otp_reg)
		return -ENODEV;

	for (retry = 0; retry < CP_PMIC_RETRY_COUNT; retry++) {
		ret = regmap_read(info->regmap, info->otp_reg, &otp_val);
		if (!ret)
			return (int)otp_val;
		fsleep(2000);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pmic_get_otp);

static int cp_pmic_dt_init(struct device *dev, struct cp_pmic_info *info)
{
	int count;
	size_t i;
	u32 *raw;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	if (of_property_read_u32(dev->of_node, "pmic-otp-reg", &info->otp_reg))
		dev_dbg(dev, "pmic-otp-reg not specified\n");

	count = of_property_count_u32_elems(dev->of_node, "warm_reset_seq");
	if (count < 0)
		return count;
	if (count == 0 || (count % CP_PMIC_SEQ_CELLS_PER_ENTRY) != 0)
		return -EINVAL;

	raw = kcalloc(count, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, "warm_reset_seq",
					 raw, count);
	if (ret) {
		kfree(raw);
		return ret;
	}

	info->warm_reset_count = count / CP_PMIC_SEQ_CELLS_PER_ENTRY;
	info->warm_reset_seq = devm_kcalloc(dev, info->warm_reset_count,
					    sizeof(*info->warm_reset_seq),
					    GFP_KERNEL);
	if (!info->warm_reset_seq) {
		kfree(raw);
		return -ENOMEM;
	}

	for (i = 0; i < info->warm_reset_count; i++) {
		size_t base = i * CP_PMIC_SEQ_CELLS_PER_ENTRY;

		info->warm_reset_seq[i].reg = raw[base];
		info->warm_reset_seq[i].val = raw[base + 1];
		info->warm_reset_seq[i].delay_ms = raw[base + 2];
	}

	kfree(raw);
	return 0;
}

static int cp_pmic_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct cp_pmic_info *info;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->sdev = sdev;
	spmi_device_set_drvdata(sdev, info);

	info->regmap = devm_regmap_init_spmi_ext(sdev, &cp_pmic_regmap_cfg);
	if (IS_ERR(info->regmap))
		return dev_err_probe(dev, PTR_ERR(info->regmap),
				     "failed to initialize PMIC regmap\n");

	ret = cp_pmic_dt_init(dev, info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse PMIC DT data\n");

	ret = devm_device_add_group(dev, &cp_pmic_attr_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add PMIC sysfs group\n");

	return 0;
}

static const struct of_device_id cp_pmic_of_match[] = {
	{ .compatible = "google,cp-pmic-spmi" },
	{}
};
MODULE_DEVICE_TABLE(of, cp_pmic_of_match);

static struct spmi_driver cp_pmic_driver = {
	.probe = cp_pmic_probe,
	.driver = {
		.name = "cp-pmic-spmi",
		.of_match_table = cp_pmic_of_match,
	},
};
module_spmi_driver(cp_pmic_driver);

struct device *pmic_get_device(struct device_node *node)
{
	if (!node)
		return NULL;

	return driver_find_device_by_of_node(&cp_pmic_driver.driver, node);
}
EXPORT_SYMBOL_GPL(pmic_get_device);

struct spmi_device *pmic_get_spmi_device(struct device_node *node)
{
	struct device *dev = pmic_get_device(node);

	return dev ? to_spmi_device(dev) : NULL;
}
EXPORT_SYMBOL_GPL(pmic_get_spmi_device);

MODULE_DESCRIPTION("Google cellular modem companion PMIC SPMI driver");
MODULE_LICENSE("GPL");
