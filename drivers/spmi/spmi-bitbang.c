// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO bit-banged SPMI master controller for Google Tensor SoCs.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spmi.h>

#define SPMI_BB_CMD_ADDR_MASK		0x0fU
#define SPMI_BB_CMD_ADDR_SHIFT		8
#define SPMI_BB_CMD_LEN			12
#define SPMI_BB_DATA_LEN		8
#define SPMI_BB_MAX_EXT_BYTES		16
#define SPMI_BB_MAX_EXTL_BYTES		8
#define SPMI_BB_DEFAULT_DELAY_US	100U

struct spmi_bitbang {
	struct spmi_controller *ctrl;
	struct device *dev;
	struct mutex lock; /* serializes bit-banged bus transactions */
	struct gpio_desc *gpio_clk;
	struct gpio_desc *gpio_dat;
	u32 delay_us;
};

static inline void spmi_bb_delay(const struct spmi_bitbang *bb)
{
	udelay(bb->delay_us);
}

static inline void spmi_bb_set_clk(const struct spmi_bitbang *bb, bool high)
{
	gpiod_set_value_cansleep(bb->gpio_clk, high ? 1 : 0);
}

static inline void spmi_bb_set_dat(const struct spmi_bitbang *bb, bool high)
{
	gpiod_set_value_cansleep(bb->gpio_dat, high ? 1 : 0);
}

static inline bool spmi_bb_get_dat(const struct spmi_bitbang *bb)
{
	return !!gpiod_get_value_cansleep(bb->gpio_dat);
}

static void spmi_bb_bus_acquire(const struct spmi_bitbang *bb)
{
	gpiod_direction_output(bb->gpio_clk, 0);
	gpiod_direction_output(bb->gpio_dat, 0);
}

static void spmi_bb_bus_release(const struct spmi_bitbang *bb)
{
	gpiod_direction_input(bb->gpio_clk);
	gpiod_direction_input(bb->gpio_dat);
}

static void spmi_bb_send_bits(const struct spmi_bitbang *bb, u32 data,
			      unsigned int num_bits)
{
	u32 mask = BIT(num_bits);

	spmi_bb_set_clk(bb, false);
	while (mask >>= 1) {
		spmi_bb_set_clk(bb, true);
		spmi_bb_set_dat(bb, !!(data & mask));
		spmi_bb_delay(bb);
		spmi_bb_set_clk(bb, false);
		spmi_bb_delay(bb);
	}
}

static u32 spmi_bb_recv_bits(const struct spmi_bitbang *bb,
			     unsigned int num_bits)
{
	u32 data = 0;

	spmi_bb_set_clk(bb, false);
	while (num_bits--) {
		spmi_bb_set_clk(bb, true);
		data <<= 1;
		data |= spmi_bb_get_dat(bb) ? 1U : 0U;
		spmi_bb_delay(bb);
		spmi_bb_set_clk(bb, false);
		spmi_bb_delay(bb);
	}

	return data;
}

static bool spmi_bb_odd_parity(u32 data, unsigned int num_bits)
{
	bool parity = true;

	while (num_bits--) {
		parity ^= (data & 0x1U);
		data >>= 1;
	}

	return parity;
}

static void spmi_bb_send_frame(const struct spmi_bitbang *bb, u32 data,
			       unsigned int num_bits)
{
	u32 parity = spmi_bb_odd_parity(data, num_bits) ? 1U : 0U;

	spmi_bb_send_bits(bb, (data << 1) | parity, num_bits + 1);
}

static bool spmi_bb_recv_frame(const struct spmi_bitbang *bb, u32 *data,
			       unsigned int num_bits)
{
	u32 raw = spmi_bb_recv_bits(bb, num_bits + 1);
	bool parity = !!(raw & 0x1U);
	u32 payload = raw >> 1;

	*data = payload;
	return parity == spmi_bb_odd_parity(payload, num_bits);
}

static void spmi_bb_send_ssc(const struct spmi_bitbang *bb)
{
	gpiod_direction_output(bb->gpio_dat, 0);
	spmi_bb_set_clk(bb, false);
	spmi_bb_set_dat(bb, true);
	spmi_bb_delay(bb);
	spmi_bb_set_dat(bb, false);
	spmi_bb_delay(bb);
}

static void spmi_bb_send_cmd(const struct spmi_bitbang *bb, u8 sid, u8 cmd_byte)
{
	u16 frame = ((u16)(sid & SPMI_BB_CMD_ADDR_MASK) << SPMI_BB_CMD_ADDR_SHIFT) |
		    cmd_byte;

	spmi_bb_send_frame(bb, frame, SPMI_BB_CMD_LEN);
}

static void spmi_bb_send_byte(const struct spmi_bitbang *bb, u8 val)
{
	spmi_bb_send_frame(bb, val, SPMI_BB_DATA_LEN);
}

static bool spmi_bb_recv_byte(const struct spmi_bitbang *bb, u8 *val)
{
	u32 raw;
	bool ok = spmi_bb_recv_frame(bb, &raw, SPMI_BB_DATA_LEN);

	*val = (u8)raw;
	return ok;
}

static void spmi_bb_park_cycle(const struct spmi_bitbang *bb)
{
	spmi_bb_set_clk(bb, true);
	spmi_bb_set_dat(bb, false);
	spmi_bb_delay(bb);
	gpiod_direction_input(bb->gpio_dat);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);
}

static bool spmi_bb_recv_ack(const struct spmi_bitbang *bb)
{
	bool ack;

	spmi_bb_park_cycle(bb);
	spmi_bb_set_clk(bb, true);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	ack = spmi_bb_get_dat(bb);
	spmi_bb_delay(bb);
	spmi_bb_park_cycle(bb);

	return ack;
}

static void spmi_bb_clock_pulse(const struct spmi_bitbang *bb)
{
	spmi_bb_set_clk(bb, true);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);
}

static void spmi_bb_arbitrate(const struct spmi_bitbang *bb)
{
	unsigned int i;

	gpiod_direction_output(bb->gpio_dat, 0);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);
	spmi_bb_delay(bb);

	spmi_bb_set_dat(bb, true);
	for (i = 0; i < 5; i++)
		spmi_bb_delay(bb);

	/* First arbitration clock */
	spmi_bb_clock_pulse(bb);

	/* Bus park cycle */
	spmi_bb_set_clk(bb, true);
	spmi_bb_set_dat(bb, false);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);

	/* Contention / arbitration phase */
	spmi_bb_set_clk(bb, true);
	spmi_bb_set_dat(bb, true);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);

	spmi_bb_set_clk(bb, true);
	spmi_bb_set_dat(bb, false);
	gpiod_direction_input(bb->gpio_dat);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);

	/* Extra clock + MPL[0..2] */
	for (i = 0; i < 4; i++)
		spmi_bb_clock_pulse(bb);

	/* MPL[3] */
	spmi_bb_set_clk(bb, true);
	gpiod_direction_output(bb->gpio_dat, 1);
	spmi_bb_delay(bb);
	spmi_bb_set_clk(bb, false);
	spmi_bb_delay(bb);

	spmi_bb_set_dat(bb, false);
}

static bool spmi_bb_seq_write(const struct spmi_bitbang *bb, u8 sid, u8 reg,
			      u8 val)
{
	u8 cmd = SPMI_CMD_WRITE | (reg & 0x1f);

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_send_byte(bb, val);

	return spmi_bb_recv_ack(bb);
}

static bool spmi_bb_seq_read(const struct spmi_bitbang *bb, u8 sid, u8 reg,
			     u8 *val)
{
	u8 cmd = SPMI_CMD_READ | (reg & 0x1f);
	bool ok;

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_park_cycle(bb);
	ok = spmi_bb_recv_byte(bb, val);
	spmi_bb_park_cycle(bb);

	return ok;
}

static bool spmi_bb_seq_ext_write(const struct spmi_bitbang *bb, u8 sid,
				  u8 reg, const u8 *buf, size_t len)
{
	u8 cmd = SPMI_CMD_EXT_WRITE | ((len - 1) & 0x0f);
	size_t i;

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_send_byte(bb, reg);
	for (i = 0; i < len; i++)
		spmi_bb_send_byte(bb, buf[i]);

	return spmi_bb_recv_ack(bb);
}

static bool spmi_bb_seq_ext_read(const struct spmi_bitbang *bb, u8 sid,
				 u8 reg, u8 *buf, size_t len)
{
	u8 cmd = SPMI_CMD_EXT_READ | ((len - 1) & 0x0f);
	bool ok = true;
	size_t i;

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_send_byte(bb, reg);
	spmi_bb_park_cycle(bb);
	for (i = 0; i < len; i++)
		ok &= spmi_bb_recv_byte(bb, &buf[i]);
	spmi_bb_park_cycle(bb);

	return ok;
}

static bool spmi_bb_seq_ext_writel(const struct spmi_bitbang *bb, u8 sid,
				   u16 reg, const u8 *buf, size_t len)
{
	u8 cmd = SPMI_CMD_EXT_WRITEL | ((len - 1) & 0x07);
	size_t i;

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_send_byte(bb, reg >> 8);
	spmi_bb_send_byte(bb, reg & 0xff);
	for (i = 0; i < len; i++)
		spmi_bb_send_byte(bb, buf[i]);

	return spmi_bb_recv_ack(bb);
}

static bool spmi_bb_seq_ext_readl(const struct spmi_bitbang *bb, u8 sid,
				  u16 reg, u8 *buf, size_t len)
{
	u8 cmd = SPMI_CMD_EXT_READL | ((len - 1) & 0x07);
	bool ok = true;
	size_t i;

	spmi_bb_send_ssc(bb);
	spmi_bb_send_cmd(bb, sid, cmd);
	spmi_bb_send_byte(bb, reg >> 8);
	spmi_bb_send_byte(bb, reg & 0xff);
	spmi_bb_park_cycle(bb);
	for (i = 0; i < len; i++)
		ok &= spmi_bb_recv_byte(bb, &buf[i]);
	spmi_bb_park_cycle(bb);

	return ok;
}

static int spmi_bb_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid)
{
	return -EOPNOTSUPP;
}

static int spmi_bb_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			     u16 addr, const u8 *buf, size_t len)
{
	struct spmi_bitbang *bb = spmi_controller_get_drvdata(ctrl);
	bool acked;
	int ret = 0;

	if (!buf || !len)
		return -EINVAL;

	switch (opc) {
	case SPMI_CMD_WRITE:
		if (len != 1 || addr > 0x1f)
			return -EINVAL;
		break;
	case SPMI_CMD_EXT_WRITE:
		if (len > SPMI_BB_MAX_EXT_BYTES || addr > 0xff)
			return -EINVAL;
		break;
	case SPMI_CMD_EXT_WRITEL:
		if (len > SPMI_BB_MAX_EXTL_BYTES)
			return -EINVAL;
		break;
	default:
		dev_err(bb->dev, "unsupported write opcode %#x\n", opc);
		return -EINVAL;
	}

	mutex_lock(&bb->lock);
	spmi_bb_bus_acquire(bb);
	spmi_bb_arbitrate(bb);

	switch (opc) {
	case SPMI_CMD_WRITE:
		acked = spmi_bb_seq_write(bb, sid, addr, buf[0]);
		break;
	case SPMI_CMD_EXT_WRITE:
		acked = spmi_bb_seq_ext_write(bb, sid, addr, buf, len);
		break;
	case SPMI_CMD_EXT_WRITEL:
		acked = spmi_bb_seq_ext_writel(bb, sid, addr, buf, len);
		break;
	}

	spmi_bb_bus_release(bb);
	mutex_unlock(&bb->lock);

	if (!acked) {
		dev_err(bb->dev, "SPMI write not acked (opc %#x sid %u addr %#x)\n",
			opc, sid, addr);
		ret = -EIO;
	}

	return ret;
}

static int spmi_bb_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			    u16 addr, u8 *buf, size_t len)
{
	struct spmi_bitbang *bb = spmi_controller_get_drvdata(ctrl);
	bool ok;
	int ret = 0;

	if (!buf || !len)
		return -EINVAL;

	switch (opc) {
	case SPMI_CMD_READ:
		if (len != 1 || addr > 0x1f)
			return -EINVAL;
		break;
	case SPMI_CMD_EXT_READ:
		if (len > SPMI_BB_MAX_EXT_BYTES || addr > 0xff)
			return -EINVAL;
		break;
	case SPMI_CMD_EXT_READL:
		if (len > SPMI_BB_MAX_EXTL_BYTES)
			return -EINVAL;
		break;
	default:
		dev_err(bb->dev, "unsupported read opcode %#x\n", opc);
		return -EINVAL;
	}

	mutex_lock(&bb->lock);
	spmi_bb_bus_acquire(bb);
	spmi_bb_arbitrate(bb);

	switch (opc) {
	case SPMI_CMD_READ:
		ok = spmi_bb_seq_read(bb, sid, addr, buf);
		break;
	case SPMI_CMD_EXT_READ:
		ok = spmi_bb_seq_ext_read(bb, sid, addr, buf, len);
		break;
	case SPMI_CMD_EXT_READL:
		ok = spmi_bb_seq_ext_readl(bb, sid, addr, buf, len);
		break;
	}

	spmi_bb_bus_release(bb);
	mutex_unlock(&bb->lock);

	if (!ok) {
		dev_err(bb->dev, "SPMI read parity error (opc %#x sid %u addr %#x)\n",
			opc, sid, addr);
		ret = -EIO;
	}

	return ret;
}

static int spmi_bitbang_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spmi_controller *ctrl;
	struct spmi_bitbang *bb;
	u32 delay_us = SPMI_BB_DEFAULT_DELAY_US;
	int ret;

	ctrl = devm_spmi_controller_alloc(dev, sizeof(*bb));
	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);

	bb = spmi_controller_get_drvdata(ctrl);
	bb->ctrl = ctrl;
	bb->dev = dev;
	mutex_init(&bb->lock);

	bb->gpio_clk = devm_gpiod_get(dev, "clk", GPIOD_IN);
	if (IS_ERR(bb->gpio_clk))
		return dev_err_probe(dev, PTR_ERR(bb->gpio_clk),
				     "failed to acquire clk GPIO\n");

	bb->gpio_dat = devm_gpiod_get(dev, "dat", GPIOD_IN);
	if (IS_ERR(bb->gpio_dat))
		return dev_err_probe(dev, PTR_ERR(bb->gpio_dat),
				     "failed to acquire dat GPIO\n");

	if (device_property_read_u32(dev, "delay-us", &delay_us) &&
	    device_property_read_u32(dev, "delay_us", &delay_us))
		delay_us = SPMI_BB_DEFAULT_DELAY_US;
	bb->delay_us = delay_us;

	ctrl->dev.of_node = dev->of_node;
	ctrl->cmd = spmi_bb_cmd;
	ctrl->read_cmd = spmi_bb_read_cmd;
	ctrl->write_cmd = spmi_bb_write_cmd;

	ret = devm_spmi_controller_add(dev, ctrl);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register bit-bang SPMI controller\n");

	return 0;
}

static const struct of_device_id spmi_bitbang_of_match[] = {
	{ .compatible = "google,bitbang-spmi-controller" },
	{}
};
MODULE_DEVICE_TABLE(of, spmi_bitbang_of_match);

static struct platform_driver spmi_bitbang_driver = {
	.probe = spmi_bitbang_probe,
	.driver = {
		.name = "spmi-bitbang",
		.of_match_table = spmi_bitbang_of_match,
	},
};
module_platform_driver(spmi_bitbang_driver);

MODULE_DESCRIPTION("Google GPIO bit-banged SPMI controller driver");
MODULE_LICENSE("GPL");
