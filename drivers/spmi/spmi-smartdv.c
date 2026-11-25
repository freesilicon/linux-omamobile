// SPDX-License-Identifier: GPL-2.0-only
/*
 * SmartDV SPMI controller driver for Google Laguna (Pixel 10) SoCs.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/spmi.h>

#define SMARTDV_SPMI_CTRL		0x00
#define  SMARTDV_CTRL_EN		BIT(0)
#define  SMARTDV_CTRL_MASTER_ARB_EN	BIT(1)
#define  SMARTDV_CTRL_MASTER_ID		GENMASK(3, 2)
#define  SMARTDV_CTRL_SPMI_VER		GENMASK(5, 4)
#define  SMARTDV_CTRL_ARB_RETRY_LIMIT	GENMASK(13, 6)
#define  SMARTDV_CTRL_PRESCALAR		GENMASK(23, 14)
#define  SMARTDV_CTRL_READ_LATENCY	GENMASK(29, 22)
#define  SMARTDV_CTRL_ARB_SCL_DELAY	GENMASK(31, 30)

#define SMARTDV_SPMI_IRQ_EN		0x04
#define  SMARTDV_IRQ_EN_MASK		GENMASK(12, 0)

#define SMARTDV_SPMI_STATUS		0x08
#define  SMARTDV_STATUS_DONE		BIT(0)
#define  SMARTDV_STATUS_ERR_MASK	GENMASK(9, 2)

#define SMARTDV_SPMI_CMD		0x10
#define  SMARTDV_CMD_START		GENMASK(1, 0)
#define  SMARTDV_CMD_OPC		GENMASK(6, 2)
#define  SMARTDV_CMD_SID		GENMASK(10, 7)
#define  SMARTDV_CMD_LEN		GENMASK(14, 11)
#define  SMARTDV_CMD_ADDR		GENMASK(30, 15)

#define SMARTDV_SPMI_DATA		0x14
#define SMARTDV_SPMI_DATA_WORDS		5
#define SMARTDV_SPMI_MAX_BYTES		16

#define SMARTDV_DEFAULT_RETRY_LIMIT	0x20
#define SMARTDV_DEFAULT_READ_LATENCY	0x20
#define SMARTDV_POLL_INTERVAL_US	1000
#define SMARTDV_POLL_TIMEOUT_US		100000

enum smartdv_hw_opc {
	SMARTDV_OPC_WRITE = 0,
	SMARTDV_OPC_READ = 1,
	SMARTDV_OPC_EXT_WRITE = 2,
	SMARTDV_OPC_EXT_READ = 3,
	SMARTDV_OPC_EXT_WRITEL = 4,
	SMARTDV_OPC_EXT_READL = 5,
	SMARTDV_OPC_ZERO_WRITE = 6,
};

struct smartdv_spmi {
	struct spmi_controller *ctrl;
	struct device *dev;
	void __iomem *base;
	struct mutex lock; /* serializes controller register programming */
	struct completion done;
	int irq;
	u32 ctrl_val;
	u32 status;
};

static irqreturn_t smartdv_spmi_irq(int irq, void *data)
{
	struct smartdv_spmi *spmi = data;
	u32 status;

	status = readl(spmi->base + SMARTDV_SPMI_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(0, spmi->base + SMARTDV_SPMI_IRQ_EN);
	writel(status, spmi->base + SMARTDV_SPMI_STATUS);
	spmi->status = status;
	complete(&spmi->done);

	return IRQ_HANDLED;
}

static int smartdv_spmi_xfer(struct smartdv_spmi *spmi, bool has_addr,
			     u8 hw_opc, u8 sid, u16 addr, size_t len)
{
	u32 cmd, status;
	unsigned long rem;
	int ret = 0;

	cmd = FIELD_PREP(SMARTDV_CMD_SID, sid) |
	      FIELD_PREP(SMARTDV_CMD_OPC, hw_opc) |
	      SMARTDV_CMD_START;
	if (has_addr)
		cmd |= FIELD_PREP(SMARTDV_CMD_LEN, len - 1) |
		       FIELD_PREP(SMARTDV_CMD_ADDR, addr);

	writel(spmi->ctrl_val | SMARTDV_CTRL_EN, spmi->base + SMARTDV_SPMI_CTRL);

	if (spmi->irq > 0) {
		spmi->status = 0;
		reinit_completion(&spmi->done);
		writel(SMARTDV_IRQ_EN_MASK, spmi->base + SMARTDV_SPMI_IRQ_EN);
		writel(cmd, spmi->base + SMARTDV_SPMI_CMD);

		rem = wait_for_completion_timeout(&spmi->done,
						  msecs_to_jiffies(100));
		if (!rem) {
			writel(0, spmi->base + SMARTDV_SPMI_IRQ_EN);
			status = readl(spmi->base + SMARTDV_SPMI_STATUS);
			if (status)
				writel(status, spmi->base + SMARTDV_SPMI_STATUS);
			ret = -ETIMEDOUT;
		} else {
			status = spmi->status;
		}
	} else {
		writel(cmd, spmi->base + SMARTDV_SPMI_CMD);
		ret = readl_poll_timeout(spmi->base + SMARTDV_SPMI_STATUS,
					 status,
					 status & (SMARTDV_STATUS_DONE |
						   SMARTDV_STATUS_ERR_MASK),
					 SMARTDV_POLL_INTERVAL_US,
					 SMARTDV_POLL_TIMEOUT_US);
		if (status)
			writel(status, spmi->base + SMARTDV_SPMI_STATUS);
	}

	writel(spmi->ctrl_val, spmi->base + SMARTDV_SPMI_CTRL);

	if (ret)
		return ret;
	if (status & SMARTDV_STATUS_ERR_MASK)
		return -EIO;
	if (!(status & SMARTDV_STATUS_DONE))
		return -EINVAL;

	return 0;
}

static int smartdv_spmi_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid)
{
	struct smartdv_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u8 hw_opc;
	int ret;

	switch (opc) {
	case SPMI_CMD_RESET:
	case SPMI_CMD_SLEEP:
	case SPMI_CMD_SHUTDOWN:
	case SPMI_CMD_WAKEUP:
		hw_opc = opc - 6;
		break;
	default:
		dev_err(spmi->dev, "unsupported SPMI command opcode %#x\n", opc);
		return -EINVAL;
	}

	mutex_lock(&spmi->lock);
	ret = smartdv_spmi_xfer(spmi, false, hw_opc, sid, 0, 0);
	mutex_unlock(&spmi->lock);

	return ret;
}

static int smartdv_spmi_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
				 u16 addr, u8 *buf, size_t len)
{
	struct smartdv_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u8 hw_opc;
	size_t i, j;
	int ret;

	if (!len || len > SMARTDV_SPMI_MAX_BYTES)
		return -EINVAL;

	switch (opc) {
	case SPMI_CMD_READ:
		hw_opc = SMARTDV_OPC_READ;
		break;
	case SPMI_CMD_EXT_READ:
		hw_opc = SMARTDV_OPC_EXT_READ;
		break;
	case SPMI_CMD_EXT_READL:
		hw_opc = SMARTDV_OPC_EXT_READL;
		break;
	default:
		dev_err(spmi->dev, "unsupported read opcode %#x\n", opc);
		return -EINVAL;
	}

	mutex_lock(&spmi->lock);

	for (i = 0; i < SMARTDV_SPMI_DATA_WORDS; i++)
		writel(0, spmi->base + SMARTDV_SPMI_DATA + i * 4);

	ret = smartdv_spmi_xfer(spmi, true, hw_opc, sid, addr, len);
	if (!ret) {
		for (i = 0; i < len; i += 4) {
			u32 val = readl(spmi->base + SMARTDV_SPMI_DATA + i);
			size_t chunk = min_t(size_t, 4, len - i);

			for (j = 0; j < chunk; j++)
				buf[i + j] = (val >> (j * 8)) & 0xff;
		}
	}

	mutex_unlock(&spmi->lock);

	return ret;
}

static int smartdv_spmi_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
				  u16 addr, const u8 *buf, size_t len)
{
	struct smartdv_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u8 hw_opc;
	size_t i, j;
	int ret;

	if (!len || len > SMARTDV_SPMI_MAX_BYTES)
		return -EINVAL;

	switch (opc) {
	case SPMI_CMD_WRITE:
		hw_opc = SMARTDV_OPC_WRITE;
		break;
	case SPMI_CMD_ZERO_WRITE:
		hw_opc = SMARTDV_OPC_ZERO_WRITE;
		break;
	case SPMI_CMD_EXT_WRITE:
		hw_opc = SMARTDV_OPC_EXT_WRITE;
		break;
	case SPMI_CMD_EXT_WRITEL:
		hw_opc = SMARTDV_OPC_EXT_WRITEL;
		break;
	default:
		dev_err(spmi->dev, "unsupported write opcode %#x\n", opc);
		return -EINVAL;
	}

	mutex_lock(&spmi->lock);

	for (i = 0; i < len; i += 4) {
		u32 val = 0;
		size_t chunk = min_t(size_t, 4, len - i);

		for (j = 0; j < chunk; j++)
			val |= (u32)buf[i + j] << (j * 8);
		writel(val, spmi->base + SMARTDV_SPMI_DATA + i);
	}

	ret = smartdv_spmi_xfer(spmi, true, hw_opc, sid, addr, len);
	mutex_unlock(&spmi->lock);

	return ret;
}

static void smartdv_spmi_parse_ctrl(struct smartdv_spmi *spmi)
{
	struct device *dev = spmi->dev;
	u32 val, ctrl = 0;

	if (!device_property_read_u32(dev, "master_arb_en", &val) && val)
		ctrl |= SMARTDV_CTRL_MASTER_ARB_EN;

	if (!device_property_read_u32(dev, "master_id", &val))
		ctrl |= FIELD_PREP(SMARTDV_CTRL_MASTER_ID, val);

	if (!device_property_read_u32(dev, "spmi_ver", &val))
		ctrl |= FIELD_PREP(SMARTDV_CTRL_SPMI_VER, val);

	if (device_property_read_u32(dev, "arb_retry_limit", &val))
		val = SMARTDV_DEFAULT_RETRY_LIMIT;
	ctrl |= FIELD_PREP(SMARTDV_CTRL_ARB_RETRY_LIMIT, val);

	if (!device_property_read_u32(dev, "prescalar", &val))
		ctrl |= FIELD_PREP(SMARTDV_CTRL_PRESCALAR, val);

	if (device_property_read_u32(dev, "read_latancy", &val) &&
	    device_property_read_u32(dev, "read-latency", &val))
		val = SMARTDV_DEFAULT_READ_LATENCY;
	ctrl |= FIELD_PREP(SMARTDV_CTRL_READ_LATENCY, val);

	if (!device_property_read_u32(dev, "arb_scl_delay", &val))
		ctrl |= FIELD_PREP(SMARTDV_CTRL_ARB_SCL_DELAY, val);

	spmi->ctrl_val = ctrl;
}

static void smartdv_spmi_reset_assert(void *data)
{
	reset_control_assert(data);
}

static int smartdv_spmi_probe(struct platform_device *pdev)
{
	struct spmi_controller *ctrl;
	struct smartdv_spmi *spmi;
	struct reset_control *rst;
	struct clk *clk;
	u32 clk_freq = 0;
	int ret;

	ctrl = devm_spmi_controller_alloc(&pdev->dev, sizeof(*spmi));
	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);

	spmi = spmi_controller_get_drvdata(ctrl);
	spmi->ctrl = ctrl;
	spmi->dev = &pdev->dev;
	mutex_init(&spmi->lock);
	init_completion(&spmi->done);

	spmi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spmi->base))
		return PTR_ERR(spmi->base);

	smartdv_spmi_parse_ctrl(spmi);

	clk = devm_clk_get_optional_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "failed to enable SPMI clock\n");

	if (clk)
		clk_freq = clk_get_rate(clk);
	if (!clk_freq)
		device_property_read_u32(&pdev->dev, "clock-frequency",
					 &clk_freq);
	if (!clk_freq)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "clock-frequency is required\n");

	rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst),
				     "failed to get reset control\n");
	if (rst) {
		ret = reset_control_deassert(rst);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to deassert reset\n");
		ret = devm_add_action_or_reset(&pdev->dev,
					       smartdv_spmi_reset_assert, rst);
		if (ret)
			return ret;
	}

	spmi->irq = platform_get_irq_optional(pdev, 0);
	if (spmi->irq < 0 && spmi->irq != -ENXIO)
		return spmi->irq;
	if (spmi->irq > 0) {
		ret = devm_request_irq(&pdev->dev, spmi->irq, smartdv_spmi_irq,
				       0, dev_name(&pdev->dev), spmi);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to request IRQ\n");
	} else {
		spmi->irq = 0;
	}

	ctrl->dev.of_node = pdev->dev.of_node;
	ctrl->cmd = smartdv_spmi_cmd;
	ctrl->read_cmd = smartdv_spmi_read_cmd;
	ctrl->write_cmd = smartdv_spmi_write_cmd;

	ret = devm_spmi_controller_add(&pdev->dev, ctrl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to add SPMI controller\n");

	return 0;
}

static const struct of_device_id smartdv_spmi_of_match[] = {
	{ .compatible = "smartdv,spmi" },
	{ .compatible = "google,smartdv-spmi" },
	{}
};
MODULE_DEVICE_TABLE(of, smartdv_spmi_of_match);

static struct platform_driver smartdv_spmi_driver = {
	.probe = smartdv_spmi_probe,
	.driver = {
		.name = "spmi-smartdv",
		.of_match_table = smartdv_spmi_of_match,
	},
};
module_platform_driver(smartdv_spmi_driver);

MODULE_DESCRIPTION("SmartDV SPMI controller driver");
MODULE_LICENSE("GPL");
