// SPDX-License-Identifier: GPL-2.0-only
/*
 * Multi-function core driver for DA9188 / DA9189 PMICs over Google CPM MBA.
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/mfd/da9188.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DA9188_MBOX_TIMEOUT_MS		3000
#define DA9188_MBOX_MAX_ATTEMPTS	3

static const struct mfd_cell da9188_mfd_cells[] = {
	{
		.name = "da9188-regulator",
		.of_compatible = "google,da9188-regulator",
	},
	{
		.name = "da9188-gpio",
		.of_compatible = "google,da9188-gpio",
	},
	{
		.name = "da9188-vgpio",
		.of_compatible = "google,da9188-vgpio",
	},
};

static void da9188_mbox_release(void *data)
{
	struct da9188_mbox *mbox = data;

	if (!IS_ERR_OR_NULL(mbox->client)) {
		cpm_iface_free_client(mbox->client);
		mbox->client = NULL;
	}
}

int da9188_devm_mbox_request(struct device *dev, struct da9188_mbox *mbox)
{
	mbox->client = cpm_iface_request_client(dev, 0, NULL, NULL);
	if (IS_ERR(mbox->client)) {
		int ret = PTR_ERR(mbox->client);

		mbox->client = NULL;
		return dev_err_probe(dev, ret, "failed to request CPM client\n");
	}

	return devm_add_action_or_reset(dev, da9188_mbox_release, mbox);
}
EXPORT_SYMBOL_GPL(da9188_devm_mbox_request);

int da9188_mbox_xfer(struct device *dev, struct da9188_mbox *mbox,
		     u8 dst_channel, u8 target, u8 cmd, u16 id_or_addr,
		     const struct da9188_mbox_data *req_data,
		     struct da9188_mbox_data *resp_data)
{
	struct cpm_iface_payload req_msg = {};
	struct cpm_iface_payload resp_msg = {};
	struct cpm_iface_req cpm_req = {
		.msg_type = REQUEST_MSG,
		.req_msg = &req_msg,
		.resp_msg = &resp_msg,
		.tout_ms = DA9188_MBOX_TIMEOUT_MS,
		.dst_id = dst_channel,
	};
	int attempt, ret = -EIO;

	if (IS_ERR_OR_NULL(mbox->client))
		return -ENODEV;

	req_msg.payload[0] = cpu_to_le32(
		FIELD_PREP(DA9188_MB_REQ_TARGET_MASK, target) |
		FIELD_PREP(DA9188_MB_REQ_CMD_MASK, cmd) |
		FIELD_PREP(DA9188_MB_REQ_ID_MASK, id_or_addr));
	if (req_data) {
		req_msg.payload[1] = cpu_to_le32(req_data->data[0]);
		req_msg.payload[2] = cpu_to_le32(req_data->data[1]);
	}

	for (attempt = 0; attempt < DA9188_MBOX_MAX_ATTEMPTS; attempt++) {
		u32 status;

		ret = cpm_send_message(mbox->client, &cpm_req);
		if (ret)
			continue;

		status = FIELD_GET(DA9188_MB_RSP_STATUS_MASK,
				   le32_to_cpu(resp_msg.payload[0]));
		if (status) {
			dev_err(dev,
				"PMIC MBA status %u (target=%u cmd=%u id=%u)\n",
				status, target, cmd, id_or_addr);
			ret = -EPIPE;
			continue;
		}

		if (resp_data) {
			resp_data->data[0] = le32_to_cpu(resp_msg.payload[1]);
			resp_data->data[1] = le32_to_cpu(resp_msg.payload[2]);
		}
		return 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(da9188_mbox_xfer);

static int da9188_mfd_probe(struct platform_device *pdev)
{
	if (!pdev->dev.of_node)
		return -ENODEV;

	return devm_mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE,
				    da9188_mfd_cells,
				    ARRAY_SIZE(da9188_mfd_cells),
				    NULL, 0, NULL);
}

static const struct of_device_id da9188_mfd_of_match[] = {
	{ .compatible = "google,da9188mfd" },
	{}
};
MODULE_DEVICE_TABLE(of, da9188_mfd_of_match);

static struct platform_driver da9188_mfd_driver = {
	.probe = da9188_mfd_probe,
	.driver = {
		.name = "da9188-mfd",
		.of_match_table = da9188_mfd_of_match,
	},
};
module_platform_driver(da9188_mfd_driver);

MODULE_DESCRIPTION("DA9188 / DA9189 PMIC MFD core driver");
MODULE_LICENSE("GPL");
