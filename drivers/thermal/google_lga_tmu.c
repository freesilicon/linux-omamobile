// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Laguna (Tensor G5) CPM Thermal Management Unit (TMU) driver.
 *
 * Queries on-die thermal zone temperatures from the Central Power Manager
 * (CPM) firmware over the CPM mailbox interface without accessing TMU MMIO
 * registers directly or installing vendor hooks.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <soc/google/goog_cpm_service_ids.h>
#include <soc/google/goog_mba_cpm_iface.h>

/* Sensor IDs: 0=BIG, 1=BIG_MID, 2=MID, 3=LITTLE, 4=GPU, 5=TPU, 6=AUR, 7=ISP, 8=MEM, 9=AOC */
#define LGA_TMU_MAX_SENSORS		10
#define LGA_TMU_MBOX_TIMEOUT_MS		3000

#define LGA_TMU_MSG_GET_TEMP		0x0001
#define LGA_TMU_MSG_CMD_MASK		GENMASK(15, 0)
#define LGA_TMU_MSG_TZID_MASK		GENMASK(23, 16)

#define LGA_TMU_RESP_STATUS_MASK	GENMASK(31, 24)
#define LGA_TMU_RESP_TEMP_MASK		GENMASK(7, 0)

struct lga_tmu;

struct lga_tmu_sensor {
	struct lga_tmu *tmu;
	u8 id;
};

struct lga_tmu {
	struct cpm_iface_client *client;
	struct lga_tmu_sensor sensors[LGA_TMU_MAX_SENSORS];
};

static int lga_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct lga_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct cpm_iface_payload req_msg = {
		.payload = {
			FIELD_PREP(LGA_TMU_MSG_CMD_MASK, LGA_TMU_MSG_GET_TEMP) |
			FIELD_PREP(LGA_TMU_MSG_TZID_MASK, sensor->id),
			0,
			0,
		},
	};
	struct cpm_iface_payload resp_msg = {};
	struct cpm_iface_req req = {
		.msg_type = REQUEST_MSG,
		.req_msg = &req_msg,
		.resp_msg = &resp_msg,
		.dst_id = CPM_COMMON_THERMAL_V2_SERVICE,
		.tout_ms = LGA_TMU_MBOX_TIMEOUT_MS,
	};
	int ret, status;

	ret = cpm_send_message(sensor->tmu->client, &req);
	if (ret)
		return ret;

	/* Signed 8-bit CPM firmware status in bits [31:24]; -ENODATA if rail gated */
	status = (s8)FIELD_GET(LGA_TMU_RESP_STATUS_MASK, resp_msg.payload[0]);
	if (status == -ENODATA) {
		*temp = THERMAL_TEMP_INVALID;
		return 0;
	}
	if (status)
		return status < 0 ? status : -EIO;

	*temp = FIELD_GET(LGA_TMU_RESP_TEMP_MASK, resp_msg.payload[1]) * 1000;
	return 0;
}

static const struct thermal_zone_device_ops lga_tmu_ops = {
	.get_temp = lga_tmu_get_temp,
};

static void lga_tmu_free_client(void *data)
{
	cpm_iface_free_client(data);
}

static int lga_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpm_iface_client *client;
	struct lga_tmu *tmu;
	unsigned int i;
	int ret;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	client = cpm_iface_request_client(dev, 0, NULL, NULL);
	if (IS_ERR(client))
		return dev_err_probe(dev, PTR_ERR(client),
				     "failed to request CPM client\n");

	ret = devm_add_action_or_reset(dev, lga_tmu_free_client, client);
	if (ret)
		return ret;

	tmu->client = client;

	for (i = 0; i < LGA_TMU_MAX_SENSORS; i++) {
		struct thermal_zone_device *tzd;

		tmu->sensors[i].tmu = tmu;
		tmu->sensors[i].id = i;

		tzd = devm_thermal_of_zone_register(dev, i, &tmu->sensors[i],
						    &lga_tmu_ops);
		if (IS_ERR(tzd)) {
			if (PTR_ERR(tzd) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tzd),
					     "failed to register thermal zone %u\n",
					     i);
		}
	}

	return 0;
}

static const struct of_device_id lga_tmu_of_match[] = {
	{ .compatible = "google,lga-tmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, lga_tmu_of_match);

static struct platform_driver lga_tmu_driver = {
	.probe = lga_tmu_probe,
	.driver = {
		.name = "google-lga-tmu",
		.of_match_table = lga_tmu_of_match,
	},
};
module_platform_driver(lga_tmu_driver);

MODULE_AUTHOR("Bobby Bones <saariuscrypto@gmail.com>");
MODULE_DESCRIPTION("Google Laguna (Tensor G5) CPM thermal sensor driver");
MODULE_LICENSE("GPL");
