/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DA9188 / DA9189 PMIC CPM mailbox interface definitions.
 */

#ifndef _LINUX_MFD_DA9188_H
#define _LINUX_MFD_DA9188_H

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/types.h>
#include <soc/google/goog_cpm_service_ids.h>
#include <soc/google/goog_mba_cpm_iface.h>

enum da9188_mb_target {
	DA9188_MB_TARGET_REGISTER	= 0,
	DA9188_MB_TARGET_REGULATOR	= 1,
	DA9188_MB_TARGET_RTC		= 2,
	DA9188_MB_TARGET_GPIO		= 3,
};

enum da9188_mb_regl_cmd {
	DA9188_REGL_CMD_GET_IS_ENABLED		= 1,
	DA9188_REGL_CMD_SET_ENABLE_DISABLE	= 2,
	DA9188_REGL_CMD_GET_VOLTAGE_SEL		= 3,
	DA9188_REGL_CMD_SET_VOLTAGE_SEL		= 4,
	DA9188_REGL_CMD_GET_MODE		= 5,
	DA9188_REGL_CMD_SET_MODE		= 6,
	DA9188_REGL_CMD_GET_BYPASS		= 7,
	DA9188_REGL_CMD_SET_BYPASS		= 8,
};

enum da9188_mb_gpio_cmd {
	DA9188_GPIO_CMD_GET_DIRECTION		= 1,
	DA9188_GPIO_CMD_SET_DIRECTION		= 2,
	DA9188_GPIO_CMD_GET_VALUE		= 3,
	DA9188_GPIO_CMD_SET_VALUE		= 4,
	DA9188_GPIO_CMD_GET_OUTPUT_DRIVE	= 5,
	DA9188_GPIO_CMD_SET_OUTPUT_DRIVE	= 6,
	DA9188_GPIO_CMD_GET_MODE		= 7,
	DA9188_GPIO_CMD_GET_DRIVE_STRENGTH	= 8,
	DA9188_GPIO_CMD_SET_DRIVE_STRENGTH	= 9,
	DA9188_GPIO_CMD_GET_PULL_CONFIG		= 10,
	DA9188_GPIO_CMD_SET_PULL_CONFIG		= 11,
};

#define DA9188_MB_REQ_TARGET_MASK	GENMASK(31, 24)
#define DA9188_MB_REQ_CMD_MASK		GENMASK(23, 16)
#define DA9188_MB_REQ_ID_MASK		GENMASK(15, 0)
#define DA9188_MB_RSP_STATUS_MASK	GENMASK(31, 24)

struct da9188_mbox_data {
	u32 data[2];
};

struct da9188_mbox {
	struct cpm_iface_client *client;
};

int da9188_devm_mbox_request(struct device *dev, struct da9188_mbox *mbox);
int da9188_mbox_xfer(struct device *dev, struct da9188_mbox *mbox,
		     u8 dst_channel, u8 target, u8 cmd, u16 id_or_addr,
		     const struct da9188_mbox_data *req_data,
		     struct da9188_mbox_data *resp_data);

#endif /* _LINUX_MFD_DA9188_H */
