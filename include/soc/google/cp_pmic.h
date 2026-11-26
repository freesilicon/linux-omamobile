/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Public interface for the Google cellular modem companion PMIC SPMI driver.
 */

#ifndef _SOC_GOOGLE_CP_PMIC_H
#define _SOC_GOOGLE_CP_PMIC_H

struct device;
struct device_node;
struct spmi_device;

int pmic_get_otp(struct device *dev);
void pmic_warm_reset_sequence(struct device *dev);
struct device *pmic_get_device(struct device_node *node);
struct spmi_device *pmic_get_spmi_device(struct device_node *node);

#endif /* _SOC_GOOGLE_CP_PMIC_H */
