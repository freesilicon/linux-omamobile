/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Public interface for the S5910 SPMI modem clock buffer driver.
 */

#ifndef _LINUX_S5910_H
#define _LINUX_S5910_H

struct device;
struct device_node;
struct spmi_device;

struct device *s5910_get_device(struct device_node *node);
struct spmi_device *s5910_get_spmi_device(struct device_node *node);
int s5910_shutdown_sequence(struct device *dev);
int s5910_turn_on_sequence(struct device *dev);
int s5910_check_lpm_mode(struct device *dev);

#endif /* _LINUX_S5910_H */
