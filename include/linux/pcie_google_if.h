/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Root Complex control interface for Google Laguna (Tensor G5) PCIe ports.
 */

#ifndef _LINUX_PCIE_GOOGLE_IF_H
#define _LINUX_PCIE_GOOGLE_IF_H

#include <linux/errno.h>
#include <linux/types.h>

#define GPCIE_NUM_LINK_SPEEDS	4

enum google_pcie_callback_type {
	GPCIE_CB_CPL_TIMEOUT,
	GPCIE_CB_LINK_DOWN,
	GPCIE_CB_FATAL_ERR,
	GPCIE_CB_UNKNOWN,
};

struct google_pcie_power_stats {
	u64 count;
	u64 duration;
	u64 last_entry_ms;
};

struct google_pcie_link_duration_stats {
	int last_link_speed;
	struct link_entry {
		u64 count;
		u64 duration;
		u64 last_entry_ts;
	} speed[GPCIE_NUM_LINK_SPEEDS];
};

struct google_pcie_link_stats {
	u64 link_up_failure_count;
	u64 link_recovery_failure_count;
	u64 link_down_irq_count;
	u64 complete_timeout_irq_count;
	u64 link_up_time_avg;
};

typedef void (*google_pcie_callback_func)(enum google_pcie_callback_type type,
					  void *priv);

#if IS_ENABLED(CONFIG_PCIE_GOOGLE_LGA)

int google_pcie_rc_poweron(int num);
int google_pcie_rc_poweroff(int num);
void google_pcie_rc_prepare_for_forced_poweroff(int num);
int google_pcie_poweron_withspeed(int num, unsigned int speed);
int google_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr);
int google_pcie_link_state(int num);
int google_pcie_link_status(int num);
int google_pcie_inbound_atu_cfg(int ch_num, unsigned long long src_addr,
				unsigned long long dst_addr,
				unsigned long size, int atu_index);
int google_pcie_get_max_link_speed(int num);
int google_pcie_get_max_link_width(int num);
int google_pcie_rc_change_link_speed(int num, unsigned int speed);
int google_pcie_register_callback(int num, google_pcie_callback_func cb_func,
				  void *priv);
int google_pcie_unregister_callback(int num);
void google_pcie_dump_debug(int num);
bool google_pcie_is_link_down(int num);
bool google_pcie_is_cpl_timeout(int num);

#else /* !CONFIG_PCIE_GOOGLE_LGA */

static inline int google_pcie_rc_poweron(int num)
{
	return -ENODEV;
}

static inline int google_pcie_rc_poweroff(int num)
{
	return -ENODEV;
}

static inline void google_pcie_rc_prepare_for_forced_poweroff(int num)
{
}

static inline int google_pcie_poweron_withspeed(int num, unsigned int speed)
{
	return -ENODEV;
}

static inline int google_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr)
{
	return -ENODEV;
}

static inline int google_pcie_link_state(int num)
{
	return -ENODEV;
}

static inline int google_pcie_link_status(int num)
{
	return -ENODEV;
}

static inline int google_pcie_inbound_atu_cfg(int ch_num,
					      unsigned long long src_addr,
					      unsigned long long dst_addr,
					      unsigned long size, int atu_index)
{
	return -ENODEV;
}

static inline int google_pcie_get_max_link_speed(int num)
{
	return -ENODEV;
}

static inline int google_pcie_get_max_link_width(int num)
{
	return -ENODEV;
}

static inline int google_pcie_rc_change_link_speed(int num, unsigned int speed)
{
	return -ENODEV;
}

static inline int google_pcie_register_callback(int num,
						google_pcie_callback_func cb_func,
						void *priv)
{
	return -ENODEV;
}

static inline int google_pcie_unregister_callback(int num)
{
	return -ENODEV;
}

static inline void google_pcie_dump_debug(int num)
{
}

static inline bool google_pcie_is_link_down(int num)
{
	return false;
}

static inline bool google_pcie_is_cpl_timeout(int num)
{
	return false;
}

#endif /* CONFIG_PCIE_GOOGLE_LGA */

#endif /* _LINUX_PCIE_GOOGLE_IF_H */
