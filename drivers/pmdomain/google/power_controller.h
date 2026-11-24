/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2024-2025 Google LLC */

#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <linux/pm_domain.h>

#include <soc/google/goog_mba_cpm_iface.h>

enum gpu_dts_version { GPU_DTS_VERSION_1 = 0, GPU_DTS_VERSION_2, GPU_DTS_VERSION_NUM };
struct gpu_core_logic_pd { enum gpu_dts_version gpu_dts_version; };

enum power_state {
	PD_STATE_OFF,	/* PM domain is off */
	PD_STATE_ON,	/* PM domain is on */
};

struct power_domain {
	struct power_controller *power_controller;
	struct generic_pm_domain genpd;
	const char *name;
	bool is_top_pd;
	bool use_smc;
	enum power_state state;
	bool boot_stay_on;
	u32 genpd_sswrp_id;
	u32 cpm_lpcm_subdomain_id;

	union {
		struct gpu_core_logic_pd gpu_core_logic;
		struct {
			struct completion cpm_resp_done;
			int cpm_err;
			/*
			 * sswrp id that CPM uses internally has different
			 * value depending on .is_top_pd:
			 * - for top power domain, it is cpm_lpb_sswrp_id.
			 * - for sub power domain, it is cpm_lpcm_sswrp_id.
			 */
			union {
				u32 cpm_lpcm_sswrp_id;
				u32 cpm_lpb_sswrp_id;
			};
		};
	};
};

struct power_controller {
	struct device *dev;
	struct cpm_iface_client *client;
	struct power_domain *pds;
	struct dentry *debugfs_root;

	u32 pd_count;
};

#endif /* POWER_CONTROLLER_H */
