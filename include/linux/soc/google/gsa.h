/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google GSA (security core) interface for other drivers
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __LINUX_SOC_GOOGLE_GSA_H
#define __LINUX_SOC_GOOGLE_GSA_H

#include <linux/types.h>

struct device;

/* enum gsa_aoc_state: what GSA_AOC_GET_STATE returns */
#define GSA_AOC_STATE_INACTIVE		0	/* not loaded, in reset */
#define GSA_AOC_STATE_LOADED		1	/* loaded, in reset */
#define GSA_AOC_STATE_RUNNING		2

/* enum gsa_aoc_cmd */
#define GSA_AOC_GET_STATE		0
#define GSA_AOC_START			1
#define GSA_AOC_SHUTDOWN		4
#define GSA_AOC_RELEASE_RESET		5
#define GSA_AOC_RESET			6

/* The AoC/TPU image headers the GSA authenticates are always 4 KiB */
#define GSA_AOC_HEADER_SIZE		4096
#define GSA_TPU_HEADER_SIZE		4096

/* enum gsa_tpu_state: what GSA_TPU_GET_STATE returns */
enum gsa_tpu_state {
	GSA_TPU_STATE_INACTIVE = 0,	/* not loaded, in reset */
	GSA_TPU_STATE_LOADED = 1,	/* loaded, in reset */
	GSA_TPU_STATE_RUNNING = 2,
	GSA_TPU_STATE_SUSPENDED = 3,
};

/* enum gsa_tpu_cmd */
enum gsa_tpu_cmd {
	GSA_TPU_GET_STATE = 0,
	GSA_TPU_START = 1,
	GSA_TPU_SUSPEND = 2,
	GSA_TPU_RESUME = 3,
	GSA_TPU_SHUTDOWN = 4,
};

struct device *gsa_get_device(struct device *client);
void gsa_put_device(struct device *gsa);

int gsa_load_aoc_fw_image(struct device *gsa, dma_addr_t header,
			  phys_addr_t body);
int gsa_send_aoc_cmd(struct device *gsa, u32 cmd);
int gsa_unload_aoc_fw_image(struct device *gsa);

int gsa_load_tpu_fw_image(struct device *gsa, dma_addr_t header,
			  phys_addr_t body);
int gsa_send_tpu_cmd(struct device *gsa, enum gsa_tpu_cmd cmd);
int gsa_unload_tpu_fw_image(struct device *gsa);

#endif /* __LINUX_SOC_GOOGLE_GSA_H */
