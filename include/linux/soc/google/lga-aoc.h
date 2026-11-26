/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Clean-room Stage 1 interface for Google Tensor G5 (Laguna) AoC.
 */

#ifndef __LINUX_SOC_GOOGLE_LGA_AOC_H
#define __LINUX_SOC_GOOGLE_LGA_AOC_H

#include <linux/notifier.h>
#include <linux/types.h>

struct device;

#define LGA_AOC_MAX_MBOXES		20
#define LGA_AOC_SERVICE_NAME_LEN	32
#define LGA_AOC_VERSION_LEN		48

#define LGA_AOC_CONTROL_MAGIC		0xa0c00a0cU
#define LGA_AOC_PARAM_MAGIC		0x0a0cda7aU

/* Service header flags */
#define LGA_AOC_SVC_FLAG_RING_PUSH	0x01000000U

/* Notifier events emitted on AoC online/offline state transitions */
#define LGA_AOC_EVENT_ONLINE		1UL
#define LGA_AOC_EVENT_OFFLINE		2UL

enum lga_aoc_service_type {
	LGA_AOC_SVC_QUEUE	= 0,
	LGA_AOC_SVC_RING	= 1,
	LGA_AOC_SVC_BUFFER	= 2,
};

enum lga_aoc_direction {
	LGA_AOC_DIR_UP		= 0,	/* AoC -> AP */
	LGA_AOC_DIR_DOWN	= 1,	/* AP -> AoC */
};

struct lga_aoc_service_info {
	char name[LGA_AOC_SERVICE_NAME_LEN];
	u32 index;
	u32 type;
	u32 flags;
	u32 mbox_index;
	u32 down_offset;
	u32 down_size;
	u32 down_slots;
	u32 up_offset;
	u32 up_size;
	u32 up_slots;
};

struct device *lga_aoc_get_device(void);
bool lga_aoc_is_online(struct device *dev);
int lga_aoc_register_notifier(struct notifier_block *nb);
int lga_aoc_unregister_notifier(struct notifier_block *nb);

int lga_aoc_find_service(struct device *dev, const char *name,
			 struct lga_aoc_service_info *out);
int lga_aoc_ring_doorbell(struct device *dev, unsigned int mbox_idx);

bool lga_aoc_ipc_can_read(struct device *dev, u32 service_idx);
bool lga_aoc_ipc_can_write(struct device *dev, u32 service_idx);

ssize_t lga_aoc_ipc_write(struct device *dev, u32 service_idx,
			  const void *buf, size_t len);
ssize_t lga_aoc_ipc_write_timeout(struct device *dev, u32 service_idx,
				  const void *buf, size_t len,
				  unsigned long timeout);
ssize_t lga_aoc_ipc_read(struct device *dev, u32 service_idx,
			 void *buf, size_t max_len);
ssize_t lga_aoc_ipc_read_timeout(struct device *dev, u32 service_idx,
				 void *buf, size_t max_len,
				 unsigned long timeout);

#if IS_REACHABLE(CONFIG_PINCTRL_GOOGLE)
int google_pinctrl_aoc_ssr_request(bool active);
#else
static inline int google_pinctrl_aoc_ssr_request(bool active)
{
	return 0;
}
#endif

#endif /* __LINUX_SOC_GOOGLE_LGA_AOC_H */
