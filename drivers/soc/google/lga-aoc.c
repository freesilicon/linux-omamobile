// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room Stage 1 Always-on Compute (AoC) boot and IPC driver for
 * Google Tensor G5 (Laguna).
 *
 * Implements step-by-step AoC firmware validation, GSA authentication over
 * google,mba-ctrl v2.0, Trusty hwmgr.aoc lifecycle control, 20-channel
 * google,mba-ctrl v2.0 doorbell messaging, and shared-carveout IPC service
 * discovery and ring/queue transport scaffolding.
 *
 * No firmware is loaded automatically at probe unless autoload=1 is passed.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/random.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/google/gsa.h>
#include <linux/soc/google/lga-aoc.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define LGA_AOC_AUTH_MAGIC		0x00434f41U	/* "AOC" */
#define LGA_AOC_SUPERBIN_MAGIC		0xaabbccddU
#define LGA_AOC_DRAM_IOVA_BASE		0x98000000U

#define LGA_AOC_HDR_FMT_RDO		1U
#define LGA_AOC_HDR_FMT_PQ		2U
#define LGA_AOC_HDR_LEGACY_SIZE		GSA_AOC_HEADER_SIZE
#define LGA_AOC_HDR_PQ_SIZE		GSA_AOC_PQ_HEADER_SIZE

/* Offsets of sub-header magic within RDO and PQ signed containers */
#define LGA_AOC_RDO_SUBHDR_OFF		0x864U
#define LGA_AOC_PQ_SUBHDR_OFF		0x4630U

/* Heaps reserved at the top of the AoC carve-out */
#define LGA_AOC_SENSOR_HEAP_SIZE	SZ_4M
#define LGA_AOC_PLAYBACK_HEAP_SIZE	SZ_16K
#define LGA_AOC_CAPTURE_HEAP_SIZE	SZ_64K

#define LGA_AOC_MAX_SERVICES		256U
#define LGA_AOC_START_TIMEOUT_US	(5U * USEC_PER_SEC)
#define LGA_AOC_START_POLL_US		20000U
#define LGA_AOC_DEFAULT_FW		"aoc.bin"
#define LGA_AOC_DEFAULT_COREDUMP_MBOX	19U

/* Service header flag bitfields */
#define LGA_AOC_SVC_TYPE_MASK		GENMASK(18, 16)
#define LGA_AOC_SVC_IRQ_MASK		GENMASK(23, 19)

/* Boot parameter TLV keys expected by AoC firmware */
enum lga_aoc_param_key {
	LGA_AOC_PARAM_BOARD_ID			= 0x1001,
	LGA_AOC_PARAM_BOARD_REV			= 0x1002,
	LGA_AOC_PARAM_SRAM_REPAIRED		= 0x1003,
	LGA_AOC_PARAM_CARVEOUT_ADDR		= 0x1005,
	LGA_AOC_PARAM_CARVEOUT_SIZE		= 0x1006,
	LGA_AOC_PARAM_SENSOR_HEAP_ADDR		= 0x1007,
	LGA_AOC_PARAM_SENSOR_HEAP_SIZE		= 0x1008,
	LGA_AOC_PARAM_FORCE_VNOM		= 0x1009,
	LGA_AOC_PARAM_DISABLE_MM		= 0x100a,
	LGA_AOC_PARAM_ENABLE_UART		= 0x100b,
	LGA_AOC_PARAM_PLAYBACK_HEAP_ADDR	= 0x100c,
	LGA_AOC_PARAM_PLAYBACK_HEAP_SIZE	= 0x100d,
	LGA_AOC_PARAM_CAPTURE_HEAP_ADDR		= 0x100e,
	LGA_AOC_PARAM_CAPTURE_HEAP_SIZE		= 0x100f,
	LGA_AOC_PARAM_RAND_SEED			= 0x1011,
	LGA_AOC_PARAM_GNSS_TYPE			= 0x1014,
};

struct lga_aoc_legacy_hdr_v1 {
	u8 signature[512];
	u8 key[512];
	__le32 magic;
	__le32 generation;
	__le32 rollback_info;
	__le32 length;
	u8 flags[16];
	u8 hash[32];
	u8 chip_id[32];
	u8 auth_config[256];
	u8 image_config[256];
} __packed;

struct lga_aoc_legacy_hdr_v2 {
	u8 signature[512];
	u8 key[512];
	__le32 magic;
	__le32 generation;
	__le32 rollback_info;
	__le32 length;
	u8 flags[16];
	u8 hash[64];
	u8 chip_id[32];
	u8 auth_config[256];
	u8 image_config[256];
} __packed;

struct lga_aoc_superbin_hdr {
	__le32 magic;
	__le32 release_type;
	__le32 reserved0;
	__le32 image_size;
	__le32 bootloader_low;
	__le32 bootloader_high;
	__le32 bootloader_offset;
	__le32 bootloader_size;
	__le32 uuid_table_offset;
	__le32 uuid_table_size;
	u8 version_prefix[8];
	u8 version_string[64];
} __packed;

struct lga_aoc_control_block {
	__le32 magic;
	__le32 fw_version;
	__le32 hw_version;
	__le32 status;
	__le64 supported_features;
	__le32 services;
	__le32 service_size;
	__le32 services_offset;
	u8 fw_version_name[LGA_AOC_VERSION_LEN];
	u8 hw_version_name[LGA_AOC_VERSION_LEN];
	__le64 system_clock_offset;
} __packed;

struct lga_aoc_ipc_region {
	__le32 offset;
	__le32 size;
	__le32 slots;
	__le32 tx;
	__le32 rx;
	__le32 wp;
	__le32 rp;
} __packed;

struct lga_aoc_ipc_service_hdr {
	char name[LGA_AOC_SERVICE_NAME_LEN];
	__le32 flags;
	struct lga_aoc_ipc_region regions[2];
} __packed;

struct lga_aoc_ipc_msg_hdr {
	__le16 length;
	__le16 r1;
	__le16 r2;
	__le16 r3;
} __packed;

struct lga_aoc_mbox_slot {
	struct lga_aoc *aoc;
	struct mbox_client client;
	struct mbox_chan *chan;
	unsigned int index;
	atomic_t rx_count;
	atomic_t tx_count;
	u32 last_rx_word;
};

enum lga_aoc_supply_id {
	LGA_AOC_SUPPLY_1V8	= 0,
	LGA_AOC_SUPPLY_3V3	= 1,
	LGA_AOC_SUPPLY_MAG	= 2,
	LGA_AOC_NUM_SUPPLIES	= 3,
};

struct lga_aoc {
	struct device *dev;
	struct device *gsa;
	struct device *pd_devs[2];
	struct device_link *pd_links[2];

	phys_addr_t dram_base;
	size_t dram_size;
	void *dram;

	void *auth_hdr_virt;
	dma_addr_t auth_hdr_dma;
	size_t auth_hdr_alloc_size;

	struct regulator *supplies[LGA_AOC_NUM_SUPPLIES];
	bool supplies_enabled;

	struct lga_aoc_mbox_slot mboxes[LGA_AOC_MAX_MBOXES];
	unsigned int num_mboxes;
	u32 coredump_mbox;

	u32 board_id;
	u32 board_rev;
	u32 gnss_type;
	const char *board_cfg;
	const char *fw_name;

	u32 ipc_offset;
	size_t hdr_size;
	char fw_version[73];

	struct lga_aoc_service_info *services;
	unsigned int num_services;
	u32 services_offset;
	u32 service_size;

	bool loaded;
	bool online;

	struct mutex lock; /* serializes load, lifecycle, and IPC access */
	wait_queue_head_t ipc_wq;
	struct dentry *debugfs;
};

static bool autoload;
module_param(autoload, bool, 0444);
MODULE_PARM_DESC(autoload, "Load and start AoC firmware automatically at probe");

static bool allow_coredump_ping;
module_param(allow_coredump_ping, bool, 0644);
MODULE_PARM_DESC(allow_coredump_ping,
		 "Allow sending debugfs doorbell pings on the coredump mailbox");

static const char * const lga_aoc_supply_names[LGA_AOC_NUM_SUPPLIES] = {
	[LGA_AOC_SUPPLY_1V8] = "sensor-1v8",
	[LGA_AOC_SUPPLY_3V3] = "sensor-3v3",
	[LGA_AOC_SUPPLY_MAG] = "sensor-mag",
};

static const char *lga_aoc_gsa_state_str(int state)
{
	switch (state) {
	case GSA_AOC_STATE_INACTIVE:
		return "inactive";
	case GSA_AOC_STATE_LOADED:
		return "loaded";
	case GSA_AOC_STATE_RUNNING:
		return "running";
	case GSA_AOC_STATE_SUSPENDED:
		return "suspended";
	default:
		return "unknown";
	}
}

static const char *lga_aoc_svc_type_str(u32 type)
{
	switch (type) {
	case LGA_AOC_SVC_QUEUE:
		return "queue";
	case LGA_AOC_SVC_RING:
		return "ring";
	case LGA_AOC_SVC_BUFFER:
		return "buffer";
	default:
		return "unknown";
	}
}

static void lga_aoc_mbox_rx_cb(struct mbox_client *cl, void *mssg)
{
	struct lga_aoc_mbox_slot *slot =
		container_of(cl, struct lga_aoc_mbox_slot, client);
	struct lga_aoc *aoc = slot->aoc;

	atomic_inc(&slot->rx_count);
	if (mssg)
		slot->last_rx_word = READ_ONCE(*(const u32 *)mssg);

	if (slot->index == aoc->coredump_mbox)
		dev_warn(aoc->dev, "AoC coredump mailbox %u signalled\n",
			 slot->index);

	wake_up_all(&aoc->ipc_wq);
}

static int lga_aoc_set_supplies(struct lga_aoc *aoc, bool enable)
{
	int i, ret;

	if (enable == aoc->supplies_enabled)
		return 0;

	if (enable) {
		for (i = 0; i < LGA_AOC_NUM_SUPPLIES; i++) {
			if (!aoc->supplies[i])
				continue;
			ret = regulator_enable(aoc->supplies[i]);
			if (ret) {
				dev_err(aoc->dev,
					"failed to enable %s supply: %d\n",
					lga_aoc_supply_names[i], ret);
				while (i--) {
					if (aoc->supplies[i])
						regulator_disable(aoc->supplies[i]);
				}
				return ret;
			}
		}
		aoc->supplies_enabled = true;
		return 0;
	}

	for (i = LGA_AOC_NUM_SUPPLIES - 1; i >= 0; i--) {
		if (aoc->supplies[i])
			regulator_disable(aoc->supplies[i]);
	}
	aoc->supplies_enabled = false;
	return 0;
}

static int lga_aoc_detect_header(const struct firmware *fw, size_t *hdr_size)
{
	const struct lga_aoc_legacy_hdr_v1 *h1;
	const struct lga_aoc_legacy_hdr_v2 *h2;
	u32 fmt, magic;

	if (!fw || !fw->data || fw->size < LGA_AOC_HDR_LEGACY_SIZE)
		return -EINVAL;

	fmt = le32_to_cpup((const __le32 *)fw->data);
	if (fmt == LGA_AOC_HDR_FMT_RDO &&
	    fw->size >= LGA_AOC_HDR_LEGACY_SIZE + sizeof(struct lga_aoc_superbin_hdr)) {
		magic = le32_to_cpup((const __le32 *)(fw->data + LGA_AOC_RDO_SUBHDR_OFF));
		if (magic == LGA_AOC_AUTH_MAGIC) {
			*hdr_size = LGA_AOC_HDR_LEGACY_SIZE;
			return 0;
		}
	}

	if (fmt == LGA_AOC_HDR_FMT_PQ &&
	    fw->size >= LGA_AOC_HDR_PQ_SIZE + sizeof(struct lga_aoc_superbin_hdr)) {
		magic = le32_to_cpup((const __le32 *)(fw->data + LGA_AOC_PQ_SUBHDR_OFF));
		if (magic == LGA_AOC_AUTH_MAGIC) {
			*hdr_size = LGA_AOC_HDR_PQ_SIZE;
			return 0;
		}
	}

	h1 = (const struct lga_aoc_legacy_hdr_v1 *)fw->data;
	if (le32_to_cpu(h1->magic) == LGA_AOC_AUTH_MAGIC &&
	    le32_to_cpu(h1->generation) == 1) {
		*hdr_size = LGA_AOC_HDR_LEGACY_SIZE;
		return 0;
	}

	h2 = (const struct lga_aoc_legacy_hdr_v2 *)fw->data;
	if (le32_to_cpu(h2->magic) == LGA_AOC_AUTH_MAGIC &&
	    le32_to_cpu(h2->generation) == 2) {
		*hdr_size = LGA_AOC_HDR_LEGACY_SIZE;
		return 0;
	}

	return -EINVAL;
}

static int lga_aoc_parse_fw(struct lga_aoc *aoc, const struct firmware *fw)
{
	const struct lga_aoc_superbin_hdr *sb;
	size_t hdr_size = 0, body_size, heaps_size;
	u32 ipc_off;
	int ret;

	ret = lga_aoc_detect_header(fw, &hdr_size);
	if (ret) {
		dev_err(aoc->dev, "unrecognized AoC signed header\n");
		return ret;
	}

	if (fw->size <= hdr_size + sizeof(*sb))
		return -EINVAL;

	body_size = fw->size - hdr_size;
	heaps_size = LGA_AOC_SENSOR_HEAP_SIZE + LGA_AOC_PLAYBACK_HEAP_SIZE +
		     LGA_AOC_CAPTURE_HEAP_SIZE;

	if (body_size >= aoc->dram_size || body_size + heaps_size > aoc->dram_size) {
		dev_err(aoc->dev, "AoC body %#zx exceeds carve-out %#zx\n",
			body_size, aoc->dram_size);
		return -ERANGE;
	}

	sb = (const struct lga_aoc_superbin_hdr *)(fw->data + hdr_size);
	if (le32_to_cpu(sb->magic) != LGA_AOC_SUPERBIN_MAGIC) {
		dev_err(aoc->dev, "invalid AoC superbin magic %#x\n",
			le32_to_cpu(sb->magic));
		return -EINVAL;
	}

	ipc_off = le32_to_cpu(sb->image_size);
	if (ipc_off < body_size || ipc_off + SZ_64K > aoc->dram_size - heaps_size) {
		dev_err(aoc->dev, "invalid AoC IPC offset %#x (body %#zx)\n",
			ipc_off, body_size);
		return -EINVAL;
	}

	aoc->hdr_size = hdr_size;
	aoc->ipc_offset = ipc_off;
	snprintf(aoc->fw_version, sizeof(aoc->fw_version), "%.8s%.64s",
		 sb->version_prefix, sb->version_string);

	dev_info(aoc->dev,
		 "parsed AoC image: hdr %#zx body %#zx ipc +%#x version '%s'\n",
		 hdr_size, body_size, ipc_off, aoc->fw_version);
	return 0;
}

static u32 lga_aoc_dram_to_iova(struct lga_aoc *aoc, phys_addr_t pa)
{
	return LGA_AOC_DRAM_IOVA_BASE + (u32)(pa - aoc->dram_base);
}

static void lga_aoc_clear_heaps(struct lga_aoc *aoc)
{
	size_t total = LGA_AOC_SENSOR_HEAP_SIZE + LGA_AOC_PLAYBACK_HEAP_SIZE +
		       LGA_AOC_CAPTURE_HEAP_SIZE;

	if (aoc->dram_size <= total)
		return;

	memset(aoc->dram + aoc->dram_size - total, 0, total);
	dma_wmb();
}

static void lga_aoc_write_boot_params(struct lga_aoc *aoc)
{
	phys_addr_t top = aoc->dram_base + aoc->dram_size;
	phys_addr_t sensor_pa = top - LGA_AOC_SENSOR_HEAP_SIZE;
	phys_addr_t playback_pa = sensor_pa - LGA_AOC_PLAYBACK_HEAP_SIZE;
	phys_addr_t capture_pa = playback_pa - LGA_AOC_CAPTURE_HEAP_SIZE;
	const u32 kv[][2] = {
		{ LGA_AOC_PARAM_BOARD_ID, aoc->board_id },
		{ LGA_AOC_PARAM_BOARD_REV, aoc->board_rev },
		{ LGA_AOC_PARAM_SRAM_REPAIRED, 0 },
		{ LGA_AOC_PARAM_CARVEOUT_ADDR, (u32)aoc->dram_base },
		{ LGA_AOC_PARAM_CARVEOUT_SIZE, (u32)aoc->dram_size },
		{ LGA_AOC_PARAM_SENSOR_HEAP_ADDR,
		  lga_aoc_dram_to_iova(aoc, sensor_pa) },
		{ LGA_AOC_PARAM_SENSOR_HEAP_SIZE, LGA_AOC_SENSOR_HEAP_SIZE },
		{ LGA_AOC_PARAM_PLAYBACK_HEAP_ADDR,
		  lga_aoc_dram_to_iova(aoc, playback_pa) },
		{ LGA_AOC_PARAM_PLAYBACK_HEAP_SIZE, LGA_AOC_PLAYBACK_HEAP_SIZE },
		{ LGA_AOC_PARAM_CAPTURE_HEAP_ADDR,
		  lga_aoc_dram_to_iova(aoc, capture_pa) },
		{ LGA_AOC_PARAM_CAPTURE_HEAP_SIZE, LGA_AOC_CAPTURE_HEAP_SIZE },
		{ LGA_AOC_PARAM_FORCE_VNOM, 0 },
		{ LGA_AOC_PARAM_DISABLE_MM, 0 },
		{ LGA_AOC_PARAM_ENABLE_UART, 0 },
		{ LGA_AOC_PARAM_RAND_SEED, get_random_u32() },
		{ LGA_AOC_PARAM_GNSS_TYPE, aoc->gnss_type },
	};
	__le32 *p = (__le32 *)(aoc->dram + aoc->ipc_offset);
	unsigned int i;

	*p++ = cpu_to_le32(LGA_AOC_PARAM_MAGIC);
	*p++ = cpu_to_le32(ARRAY_SIZE(kv));
	*p++ = cpu_to_le32(12U + ARRAY_SIZE(kv) * 3U * sizeof(u32));
	for (i = 0; i < ARRAY_SIZE(kv); i++) {
		*p++ = cpu_to_le32(kv[i][0]);
		*p++ = cpu_to_le32(sizeof(u32));
		*p++ = cpu_to_le32(kv[i][1]);
	}
	dma_wmb();
}

static int lga_aoc_discover_services(struct lga_aoc *aoc)
{
	const struct lga_aoc_control_block *cb;
	struct lga_aoc_service_info *list;
	u32 count, svc_size, svc_off, i;
	size_t table_end;

	cb = (const struct lga_aoc_control_block *)(aoc->dram + aoc->ipc_offset);
	dma_rmb();

	if (le32_to_cpu(cb->magic) != LGA_AOC_CONTROL_MAGIC)
		return -ENODEV;

	count = le32_to_cpu(cb->services);
	svc_size = le32_to_cpu(cb->service_size);
	svc_off = le32_to_cpu(cb->services_offset);

	if (!count || count > LGA_AOC_MAX_SERVICES ||
	    svc_size < sizeof(struct lga_aoc_ipc_service_hdr)) {
		dev_err(aoc->dev, "invalid AoC service table (n=%u size=%u)\n",
			count, svc_size);
		return -EINVAL;
	}

	table_end = (size_t)aoc->ipc_offset + svc_off + (size_t)count * svc_size;
	if (table_end > aoc->dram_size) {
		dev_err(aoc->dev, "AoC service table exceeds carve-out\n");
		return -ERANGE;
	}

	list = kcalloc(count, sizeof(*list), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		const struct lga_aoc_ipc_service_hdr *hdr;
		u32 flags;

		hdr = (const struct lga_aoc_ipc_service_hdr *)
			(aoc->dram + aoc->ipc_offset + svc_off + i * svc_size);
		flags = le32_to_cpu(hdr->flags);

		memcpy(list[i].name, hdr->name, LGA_AOC_SERVICE_NAME_LEN - 1);
		list[i].name[LGA_AOC_SERVICE_NAME_LEN - 1] = '\0';
		list[i].index = i;
		list[i].type = FIELD_GET(LGA_AOC_SVC_TYPE_MASK, flags);
		list[i].mbox_index = FIELD_GET(LGA_AOC_SVC_IRQ_MASK, flags);
		list[i].down_offset = le32_to_cpu(hdr->regions[LGA_AOC_DIR_DOWN].offset);
		list[i].down_size = le32_to_cpu(hdr->regions[LGA_AOC_DIR_DOWN].size);
		list[i].down_slots = le32_to_cpu(hdr->regions[LGA_AOC_DIR_DOWN].slots);
		list[i].up_offset = le32_to_cpu(hdr->regions[LGA_AOC_DIR_UP].offset);
		list[i].up_size = le32_to_cpu(hdr->regions[LGA_AOC_DIR_UP].size);
		list[i].up_slots = le32_to_cpu(hdr->regions[LGA_AOC_DIR_UP].slots);
	}

	kfree(aoc->services);
	aoc->services = list;
	aoc->num_services = count;
	aoc->services_offset = svc_off;
	aoc->service_size = svc_size;
	aoc->online = true;

	dev_info(aoc->dev, "AoC online: %u services published\n", count);
	return 0;
}

static int lga_aoc_load_locked(struct lga_aoc *aoc, const char *name)
{
	const struct firmware *fw;
	size_t body_size;
	int state, ret;

	if (aoc->loaded)
		return -EALREADY;

	state = gsa_send_aoc_cmd(aoc->gsa, GSA_AOC_GET_STATE);
	if (state < 0)
		return state;

	if (state != GSA_AOC_STATE_INACTIVE) {
		dev_err(aoc->dev,
			"refusing load: GSA AoC state is %d (%s), expected inactive\n",
			state, lga_aoc_gsa_state_str(state));
		return -EBUSY;
	}

	ret = request_firmware(&fw, name, aoc->dev);
	if (ret) {
		dev_err(aoc->dev, "request_firmware(%s) failed: %d\n", name, ret);
		return ret;
	}

	ret = lga_aoc_parse_fw(aoc, fw);
	if (ret)
		goto out_rel;

	if (!aoc->auth_hdr_virt || aoc->auth_hdr_alloc_size < aoc->hdr_size) {
		/*
		 * Keep DMA auth header buffer device-managed so GSA never
		 * accesses freed memory if an authentication command times out.
		 */
		aoc->auth_hdr_virt = dmam_alloc_coherent(aoc->gsa, aoc->hdr_size,
							 &aoc->auth_hdr_dma,
							 GFP_KERNEL);
		if (!aoc->auth_hdr_virt) {
			ret = -ENOMEM;
			goto out_rel;
		}
		aoc->auth_hdr_alloc_size = aoc->hdr_size;
	}

	lga_aoc_clear_heaps(aoc);

	body_size = fw->size - aoc->hdr_size;
	memcpy(aoc->dram, fw->data + aoc->hdr_size, body_size);
	dma_wmb();

	memcpy(aoc->auth_hdr_virt, fw->data, aoc->hdr_size);
	dma_wmb();

	ret = gsa_load_aoc_fw_image(aoc->gsa, aoc->auth_hdr_dma, aoc->dram_base);
	if (ret) {
		dev_err(aoc->dev, "GSA AoC image authentication failed: %d\n", ret);
		goto out_rel;
	}

	lga_aoc_write_boot_params(aoc);
	aoc->loaded = true;
	dev_info(aoc->dev, "AoC firmware '%s' authenticated and loaded\n", name);

out_rel:
	release_firmware(fw);
	return ret;
}

static u32 lga_aoc_read_cb_magic(const struct lga_aoc_control_block *cb)
{
	return le32_to_cpu(READ_ONCE(cb->magic));
}

static int lga_aoc_start_locked(struct lga_aoc *aoc)
{
	const struct lga_aoc_control_block *cb;
	u32 magic = 0;
	int ret;

	if (!aoc->loaded)
		return -EINVAL;
	if (aoc->online)
		return 0;

	ret = lga_aoc_set_supplies(aoc, true);
	if (ret)
		return ret;

	ret = gsa_aoc_start(aoc->gsa);
	if (ret) {
		lga_aoc_set_supplies(aoc, false);
		return ret;
	}

	cb = (const struct lga_aoc_control_block *)(aoc->dram + aoc->ipc_offset);
	ret = read_poll_timeout(lga_aoc_read_cb_magic, magic,
				magic == LGA_AOC_CONTROL_MAGIC,
				LGA_AOC_START_POLL_US, LGA_AOC_START_TIMEOUT_US,
				false, cb);
	if (ret) {
		dev_err(aoc->dev,
			"timed out waiting for AoC control block (magic %#x)\n",
			magic);
		return ret;
	}

	return lga_aoc_discover_services(aoc);
}

static int lga_aoc_stop_locked(struct lga_aoc *aoc)
{
	int ret;

	ret = gsa_aoc_stop(aoc->gsa);
	lga_aoc_set_supplies(aoc, false);

	kfree(aoc->services);
	aoc->services = NULL;
	aoc->num_services = 0;
	aoc->online = false;
	if (!ret)
		aoc->loaded = false;

	return ret;
}

int lga_aoc_find_service(struct device *dev, const char *name,
			 struct lga_aoc_service_info *out)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	unsigned int i;
	int ret = -ENOENT;

	if (!aoc || !name || !out)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	if (!aoc->online) {
		ret = -ENODEV;
		goto out;
	}

	for (i = 0; i < aoc->num_services; i++) {
		if (!strcmp(aoc->services[i].name, name)) {
			*out = aoc->services[i];
			ret = 0;
			break;
		}
	}
out:
	mutex_unlock(&aoc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lga_aoc_find_service);

int lga_aoc_ring_doorbell(struct device *dev, unsigned int mbox_idx)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	u32 word = 1;
	int ret;

	if (!aoc || mbox_idx >= aoc->num_mboxes || !aoc->mboxes[mbox_idx].chan)
		return -EINVAL;

	ret = mbox_send_message(aoc->mboxes[mbox_idx].chan, &word);
	if (ret < 0)
		return ret;

	atomic_inc(&aoc->mboxes[mbox_idx].tx_count);
	return 0;
}
EXPORT_SYMBOL_GPL(lga_aoc_ring_doorbell);

static struct lga_aoc_ipc_service_hdr *
lga_aoc_service_hdr(struct lga_aoc *aoc, u32 service_idx)
{
	if (!aoc->online || service_idx >= aoc->num_services)
		return NULL;

	return (struct lga_aoc_ipc_service_hdr *)
		(aoc->dram + aoc->ipc_offset + aoc->services_offset +
		 (size_t)service_idx * aoc->service_size);
}

static bool lga_aoc_region_bounds_ok(struct lga_aoc *aoc, u32 offset,
				     u32 size, u32 slots)
{
	u64 total = (u64)size * slots;

	if (!size || !slots)
		return false;
	return (u64)aoc->ipc_offset + offset + total <= aoc->dram_size;
}

ssize_t lga_aoc_ipc_write(struct device *dev, u32 service_idx,
			  const void *buf, size_t len)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	struct lga_aoc_ipc_service_hdr *hdr;
	struct lga_aoc_ipc_region *reg;
	struct lga_aoc_ipc_msg_hdr mhdr = {};
	u32 offset, size, slots, tx, rx, wp, mbox_idx, type;
	void *slot_ptr;
	int ret;

	if (!aoc || !buf || !len)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	hdr = lga_aoc_service_hdr(aoc, service_idx);
	if (!hdr) {
		ret = -ENODEV;
		goto out;
	}

	type = aoc->services[service_idx].type;
	mbox_idx = aoc->services[service_idx].mbox_index;
	reg = &hdr->regions[LGA_AOC_DIR_DOWN];
	offset = le32_to_cpu(reg->offset);
	size = le32_to_cpu(reg->size);
	slots = le32_to_cpu(reg->slots);

	if (!lga_aoc_region_bounds_ok(aoc, offset, size, slots)) {
		ret = -EIO;
		goto out;
	}

	tx = le32_to_cpu(READ_ONCE(reg->tx));
	rx = le32_to_cpu(READ_ONCE(reg->rx));
	wp = le32_to_cpu(READ_ONCE(reg->wp));

	if (type == LGA_AOC_SVC_QUEUE) {
		if (tx - rx >= slots) {
			ret = -ENOSPC;
			goto out;
		}
		if (size <= sizeof(mhdr) || len > size - sizeof(mhdr) ||
		    len > U16_MAX) {
			ret = -EMSGSIZE;
			goto out;
		}
		wp %= slots;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset + (size_t)wp * size;
		mhdr.length = cpu_to_le16(len);
		memcpy(slot_ptr, &mhdr, sizeof(mhdr));
		memcpy(slot_ptr + sizeof(mhdr), buf, len);
		dma_wmb();
		WRITE_ONCE(reg->wp, cpu_to_le32((wp + 1U) % slots));
		WRITE_ONCE(reg->tx, cpu_to_le32(tx + 1U));
	} else if (type == LGA_AOC_SVC_RING) {
		size_t avail = size - min_t(size_t, tx - rx, size);
		size_t first;

		if (len > avail) {
			ret = -ENOSPC;
			goto out;
		}
		wp %= size;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset;
		first = min_t(size_t, len, size - wp);
		memcpy(slot_ptr + wp, buf, first);
		if (len > first)
			memcpy(slot_ptr, buf + first, len - first);
		dma_wmb();
		WRITE_ONCE(reg->wp, cpu_to_le32((wp + len) % size));
		WRITE_ONCE(reg->tx, cpu_to_le32(tx + len));
	} else {
		ret = -EOPNOTSUPP;
		goto out;
	}

	dma_wmb();
	if (mbox_idx < aoc->num_mboxes)
		lga_aoc_ring_doorbell(dev, mbox_idx);

	ret = len;
out:
	mutex_unlock(&aoc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_write);

ssize_t lga_aoc_ipc_read(struct device *dev, u32 service_idx,
			 void *buf, size_t max_len)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	struct lga_aoc_ipc_service_hdr *hdr;
	struct lga_aoc_ipc_region *reg;
	struct lga_aoc_ipc_msg_hdr mhdr;
	u32 offset, size, slots, tx, rx, rp, type;
	void *slot_ptr;
	size_t msg_len;
	int ret;

	if (!aoc || !buf || !max_len)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	hdr = lga_aoc_service_hdr(aoc, service_idx);
	if (!hdr) {
		ret = -ENODEV;
		goto out;
	}

	type = aoc->services[service_idx].type;
	reg = &hdr->regions[LGA_AOC_DIR_UP];
	offset = le32_to_cpu(reg->offset);
	size = le32_to_cpu(reg->size);
	slots = le32_to_cpu(reg->slots);

	if (!lga_aoc_region_bounds_ok(aoc, offset, size, slots)) {
		ret = -EIO;
		goto out;
	}

	tx = le32_to_cpu(READ_ONCE(reg->tx));
	rx = le32_to_cpu(READ_ONCE(reg->rx));
	rp = le32_to_cpu(READ_ONCE(reg->rp));
	dma_rmb();

	if (tx == rx) {
		ret = -EAGAIN;
		goto out;
	}

	if (type == LGA_AOC_SVC_QUEUE) {
		if (size <= sizeof(mhdr)) {
			ret = -EIO;
			goto out;
		}
		rp %= slots;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset + (size_t)rp * size;
		memcpy(&mhdr, slot_ptr, sizeof(mhdr));
		msg_len = le16_to_cpu(mhdr.length);
		if (msg_len > size - sizeof(mhdr) || msg_len > max_len) {
			ret = -EMSGSIZE;
			goto out;
		}
		memcpy(buf, slot_ptr + sizeof(mhdr), msg_len);
		dma_wmb();
		WRITE_ONCE(reg->rp, cpu_to_le32((rp + 1U) % slots));
		WRITE_ONCE(reg->rx, cpu_to_le32(rx + 1U));
		ret = msg_len;
	} else if (type == LGA_AOC_SVC_RING) {
		size_t avail = min_t(size_t, tx - rx, size);
		size_t to_read = min(avail, max_len);
		size_t first;

		rp %= size;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset;
		first = min_t(size_t, to_read, size - rp);
		memcpy(buf, slot_ptr + rp, first);
		if (to_read > first)
			memcpy(buf + first, slot_ptr, to_read - first);
		dma_wmb();
		WRITE_ONCE(reg->rp, cpu_to_le32((rp + to_read) % size));
		WRITE_ONCE(reg->rx, cpu_to_le32(rx + to_read));
		ret = to_read;
	} else {
		ret = -EOPNOTSUPP;
	}
out:
	mutex_unlock(&aoc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_read);

/* debugfs bring-up interface */

static int lga_aoc_info_show(struct seq_file *s, void *unused)
{
	struct lga_aoc *aoc = s->private;
	int gsa_state;

	gsa_state = gsa_send_aoc_cmd(aoc->gsa, GSA_AOC_GET_STATE);

	seq_printf(s, "board_cfg: %s\n", aoc->board_cfg ?: "unknown");
	seq_printf(s, "board_id: %#x\n", aoc->board_id);
	seq_printf(s, "board_rev: %#x\n", aoc->board_rev);
	seq_printf(s, "gnss_type: %u\n", aoc->gnss_type);
	seq_printf(s, "firmware_name: %s\n", aoc->fw_name);
	seq_printf(s, "carveout: %pa + %#zx\n", &aoc->dram_base, aoc->dram_size);
	seq_printf(s, "mboxes: %u (coredump_idx=%u)\n",
		   aoc->num_mboxes, aoc->coredump_mbox);
	seq_printf(s, "gsa_aoc_state: %d (%s)\n",
		   gsa_state, lga_aoc_gsa_state_str(gsa_state));
	seq_printf(s, "supplies_enabled: %u\n", aoc->supplies_enabled);
	seq_printf(s, "loaded: %u online: %u services: %u\n",
		   aoc->loaded, aoc->online, aoc->num_services);
	if (aoc->loaded)
		seq_printf(s, "fw_version: %s (hdr %#zx ipc +%#x)\n",
			   aoc->fw_version, aoc->hdr_size, aoc->ipc_offset);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lga_aoc_info);

static ssize_t lga_aoc_load_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct lga_aoc *aoc = file->private_data;
	char name[96];
	const char *fw;
	int ret;

	if (!count || count >= sizeof(name))
		return -EINVAL;
	if (copy_from_user(name, ubuf, count))
		return -EFAULT;
	name[count] = '\0';
	strim(name);

	fw = (!strcmp(name, "1") || !*name) ? aoc->fw_name : name;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_load_locked(aoc, fw);
	mutex_unlock(&aoc->lock);
	return ret ?: count;
}

static const struct file_operations lga_aoc_load_fops = {
	.open = simple_open,
	.write = lga_aoc_load_write,
};

static ssize_t lga_aoc_start_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct lga_aoc *aoc = file->private_data;
	bool en;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &en);
	if (ret)
		return ret;
	if (!en)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_start_locked(aoc);
	mutex_unlock(&aoc->lock);
	return ret ?: count;
}

static const struct file_operations lga_aoc_start_fops = {
	.open = simple_open,
	.write = lga_aoc_start_write,
};

static ssize_t lga_aoc_stop_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct lga_aoc *aoc = file->private_data;
	bool en;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &en);
	if (ret)
		return ret;
	if (!en)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_stop_locked(aoc);
	mutex_unlock(&aoc->lock);
	return ret ?: count;
}

static const struct file_operations lga_aoc_stop_fops = {
	.open = simple_open,
	.write = lga_aoc_stop_write,
};

static int lga_aoc_services_show(struct seq_file *s, void *unused)
{
	struct lga_aoc *aoc = s->private;
	unsigned int i;

	mutex_lock(&aoc->lock);
	if (!aoc->online) {
		seq_puts(s, "offline\n");
		mutex_unlock(&aoc->lock);
		return 0;
	}

	seq_printf(s, "services: %u (table +%#x, entry %u bytes)\n",
		   aoc->num_services, aoc->services_offset, aoc->service_size);
	for (i = 0; i < aoc->num_services; i++) {
		const struct lga_aoc_service_info *si = &aoc->services[i];

		seq_printf(s, "  %3u %-32s %-6s mbox=%2u down=%ux%u@+%#x up=%ux%u@+%#x\n",
			   si->index, si->name, lga_aoc_svc_type_str(si->type),
			   si->mbox_index, si->down_slots, si->down_size,
			   si->down_offset, si->up_slots, si->up_size,
			   si->up_offset);
	}
	mutex_unlock(&aoc->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lga_aoc_services);

static int lga_aoc_ipc_ring_show(struct seq_file *s, void *unused)
{
	struct lga_aoc *aoc = s->private;
	unsigned int i;

	mutex_lock(&aoc->lock);
	if (!aoc->online) {
		seq_puts(s, "offline\n");
		mutex_unlock(&aoc->lock);
		return 0;
	}

	for (i = 0; i < aoc->num_services; i++) {
		const struct lga_aoc_ipc_service_hdr *hdr;
		const struct lga_aoc_ipc_region *dn, *up;

		hdr = lga_aoc_service_hdr(aoc, i);
		if (!hdr)
			continue;
		dn = &hdr->regions[LGA_AOC_DIR_DOWN];
		up = &hdr->regions[LGA_AOC_DIR_UP];
		seq_printf(s, "%3u %-28s dn(tx=%u rx=%u wp=%u rp=%u) up(tx=%u rx=%u wp=%u rp=%u)\n",
			   i, aoc->services[i].name,
			   le32_to_cpu(READ_ONCE(dn->tx)),
			   le32_to_cpu(READ_ONCE(dn->rx)),
			   le32_to_cpu(READ_ONCE(dn->wp)),
			   le32_to_cpu(READ_ONCE(dn->rp)),
			   le32_to_cpu(READ_ONCE(up->tx)),
			   le32_to_cpu(READ_ONCE(up->rx)),
			   le32_to_cpu(READ_ONCE(up->wp)),
			   le32_to_cpu(READ_ONCE(up->rp)));
	}
	mutex_unlock(&aoc->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lga_aoc_ipc_ring);

static int lga_aoc_mbox_show(struct seq_file *s, void *unused)
{
	struct lga_aoc *aoc = s->private;
	unsigned int i;

	for (i = 0; i < aoc->num_mboxes; i++) {
		struct lga_aoc_mbox_slot *slot = &aoc->mboxes[i];

		seq_printf(s, "ch%02u: tx=%d rx=%d last_rx=%#x%s\n",
			   slot->index, atomic_read(&slot->tx_count),
			   atomic_read(&slot->rx_count), slot->last_rx_word,
			   slot->index == aoc->coredump_mbox ? " (coredump)" : "");
	}
	return 0;
}

static int lga_aoc_mbox_open(struct inode *inode, struct file *file)
{
	return single_open(file, lga_aoc_mbox_show, inode->i_private);
}

static ssize_t lga_aoc_mbox_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct lga_aoc *aoc = seq->private;
	unsigned int idx;
	int ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &idx);
	if (ret)
		return ret;
	if (idx >= aoc->num_mboxes)
		return -EINVAL;
	if (idx == aoc->coredump_mbox && !allow_coredump_ping)
		return -EPERM;

	ret = lga_aoc_ring_doorbell(aoc->dev, idx);
	return ret ?: count;
}

static const struct file_operations lga_aoc_mbox_fops = {
	.open = lga_aoc_mbox_open,
	.read = seq_read,
	.write = lga_aoc_mbox_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void lga_aoc_detach_pds(void *data)
{
	struct lga_aoc *aoc = data;
	int i;

	for (i = ARRAY_SIZE(aoc->pd_devs) - 1; i >= 0; i--) {
		if (aoc->pd_links[i])
			device_link_del(aoc->pd_links[i]);
		if (!IS_ERR_OR_NULL(aoc->pd_devs[i]))
			dev_pm_domain_detach(aoc->pd_devs[i], true);
	}
}

static int lga_aoc_attach_pds(struct lga_aoc *aoc)
{
	static const char * const pd_names[] = { "sswrp_aoc_pd", "aoc_core_pd" };
	int i;

	if (of_count_phandle_with_args(aoc->dev->of_node, "power-domains",
				       "#power-domain-cells") < 2)
		return 0;

	for (i = 0; i < ARRAY_SIZE(pd_names); i++) {
		struct device *pd_dev;
		struct device_link *link;

		pd_dev = dev_pm_domain_attach_by_name(aoc->dev, pd_names[i]);
		if (IS_ERR(pd_dev)) {
			lga_aoc_detach_pds(aoc);
			return PTR_ERR(pd_dev);
		}
		aoc->pd_devs[i] = pd_dev;
		if (pd_dev) {
			link = device_link_add(aoc->dev, pd_dev,
					       DL_FLAG_STATELESS |
					       DL_FLAG_PM_RUNTIME);
			if (!link) {
				lga_aoc_detach_pds(aoc);
				return -ENOMEM;
			}
			aoc->pd_links[i] = link;
		}
	}

	return devm_add_action_or_reset(aoc->dev, lga_aoc_detach_pds, aoc);
}

static void lga_aoc_free_mboxes(void *data)
{
	struct lga_aoc *aoc = data;
	unsigned int i;

	for (i = 0; i < aoc->num_mboxes; i++) {
		if (!IS_ERR_OR_NULL(aoc->mboxes[i].chan)) {
			mbox_free_channel(aoc->mboxes[i].chan);
			aoc->mboxes[i].chan = NULL;
		}
	}
}

static void lga_aoc_put_gsa(void *data)
{
	gsa_put_device(data);
}

static int lga_aoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource rmem;
	struct lga_aoc *aoc;
	int n_mboxes, ret;
	unsigned int i;

	aoc = devm_kzalloc(dev, sizeof(*aoc), GFP_KERNEL);
	if (!aoc)
		return -ENOMEM;

	aoc->dev = dev;
	mutex_init(&aoc->lock);
	init_waitqueue_head(&aoc->ipc_wq);

	ret = of_reserved_mem_region_to_resource(np, 0, &rmem);
	if (ret)
		return dev_err_probe(dev, ret, "missing AoC memory-region\n");

	aoc->dram_base = rmem.start;
	aoc->dram_size = resource_size(&rmem);
	aoc->dram = devm_memremap(dev, aoc->dram_base, aoc->dram_size,
				  MEMREMAP_WC);
	if (IS_ERR(aoc->dram))
		return dev_err_probe(dev, PTR_ERR(aoc->dram),
				     "failed to map AoC carve-out\n");

	aoc->gsa = gsa_get_device(dev);
	if (IS_ERR(aoc->gsa))
		return dev_err_probe(dev, PTR_ERR(aoc->gsa),
				     "failed to acquire GSA device\n");
	ret = devm_add_action_or_reset(dev, lga_aoc_put_gsa, aoc->gsa);
	if (ret)
		return ret;

	ret = lga_aoc_attach_pds(aoc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach AoC power domains\n");

	for (i = 0; i < LGA_AOC_NUM_SUPPLIES; i++) {
		struct regulator *reg;

		reg = devm_regulator_get_optional(dev, lga_aoc_supply_names[i]);
		if (IS_ERR(reg)) {
			if (PTR_ERR(reg) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(reg),
					     "failed to get %s supply\n",
					     lga_aoc_supply_names[i]);
		}
		aoc->supplies[i] = reg;
	}

	n_mboxes = of_count_phandle_with_args(np, "mboxes", "#mbox-cells");
	if (n_mboxes <= 0)
		return dev_err_probe(dev, -EINVAL, "no AoC mailboxes defined\n");

	aoc->num_mboxes = min_t(unsigned int, n_mboxes, LGA_AOC_MAX_MBOXES);
	ret = devm_add_action_or_reset(dev, lga_aoc_free_mboxes, aoc);
	if (ret)
		return ret;

	for (i = 0; i < aoc->num_mboxes; i++) {
		struct lga_aoc_mbox_slot *slot = &aoc->mboxes[i];

		slot->aoc = aoc;
		slot->index = i;
		slot->client.dev = dev;
		slot->client.tx_block = false;
		slot->client.tx_tout = 100;
		slot->client.rx_callback = lga_aoc_mbox_rx_cb;

		slot->chan = mbox_request_channel(&slot->client, i);
		if (IS_ERR(slot->chan)) {
			ret = PTR_ERR(slot->chan);
			slot->chan = NULL;
			return dev_err_probe(dev, ret,
					     "failed to request AoC mbox %u\n", i);
		}
	}

	aoc->coredump_mbox = LGA_AOC_DEFAULT_COREDUMP_MBOX;
	of_property_read_u32(np, "google,coredump-mbox", &aoc->coredump_mbox);
	of_property_read_u32(np, "google,board-id", &aoc->board_id);
	of_property_read_u32(np, "google,board-rev", &aoc->board_rev);
	of_property_read_u32(np, "google,gnss-type", &aoc->gnss_type);
	of_property_read_string(np, "google,board-cfg", &aoc->board_cfg);
	if (of_property_read_string(np, "firmware-name", &aoc->fw_name))
		aoc->fw_name = LGA_AOC_DEFAULT_FW;

	platform_set_drvdata(pdev, aoc);
	pm_runtime_enable(dev);

	aoc->debugfs = debugfs_create_dir("lga-aoc", NULL);
	debugfs_create_file("info", 0400, aoc->debugfs, aoc, &lga_aoc_info_fops);
	debugfs_create_file("load", 0200, aoc->debugfs, aoc, &lga_aoc_load_fops);
	debugfs_create_file("start", 0200, aoc->debugfs, aoc, &lga_aoc_start_fops);
	debugfs_create_file("stop", 0200, aoc->debugfs, aoc, &lga_aoc_stop_fops);
	debugfs_create_file("services", 0400, aoc->debugfs, aoc,
			    &lga_aoc_services_fops);
	debugfs_create_file("ipc_ring", 0400, aoc->debugfs, aoc,
			    &lga_aoc_ipc_ring_fops);
	debugfs_create_file("mbox", 0600, aoc->debugfs, aoc, &lga_aoc_mbox_fops);

	dev_info(dev,
		 "Laguna AoC ready (cfg=%s board=%#x/%#x carveout=%pa+%#zx mboxes=%u)\n",
		 aoc->board_cfg ?: "?", aoc->board_id, aoc->board_rev,
		 &aoc->dram_base, aoc->dram_size, aoc->num_mboxes);

	if (autoload) {
		mutex_lock(&aoc->lock);
		ret = lga_aoc_load_locked(aoc, aoc->fw_name);
		if (!ret)
			ret = lga_aoc_start_locked(aoc);
		mutex_unlock(&aoc->lock);
		if (ret)
			dev_warn(dev, "autoload failed: %d\n", ret);
	}

	return 0;
}

static void lga_aoc_remove(struct platform_device *pdev)
{
	struct lga_aoc *aoc = platform_get_drvdata(pdev);

	debugfs_remove_recursive(aoc->debugfs);
	mutex_lock(&aoc->lock);
	if (aoc->loaded || aoc->online)
		lga_aoc_stop_locked(aoc);
	else
		lga_aoc_set_supplies(aoc, false);
	mutex_unlock(&aoc->lock);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id lga_aoc_of_match[] = {
	{ .compatible = "google,lga-aoc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lga_aoc_of_match);

static struct platform_driver lga_aoc_driver = {
	.probe = lga_aoc_probe,
	.remove = lga_aoc_remove,
	.driver = {
		.name = "google-lga-aoc",
		.of_match_table = lga_aoc_of_match,
	},
};
module_platform_driver(lga_aoc_driver);

MODULE_DESCRIPTION("Google Tensor G5 (Laguna) Stage 1 AoC boot and IPC driver");
MODULE_LICENSE("GPL");
