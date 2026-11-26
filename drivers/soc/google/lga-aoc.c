// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room Stage 1 Always-on Compute (AoC) boot and IPC driver for
 * Google Tensor G5 (Laguna).
 *
 * Implements step-by-step AoC firmware validation, GSA authentication over
 * google,mba-ctrl v2.0, Trusty hwmgr.aoc lifecycle control, optional SMMUv3
 * window mapping from the signed image header, 20-channel google,mba-ctrl
 * v2.0 doorbell messaging, and shared-carveout IPC service discovery and
 * queue/ring transport for downstream consumers (e.g. hci_lga_aoc).
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
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
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

#define LGA_AOC_HDR_FMT_PQ		2U
#define LGA_AOC_HDR_STD_SIZE		GSA_AOC_HEADER_SIZE
#define LGA_AOC_HDR_PQ_SIZE		GSA_AOC_PQ_HEADER_SIZE

/* Offsets within signed firmware containers */
#define LGA_AOC_LEGACY_V1_CFG_OFF	0x560U
#define LGA_AOC_LEGACY_V2_CFG_OFF	0x580U
#define LGA_AOC_NEW_CFG_OFF		0x9f0U
#define LGA_AOC_PQ_SUBHDR_OFF		0x4630U
#define LGA_AOC_IMG_CFG_SIZE		256U

/* LCPM scratch register cleared during Laguna probe */
#define LGA_AOC_LCPM_INIT_OFF		0x480U

/* Four heaps reserved at the top of the AoC carve-out (total 0x434000) */
#define LGA_AOC_AUX_HEAP_SIZE		SZ_128K
#define LGA_AOC_SENSOR_HEAP_SIZE	SZ_4M
#define LGA_AOC_PLAYBACK_HEAP_SIZE	SZ_16K
#define LGA_AOC_CAPTURE_HEAP_SIZE	SZ_64K
#define LGA_AOC_TOTAL_HEAPS_SIZE	(LGA_AOC_AUX_HEAP_SIZE + \
					 LGA_AOC_SENSOR_HEAP_SIZE + \
					 LGA_AOC_PLAYBACK_HEAP_SIZE + \
					 LGA_AOC_CAPTURE_HEAP_SIZE)

#define LGA_AOC_MAX_SERVICES		256U
#define LGA_AOC_MAX_IOMMU_MAPS		33U
#define LGA_AOC_START_TIMEOUT_MS	10000U
#define LGA_AOC_START_POLL_MS		50U
#define LGA_AOC_DEFAULT_FW		"aoc.bin"
#define LGA_AOC_DEFAULT_COREDUMP_MBOX	19U
#define LGA_AOC_DEFAULT_BOARD_ID	0x20202U
#define LGA_AOC_DEFAULT_BOARD_REV	0x10000U
#define LGA_AOC_DEFAULT_PRODUCT_ID	5U		/* Laguna B0 */
#define LGA_AOC_DEFAULT_CHIP_REV	0x10U		/* major=1, minor=0 */

/* Service header flag bitfields */
#define LGA_AOC_SVC_TYPE_MASK		GENMASK(18, 16)
#define LGA_AOC_SVC_IRQ_MASK		GENMASK(23, 19)

/* Boot parameter TLV keys expected by Laguna AoC firmware (24 entries) */
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
	LGA_AOC_PARAM_FORCE_SPKR_US		= 0x1010,
	LGA_AOC_PARAM_RAND_SEED			= 0x1011,
	LGA_AOC_PARAM_CHIP_REV			= 0x1012,
	LGA_AOC_PARAM_CHIP_TYPE			= 0x1013,
	LGA_AOC_PARAM_GNSS_TYPE			= 0x1014,
	LGA_AOC_PARAM_VOLTE_RELEASE_MIF		= 0x1015,
	LGA_AOC_PARAM_CHIP_PRODUCT_ID		= 0x1016,
	LGA_AOC_PARAM_WIFI_CHIP			= 0x1017,
	LGA_AOC_PARAM_AUX_HEAP_ADDR		= 0x101b,
	LGA_AOC_PARAM_AUX_HEAP_SIZE		= 0x101c,
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

struct lga_aoc_image_config {
	__le32 version;
	__le32 fw_start;
	__le32 fw_size;
	__le32 privilege_level;
	__le16 bl_offset;
	__le16 bl_size;
	__le16 iommu_offset;
	__le16 iommu_size;
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

struct lga_aoc_iommu_map {
	unsigned long iova;
	size_t size;
};

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
	bool sswrp_pd_on;
	bool core_pd_on;

	void __iomem *lcpm;
	resource_size_t lcpm_size;

	phys_addr_t dram_base;
	size_t dram_size;
	void *dram;

	void *auth_hdr_virt;
	dma_addr_t auth_hdr_dma;
	size_t auth_hdr_alloc_size;

	struct lga_aoc_iommu_map iommu_maps[LGA_AOC_MAX_IOMMU_MAPS];
	unsigned int num_iommu_maps;

	struct regulator *supplies[LGA_AOC_NUM_SUPPLIES];
	bool supplies_enabled;

	struct lga_aoc_mbox_slot mboxes[LGA_AOC_MAX_MBOXES];
	unsigned int num_mboxes;
	u32 coredump_mbox;

	u32 board_id;
	u32 board_rev;
	u32 gnss_type;
	u32 wifi_chip;
	u32 force_vnom;
	u32 chip_product_id;
	u32 chip_rev;
	const char *board_cfg;
	const char *fw_name;

	u32 ipc_offset;
	size_t hdr_size;
	size_t img_cfg_off;
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

static DEFINE_MUTEX(lga_aoc_dev_lock);
static struct lga_aoc *lga_aoc_active_dev;
static BLOCKING_NOTIFIER_HEAD(lga_aoc_notifiers);

static bool autoload;
module_param(autoload, bool, 0444);
MODULE_PARM_DESC(autoload, "Load and start AoC firmware automatically at probe");

static bool allow_coredump_ping;
module_param(allow_coredump_ping, bool, 0644);
MODULE_PARM_DESC(allow_coredump_ping,
		 "Allow sending debugfs doorbell pings on the coredump mailbox");

static bool configure_iommu = true;
module_param(configure_iommu, bool, 0644);
MODULE_PARM_DESC(configure_iommu,
		 "Map AoC firmware IOMMU windows into a non-identity SMMU domain");

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

struct device *lga_aoc_get_device(void)
{
	struct device *dev = NULL;

	mutex_lock(&lga_aoc_dev_lock);
	if (lga_aoc_active_dev)
		dev = get_device(lga_aoc_active_dev->dev);
	mutex_unlock(&lga_aoc_dev_lock);
	return dev;
}
EXPORT_SYMBOL_GPL(lga_aoc_get_device);

bool lga_aoc_is_online(struct device *dev)
{
	struct lga_aoc *aoc;
	bool online;

	if (!dev)
		return false;

	aoc = dev_get_drvdata(dev);
	if (!aoc)
		return false;

	mutex_lock(&aoc->lock);
	online = aoc->online;
	mutex_unlock(&aoc->lock);
	return online;
}
EXPORT_SYMBOL_GPL(lga_aoc_is_online);

int lga_aoc_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lga_aoc_notifiers, nb);
}
EXPORT_SYMBOL_GPL(lga_aoc_register_notifier);

int lga_aoc_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&lga_aoc_notifiers, nb);
}
EXPORT_SYMBOL_GPL(lga_aoc_unregister_notifier);

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

static int lga_aoc_core_pd_set(struct lga_aoc *aoc, bool enable)
{
	int ret;

	if (!aoc->pd_devs[1])
		return 0;

	if (enable) {
		if (aoc->core_pd_on)
			return 0;
		ret = pm_runtime_resume_and_get(aoc->pd_devs[1]);
		if (ret) {
			dev_err(aoc->dev, "failed to power on aoc_core_pd: %d\n",
				ret);
			return ret;
		}
		aoc->core_pd_on = true;
		return 0;
	}

	if (!aoc->core_pd_on)
		return 0;

	ret = pm_runtime_put_sync(aoc->pd_devs[1]);
	if (ret < 0)
		dev_warn(aoc->dev, "failed to release aoc_core_pd: %d\n", ret);
	aoc->core_pd_on = false;
	return 0;
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

static void lga_aoc_unmap_iommu(struct lga_aoc *aoc)
{
	struct iommu_domain *domain;
	unsigned int i;

	if (!aoc->num_iommu_maps)
		return;

	domain = iommu_get_domain_for_dev(aoc->dev);
	if (domain && domain->type != IOMMU_DOMAIN_IDENTITY) {
		for (i = 0; i < aoc->num_iommu_maps; i++)
			iommu_unmap(domain, aoc->iommu_maps[i].iova,
				    aoc->iommu_maps[i].size);
	}
	aoc->num_iommu_maps = 0;
}

static int lga_aoc_map_one_window(struct lga_aoc *aoc,
				  struct iommu_domain *domain,
				  unsigned long iova, phys_addr_t paddr,
				  size_t size)
{
	int ret;

	if (!size || !PAGE_ALIGNED(iova) || !PAGE_ALIGNED(paddr) ||
	    !PAGE_ALIGNED(size))
		return -EINVAL;

	if (iommu_iova_to_phys(domain, iova))
		return 0;

	if (aoc->num_iommu_maps >= LGA_AOC_MAX_IOMMU_MAPS)
		return -ENOSPC;

	ret = iommu_map(domain, iova, paddr, size, IOMMU_READ | IOMMU_WRITE,
			GFP_KERNEL);
	if (ret) {
		dev_err(aoc->dev,
			"iommu_map(iova=%#lx pa=%pa size=%#zx) failed: %d\n",
			iova, &paddr, size, ret);
		return ret;
	}

	aoc->iommu_maps[aoc->num_iommu_maps].iova = iova;
	aoc->iommu_maps[aoc->num_iommu_maps].size = size;
	aoc->num_iommu_maps++;
	return 0;
}

static int lga_aoc_configure_iommu(struct lga_aoc *aoc,
				   const struct firmware *fw)
{
	struct iommu_domain *domain;
	const struct lga_aoc_image_config *cfg;
	const __le64 *entries;
	u16 tbl_off, tbl_size;
	unsigned int i, count;
	bool dram_mapped = false;
	int ret;

	if (!configure_iommu)
		return 0;

	domain = iommu_get_domain_for_dev(aoc->dev);
	if (!domain || domain->type == IOMMU_DOMAIN_IDENTITY)
		return 0;

	lga_aoc_unmap_iommu(aoc);

	if (aoc->img_cfg_off &&
	    aoc->img_cfg_off + LGA_AOC_IMG_CFG_SIZE <= aoc->hdr_size) {
		cfg = (const struct lga_aoc_image_config *)
			(fw->data + aoc->img_cfg_off);
		tbl_off = le16_to_cpu(cfg->iommu_offset);
		tbl_size = le16_to_cpu(cfg->iommu_size);

		if (tbl_size >= sizeof(u64) &&
		    tbl_off < LGA_AOC_IMG_CFG_SIZE &&
		    (u32)tbl_off + tbl_size <= LGA_AOC_IMG_CFG_SIZE &&
		    (tbl_size % sizeof(u64)) == 0) {
			entries = (const __le64 *)((const u8 *)cfg + tbl_off);
			count = tbl_size / sizeof(u64);

			for (i = 0; i < count; i++) {
				u64 raw = le64_to_cpu(entries[i]);
				unsigned long iova = (u32)(raw << 12);
				phys_addr_t paddr =
					(phys_addr_t)((raw >> 8) &
						      0xffffff000ULL);
				size_t sz = (size_t)((raw >> 32) &
						     0xfffff000ULL);

				if (!sz)
					continue;
				if (iova <= LGA_AOC_DRAM_IOVA_BASE &&
				    iova + sz >= LGA_AOC_DRAM_IOVA_BASE +
						 aoc->dram_size)
					dram_mapped = true;

				ret = lga_aoc_map_one_window(aoc, domain, iova,
							     paddr, sz);
				if (ret)
					goto err_unmap;
			}
		}
	}

	if (!dram_mapped) {
		ret = lga_aoc_map_one_window(aoc, domain,
					     LGA_AOC_DRAM_IOVA_BASE,
					     aoc->dram_base, aoc->dram_size);
		if (ret)
			goto err_unmap;
	}

	dev_info(aoc->dev, "configured %u SMMU window(s) for AoC\n",
		 aoc->num_iommu_maps);
	return 0;

err_unmap:
	lga_aoc_unmap_iommu(aoc);
	return ret;
}

static int lga_aoc_detect_header(const struct firmware *fw, size_t *hdr_size,
				 size_t *img_cfg_off)
{
	const struct lga_aoc_legacy_hdr_v1 *h1;
	const struct lga_aoc_legacy_hdr_v2 *h2;
	u32 fmt, magic, sb_magic;

	if (!fw || !fw->data || fw->size < LGA_AOC_HDR_STD_SIZE)
		return -EINVAL;

	/* Legacy v1/v2 signed 4 KiB header ("AOC" magic at +0x400) */
	h1 = (const struct lga_aoc_legacy_hdr_v1 *)fw->data;
	if (le32_to_cpu(h1->magic) == LGA_AOC_AUTH_MAGIC &&
	    le32_to_cpu(h1->generation) == 1) {
		*hdr_size = LGA_AOC_HDR_STD_SIZE;
		*img_cfg_off = LGA_AOC_LEGACY_V1_CFG_OFF;
		return 0;
	}

	h2 = (const struct lga_aoc_legacy_hdr_v2 *)fw->data;
	if (le32_to_cpu(h2->magic) == LGA_AOC_AUTH_MAGIC &&
	    le32_to_cpu(h2->generation) == 2) {
		*hdr_size = LGA_AOC_HDR_STD_SIZE;
		*img_cfg_off = LGA_AOC_LEGACY_V2_CFG_OFF;
		return 0;
	}

	fmt = le32_to_cpup((const __le32 *)fw->data);

	/* Laguna signed 4 KiB header (non-zero header version, superbin at +0x1000) */
	if (fmt != 0 &&
	    fw->size >= LGA_AOC_HDR_STD_SIZE + sizeof(struct lga_aoc_superbin_hdr)) {
		sb_magic = le32_to_cpup((const __le32 *)(fw->data +
							 LGA_AOC_HDR_STD_SIZE));
		if (sb_magic == LGA_AOC_SUPERBIN_MAGIC) {
			*hdr_size = LGA_AOC_HDR_STD_SIZE;
			*img_cfg_off = LGA_AOC_NEW_CFG_OFF;
			return 0;
		}
	}

	/* Post-quantum 20 KiB signed header ("AOC" at +0x4630, superbin at +0x5000) */
	if (fmt == LGA_AOC_HDR_FMT_PQ &&
	    fw->size >= LGA_AOC_HDR_PQ_SIZE + sizeof(struct lga_aoc_superbin_hdr)) {
		magic = le32_to_cpup((const __le32 *)(fw->data +
						      LGA_AOC_PQ_SUBHDR_OFF));
		if (magic == LGA_AOC_AUTH_MAGIC) {
			*hdr_size = LGA_AOC_HDR_PQ_SIZE;
			*img_cfg_off = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static int lga_aoc_parse_fw(struct lga_aoc *aoc, const struct firmware *fw)
{
	const struct lga_aoc_superbin_hdr *sb;
	size_t hdr_size = 0, img_cfg_off = 0, body_size;
	u32 ipc_off, bl_off, bl_size, uuid_off, uuid_size;
	int ret;

	ret = lga_aoc_detect_header(fw, &hdr_size, &img_cfg_off);
	if (ret) {
		dev_err(aoc->dev, "unrecognized AoC signed header\n");
		return ret;
	}

	if (fw->size <= hdr_size + sizeof(*sb))
		return -EINVAL;

	body_size = fw->size - hdr_size;
	if (body_size >= aoc->dram_size ||
	    body_size + LGA_AOC_TOTAL_HEAPS_SIZE > aoc->dram_size) {
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

	uuid_off = le32_to_cpu(sb->uuid_table_offset);
	uuid_size = le32_to_cpu(sb->uuid_table_size);
	if ((u64)uuid_off + uuid_size >= body_size) {
		dev_err(aoc->dev, "invalid AoC UUID table (%#x+%#x >= %#zx)\n",
			uuid_off, uuid_size, body_size);
		return -EINVAL;
	}

	bl_off = le32_to_cpu(sb->bootloader_offset);
	bl_size = le32_to_cpu(sb->bootloader_size);
	if ((u64)bl_off + bl_size >= body_size) {
		dev_err(aoc->dev, "invalid AoC bootloader (%#x+%#x >= %#zx)\n",
			bl_off, bl_size, body_size);
		return -EINVAL;
	}

	ipc_off = le32_to_cpu(sb->image_size);
	if (ipc_off < body_size ||
	    ipc_off + SZ_64K > aoc->dram_size - LGA_AOC_TOTAL_HEAPS_SIZE) {
		dev_err(aoc->dev, "invalid AoC IPC offset %#x (body %#zx)\n",
			ipc_off, body_size);
		return -EINVAL;
	}

	aoc->hdr_size = hdr_size;
	aoc->img_cfg_off = img_cfg_off;
	aoc->ipc_offset = ipc_off;
	snprintf(aoc->fw_version, sizeof(aoc->fw_version), "%.8s%.64s",
		 sb->version_prefix, sb->version_string);

	dev_info(aoc->dev,
		 "parsed AoC image: hdr %#zx cfg +%#zx body %#zx ipc +%#x version '%s'\n",
		 hdr_size, img_cfg_off, body_size, ipc_off, aoc->fw_version);
	return 0;
}

static u32 lga_aoc_dram_to_iova(struct lga_aoc *aoc, phys_addr_t pa)
{
	return LGA_AOC_DRAM_IOVA_BASE + (u32)(pa - aoc->dram_base);
}

static void lga_aoc_clear_heaps(struct lga_aoc *aoc)
{
	if (aoc->dram_size <= LGA_AOC_TOTAL_HEAPS_SIZE)
		return;

	memset(aoc->dram + aoc->dram_size - LGA_AOC_TOTAL_HEAPS_SIZE, 0,
	       LGA_AOC_TOTAL_HEAPS_SIZE);
	dma_wmb();
}

static void lga_aoc_write_boot_params(struct lga_aoc *aoc)
{
	phys_addr_t top = aoc->dram_base + aoc->dram_size;
	phys_addr_t aux_pa = top - LGA_AOC_AUX_HEAP_SIZE;
	phys_addr_t sensor_pa = aux_pa - LGA_AOC_SENSOR_HEAP_SIZE;
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
		{ LGA_AOC_PARAM_AUX_HEAP_ADDR,
		  lga_aoc_dram_to_iova(aoc, aux_pa) },
		{ LGA_AOC_PARAM_AUX_HEAP_SIZE, LGA_AOC_AUX_HEAP_SIZE },
		{ LGA_AOC_PARAM_FORCE_VNOM, aoc->force_vnom },
		{ LGA_AOC_PARAM_DISABLE_MM, 0 },
		{ LGA_AOC_PARAM_ENABLE_UART, 0 },
		{ LGA_AOC_PARAM_FORCE_SPKR_US, 0 },
		{ LGA_AOC_PARAM_RAND_SEED, get_random_u32() },
		{ LGA_AOC_PARAM_CHIP_REV, aoc->chip_rev },
		{ LGA_AOC_PARAM_CHIP_TYPE, 0 },
		{ LGA_AOC_PARAM_GNSS_TYPE, aoc->gnss_type },
		{ LGA_AOC_PARAM_VOLTE_RELEASE_MIF, 0 },
		{ LGA_AOC_PARAM_CHIP_PRODUCT_ID, aoc->chip_product_id },
		{ LGA_AOC_PARAM_WIFI_CHIP, aoc->wifi_chip },
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
		list[i].flags = flags;
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

static int lga_aoc_load_locked(struct lga_aoc *aoc, const char *name,
			       bool allow_fallback)
{
	const struct firmware *fw;
	const char *loaded_name = name;
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
	if (ret == -ENOENT && allow_fallback &&
	    strcmp(name, LGA_AOC_DEFAULT_FW) != 0) {
		dev_info(aoc->dev,
			 "firmware '%s' not found, falling back to '%s'\n",
			 name, LGA_AOC_DEFAULT_FW);
		loaded_name = LGA_AOC_DEFAULT_FW;
		ret = request_firmware(&fw, loaded_name, aoc->dev);
	}
	if (ret) {
		dev_err(aoc->dev, "request_firmware(%s) failed: %d\n",
			loaded_name, ret);
		return ret;
	}

	ret = lga_aoc_parse_fw(aoc, fw);
	if (ret)
		goto out_rel;

	ret = lga_aoc_core_pd_set(aoc, true);
	if (ret)
		goto out_rel;

	ret = lga_aoc_configure_iommu(aoc, fw);
	if (ret)
		goto out_pd;

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
			goto out_iommu;
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
		goto out_iommu;
	}

	lga_aoc_write_boot_params(aoc);
	aoc->loaded = true;
	dev_info(aoc->dev, "AoC firmware '%s' authenticated and loaded\n",
		 loaded_name);
	release_firmware(fw);
	return 0;

out_iommu:
	lga_aoc_unmap_iommu(aoc);
out_pd:
	lga_aoc_core_pd_set(aoc, false);
out_rel:
	release_firmware(fw);
	return ret;
}

static u32 lga_aoc_read_cb_magic(const struct lga_aoc_control_block *cb)
{
	dma_rmb();
	return le32_to_cpu(READ_ONCE(cb->magic));
}

static int lga_aoc_start_locked(struct lga_aoc *aoc, bool *became_online)
{
	const struct lga_aoc_control_block *cb;
	unsigned long deadline;
	u32 magic = 0;
	int ret;

	*became_online = false;

	if (!aoc->loaded)
		return -EINVAL;
	if (aoc->online)
		return 0;

	ret = lga_aoc_core_pd_set(aoc, true);
	if (ret)
		return ret;

	ret = lga_aoc_set_supplies(aoc, true);
	if (ret)
		return ret;

	ret = gsa_aoc_start(aoc->gsa);
	if (ret) {
		lga_aoc_set_supplies(aoc, false);
		return ret;
	}

	cb = (const struct lga_aoc_control_block *)(aoc->dram + aoc->ipc_offset);
	deadline = jiffies + msecs_to_jiffies(LGA_AOC_START_TIMEOUT_MS);

	for (;;) {
		magic = lga_aoc_read_cb_magic(cb);
		if (magic == LGA_AOC_CONTROL_MAGIC)
			break;
		if (time_after(jiffies, deadline)) {
			dev_err(aoc->dev,
				"timed out waiting for AoC control block (magic %#x)\n",
				magic);
			return -ETIMEDOUT;
		}
		wait_event_timeout(aoc->ipc_wq,
				   lga_aoc_read_cb_magic(cb) == LGA_AOC_CONTROL_MAGIC,
				   msecs_to_jiffies(LGA_AOC_START_POLL_MS));
	}

	ret = lga_aoc_discover_services(aoc);
	if (ret)
		return ret;

	/* Release boot vote on aoc_core_pd once AoC reports online */
	lga_aoc_core_pd_set(aoc, false);
	*became_online = true;
	return 0;
}

static int lga_aoc_stop_locked(struct lga_aoc *aoc, bool *was_online)
{
	int ret;

	*was_online = aoc->online;
	aoc->online = false;
	wake_up_all(&aoc->ipc_wq);

	ret = gsa_aoc_stop(aoc->gsa);
	lga_aoc_set_supplies(aoc, false);
	lga_aoc_core_pd_set(aoc, false);
	lga_aoc_unmap_iommu(aoc);

	kfree(aoc->services);
	aoc->services = NULL;
	aoc->num_services = 0;
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
	int ret;

	if (!aoc || mbox_idx >= aoc->num_mboxes || !aoc->mboxes[mbox_idx].chan)
		return -EINVAL;

	/* Pure doorbell signal works for both payload-size=0 and 1 channels */
	ret = mbox_send_message(aoc->mboxes[mbox_idx].chan, NULL);
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

static bool lga_aoc_can_read_locked(struct lga_aoc *aoc, u32 service_idx)
{
	struct lga_aoc_ipc_service_hdr *hdr;
	struct lga_aoc_ipc_region *reg;
	u32 slots, tx, rx;

	hdr = lga_aoc_service_hdr(aoc, service_idx);
	if (!hdr)
		return false;

	if (aoc->services[service_idx].type == LGA_AOC_SVC_BUFFER)
		return false;

	reg = &hdr->regions[LGA_AOC_DIR_UP];
	slots = le32_to_cpu(READ_ONCE(reg->slots));
	if (!slots)
		return false;

	tx = le32_to_cpu(READ_ONCE(reg->tx));
	rx = le32_to_cpu(READ_ONCE(reg->rx));
	return tx != rx;
}

static bool lga_aoc_can_write_locked(struct lga_aoc *aoc, u32 service_idx,
				     size_t min_bytes)
{
	struct lga_aoc_ipc_service_hdr *hdr;
	struct lga_aoc_ipc_region *reg;
	u32 type, slots, size, tx, rx;

	hdr = lga_aoc_service_hdr(aoc, service_idx);
	if (!hdr)
		return false;

	type = aoc->services[service_idx].type;
	if (type == LGA_AOC_SVC_BUFFER)
		return false;

	reg = &hdr->regions[LGA_AOC_DIR_DOWN];
	slots = le32_to_cpu(READ_ONCE(reg->slots));
	size = le32_to_cpu(READ_ONCE(reg->size));
	if (!slots || !size)
		return false;

	tx = le32_to_cpu(READ_ONCE(reg->tx));
	rx = le32_to_cpu(READ_ONCE(reg->rx));

	if (type == LGA_AOC_SVC_RING) {
		size_t avail = size - min_t(size_t, (u32)(tx - rx), size);

		return avail >= min_bytes;
	}

	return (u32)(tx - rx) < slots;
}

static bool lga_aoc_ipc_can_write_len(struct device *dev, u32 service_idx,
				      size_t min_bytes)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	bool ready;

	if (!aoc)
		return false;

	mutex_lock(&aoc->lock);
	ready = lga_aoc_can_write_locked(aoc, service_idx, min_bytes);
	mutex_unlock(&aoc->lock);
	return ready;
}

bool lga_aoc_ipc_can_read(struct device *dev, u32 service_idx)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	bool ready;

	if (!aoc)
		return false;

	mutex_lock(&aoc->lock);
	ready = lga_aoc_can_read_locked(aoc, service_idx);
	mutex_unlock(&aoc->lock);
	return ready;
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_can_read);

bool lga_aoc_ipc_can_write(struct device *dev, u32 service_idx)
{
	return lga_aoc_ipc_can_write_len(dev, service_idx, 1U);
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_can_write);

ssize_t lga_aoc_ipc_write(struct device *dev, u32 service_idx,
			  const void *buf, size_t len)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	struct lga_aoc_ipc_service_hdr *hdr;
	struct lga_aoc_ipc_region *reg;
	struct lga_aoc_ipc_msg_hdr mhdr = {};
	u32 offset, size, slots, tx, rx, wp, mbox_idx, type, flags;
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
	flags = aoc->services[service_idx].flags;
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
		u32 slot_idx;

		if ((u32)(tx - rx) >= slots) {
			ret = -ENOSPC;
			goto out;
		}
		if (size <= sizeof(mhdr) || len > size - sizeof(mhdr) ||
		    len > U16_MAX) {
			ret = -EMSGSIZE;
			goto out;
		}
		slot_idx = tx % slots;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset +
			   (size_t)slot_idx * size;
		mhdr.length = cpu_to_le16(len);
		memcpy(slot_ptr, &mhdr, sizeof(mhdr));
		memcpy(slot_ptr + sizeof(mhdr), buf, len);
		dma_wmb();
		WRITE_ONCE(reg->tx, cpu_to_le32(tx + 1U));
		WRITE_ONCE(reg->wp, cpu_to_le32((wp + 1U) % size));
	} else if (type == LGA_AOC_SVC_RING) {
		size_t avail = size - min_t(size_t, (u32)(tx - rx), size);
		size_t first;

		if (len > size) {
			ret = -EMSGSIZE;
			goto out;
		}
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
		WRITE_ONCE(reg->tx, cpu_to_le32(tx + len));
		WRITE_ONCE(reg->wp, cpu_to_le32((wp + len) % size));
	} else {
		ret = -EOPNOTSUPP;
		goto out;
	}

	dma_wmb();
	if ((type != LGA_AOC_SVC_RING || (flags & LGA_AOC_SVC_FLAG_RING_PUSH)) &&
	    mbox_idx < aoc->num_mboxes)
		lga_aoc_ring_doorbell(dev, mbox_idx);

	ret = len;
out:
	mutex_unlock(&aoc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_write);

ssize_t lga_aoc_ipc_write_timeout(struct device *dev, u32 service_idx,
				  const void *buf, size_t len,
				  unsigned long timeout)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	unsigned long deadline;
	ssize_t ret;

	if (!aoc || !buf || !len)
		return -EINVAL;

	ret = lga_aoc_ipc_write(dev, service_idx, buf, len);
	if (ret != -ENOSPC || !timeout)
		return ret;

	deadline = jiffies + timeout;
	for (;;) {
		long rem = deadline - jiffies;
		long wret;

		if (rem <= 0)
			return -ETIMEDOUT;

		wret = wait_event_interruptible_timeout(aoc->ipc_wq,
							!lga_aoc_is_online(dev) ||
							lga_aoc_ipc_can_write_len(dev,
										  service_idx,
										  len),
							rem);
		if (wret < 0)
			return wret;
		if (!lga_aoc_is_online(dev))
			return -ENODEV;

		ret = lga_aoc_ipc_write(dev, service_idx, buf, len);
		if (ret != -ENOSPC)
			return ret;
		if (wret == 0)
			return -ETIMEDOUT;
	}
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_write_timeout);

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
		u32 slot_idx;

		if (size <= sizeof(mhdr)) {
			ret = -EIO;
			goto out;
		}
		slot_idx = rx % slots;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset +
			   (size_t)slot_idx * size;
		memcpy(&mhdr, slot_ptr, sizeof(mhdr));
		msg_len = le16_to_cpu(mhdr.length);
		if (msg_len > size - sizeof(mhdr) || msg_len > max_len) {
			ret = -EMSGSIZE;
			goto out;
		}
		memcpy(buf, slot_ptr + sizeof(mhdr), msg_len);
		dma_wmb();
		WRITE_ONCE(reg->rx, cpu_to_le32(rx + 1U));
		WRITE_ONCE(reg->rp, cpu_to_le32((rp + 1U) % size));
		ret = msg_len;
	} else if (type == LGA_AOC_SVC_RING) {
		u32 delta = tx - rx;
		size_t avail, to_read, first;

		if (delta > size) {
			u32 skip = delta - size;

			rx += skip;
			rp = (rp + skip) % size;
			WRITE_ONCE(reg->rx, cpu_to_le32(rx));
			WRITE_ONCE(reg->rp, cpu_to_le32(rp));
			delta = size;
		}
		avail = delta;
		to_read = min(avail, max_len);
		rp %= size;
		slot_ptr = aoc->dram + aoc->ipc_offset + offset;
		first = min_t(size_t, to_read, size - rp);
		memcpy(buf, slot_ptr + rp, first);
		if (to_read > first)
			memcpy(buf + first, slot_ptr, to_read - first);
		dma_wmb();
		WRITE_ONCE(reg->rx, cpu_to_le32(rx + to_read));
		WRITE_ONCE(reg->rp, cpu_to_le32((rp + to_read) % size));
		ret = to_read;
	} else {
		ret = -EOPNOTSUPP;
	}
out:
	mutex_unlock(&aoc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_read);

ssize_t lga_aoc_ipc_read_timeout(struct device *dev, u32 service_idx,
				 void *buf, size_t max_len,
				 unsigned long timeout)
{
	struct lga_aoc *aoc = dev_get_drvdata(dev);
	unsigned long deadline;
	ssize_t ret;

	if (!aoc || !buf || !max_len)
		return -EINVAL;

	ret = lga_aoc_ipc_read(dev, service_idx, buf, max_len);
	if (ret != -EAGAIN || !timeout)
		return ret;

	deadline = jiffies + timeout;
	for (;;) {
		long rem = deadline - jiffies;
		long wret;

		if (rem <= 0)
			return -ETIMEDOUT;

		wret = wait_event_interruptible_timeout(aoc->ipc_wq,
							!lga_aoc_is_online(dev) ||
							lga_aoc_ipc_can_read(dev,
									     service_idx),
							rem);
		if (wret < 0)
			return wret;
		if (!lga_aoc_is_online(dev))
			return -ENODEV;

		ret = lga_aoc_ipc_read(dev, service_idx, buf, max_len);
		if (ret != -EAGAIN)
			return ret;
		if (wret == 0)
			return -ETIMEDOUT;
	}
}
EXPORT_SYMBOL_GPL(lga_aoc_ipc_read_timeout);

/* debugfs bring-up interface */

static int lga_aoc_info_show(struct seq_file *s, void *unused)
{
	struct lga_aoc *aoc = s->private;
	int gsa_state;

	gsa_state = gsa_send_aoc_cmd(aoc->gsa, GSA_AOC_GET_STATE);

	seq_printf(s, "board_cfg: %s\n", aoc->board_cfg ?: "unknown");
	seq_printf(s, "board_id: %#x\n", aoc->board_id);
	seq_printf(s, "board_rev: %#x\n", aoc->board_rev);
	seq_printf(s, "chip_product_id: %u chip_rev: %#x\n",
		   aoc->chip_product_id, aoc->chip_rev);
	seq_printf(s, "gnss_type: %u wifi_chip: %u force_vnom: %u\n",
		   aoc->gnss_type, aoc->wifi_chip, aoc->force_vnom);
	seq_printf(s, "firmware_name: %s\n", aoc->fw_name);
	seq_printf(s, "carveout: %pa + %#zx\n", &aoc->dram_base, aoc->dram_size);
	seq_printf(s, "mboxes: %u (coredump_idx=%u)\n",
		   aoc->num_mboxes, aoc->coredump_mbox);
	seq_printf(s, "gsa_aoc_state: %d (%s)\n",
		   gsa_state, lga_aoc_gsa_state_str(gsa_state));
	seq_printf(s, "sswrp_pd_on: %u core_pd_on: %u supplies_enabled: %u iommu_maps: %u\n",
		   aoc->sswrp_pd_on, aoc->core_pd_on, aoc->supplies_enabled,
		   aoc->num_iommu_maps);
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
	bool use_default;
	const char *fw;
	int ret;

	if (!count || count >= sizeof(name))
		return -EINVAL;
	if (copy_from_user(name, ubuf, count))
		return -EFAULT;
	name[count] = '\0';
	strim(name);

	use_default = !strcmp(name, "1") || !*name;
	fw = use_default ? aoc->fw_name : name;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_load_locked(aoc, fw, use_default);
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
	bool en, became_online = false;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &en);
	if (ret)
		return ret;
	if (!en)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_start_locked(aoc, &became_online);
	mutex_unlock(&aoc->lock);

	if (became_online)
		blocking_notifier_call_chain(&lga_aoc_notifiers,
					     LGA_AOC_EVENT_ONLINE, aoc->dev);

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
	bool en, was_online = false;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &en);
	if (ret)
		return ret;
	if (!en)
		return -EINVAL;

	mutex_lock(&aoc->lock);
	ret = lga_aoc_stop_locked(aoc, &was_online);
	mutex_unlock(&aoc->lock);

	if (was_online)
		blocking_notifier_call_chain(&lga_aoc_notifiers,
					     LGA_AOC_EVENT_OFFLINE, aoc->dev);

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

		seq_printf(s, "  %3u %-32s %-6s flags=%#010x mbox=%2u down=%ux%u@+%#x up=%ux%u@+%#x\n",
			   si->index, si->name, lga_aoc_svc_type_str(si->type),
			   si->flags, si->mbox_index,
			   si->down_slots, si->down_size, si->down_offset,
			   si->up_slots, si->up_size, si->up_offset);
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

static void lga_aoc_read_soc_info(struct lga_aoc *aoc)
{
	struct device_node *soc_np, *child;
	u32 prod_id, major, minor;

	aoc->chip_product_id = LGA_AOC_DEFAULT_PRODUCT_ID;
	aoc->chip_rev = LGA_AOC_DEFAULT_CHIP_REV;

	soc_np = of_find_node_by_name(NULL, "soc_compatible");
	if (!soc_np)
		return;

	child = of_get_next_child(soc_np, NULL);
	if (child) {
		if (!of_property_read_u32(child, "product_id", &prod_id))
			aoc->chip_product_id = prod_id;
		if (!of_property_read_u32(child, "major", &major) &&
		    !of_property_read_u32(child, "minor", &minor))
			aoc->chip_rev = ((major & 0xfU) << 4) | (minor & 0xfU);
		of_node_put(child);
	}
	of_node_put(soc_np);
}

static void lga_aoc_detach_pds(void *data)
{
	struct lga_aoc *aoc = data;
	int i;

	if (aoc->core_pd_on && !IS_ERR_OR_NULL(aoc->pd_devs[1])) {
		pm_runtime_put_sync(aoc->pd_devs[1]);
		aoc->core_pd_on = false;
	}
	if (aoc->sswrp_pd_on && !IS_ERR_OR_NULL(aoc->pd_devs[0])) {
		pm_runtime_put_sync(aoc->pd_devs[0]);
		aoc->sswrp_pd_on = false;
	}

	for (i = ARRAY_SIZE(aoc->pd_devs) - 1; i >= 0; i--) {
		if (!IS_ERR_OR_NULL(aoc->pd_devs[i])) {
			dev_pm_domain_detach(aoc->pd_devs[i], true);
			aoc->pd_devs[i] = NULL;
		}
	}
}

static int lga_aoc_attach_pds(struct lga_aoc *aoc)
{
	static const char * const pd_names[] = { "sswrp_aoc_pd", "aoc_core_pd" };
	int i, ret;

	if (of_count_phandle_with_args(aoc->dev->of_node, "power-domains",
				       "#power-domain-cells") < 2)
		return 0;

	for (i = 0; i < ARRAY_SIZE(pd_names); i++) {
		struct device *pd_dev;

		pd_dev = dev_pm_domain_attach_by_name(aoc->dev, pd_names[i]);
		if (IS_ERR(pd_dev)) {
			lga_aoc_detach_pds(aoc);
			return PTR_ERR(pd_dev);
		}
		aoc->pd_devs[i] = pd_dev;
	}

	if (aoc->pd_devs[0]) {
		ret = pm_runtime_resume_and_get(aoc->pd_devs[0]);
		if (ret) {
			dev_err(aoc->dev, "failed to power on sswrp_aoc_pd: %d\n",
				ret);
			lga_aoc_detach_pds(aoc);
			return ret;
		}
		aoc->sswrp_pd_on = true;
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
	struct resource rmem, *lcpm_res;
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

	lcpm_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lcpm");
	if (lcpm_res) {
		aoc->lcpm = devm_ioremap_resource(dev, lcpm_res);
		if (IS_ERR(aoc->lcpm)) {
			dev_warn(dev, "optional lcpm map failed: %ld\n",
				 PTR_ERR(aoc->lcpm));
			aoc->lcpm = NULL;
		} else {
			aoc->lcpm_size = resource_size(lcpm_res);
			if (aoc->lcpm_size >= LGA_AOC_LCPM_INIT_OFF + sizeof(u32))
				writel(0, aoc->lcpm + LGA_AOC_LCPM_INIT_OFF);
		}
	}

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
	aoc->board_id = LGA_AOC_DEFAULT_BOARD_ID;
	aoc->board_rev = LGA_AOC_DEFAULT_BOARD_REV;
	aoc->force_vnom = 1;
	of_property_read_u32(np, "google,coredump-mbox", &aoc->coredump_mbox);
	of_property_read_u32(np, "google,board-id", &aoc->board_id);
	of_property_read_u32(np, "google,board-rev", &aoc->board_rev);
	of_property_read_u32(np, "google,gnss-type", &aoc->gnss_type);
	of_property_read_u32(np, "google,wifi-chip", &aoc->wifi_chip);
	of_property_read_u32(np, "google,force-vnom", &aoc->force_vnom);
	of_property_read_string(np, "google,board-cfg", &aoc->board_cfg);
	if (of_property_read_string(np, "firmware-name", &aoc->fw_name))
		aoc->fw_name = LGA_AOC_DEFAULT_FW;
	lga_aoc_read_soc_info(aoc);

	platform_set_drvdata(pdev, aoc);
	pm_runtime_enable(dev);

	mutex_lock(&lga_aoc_dev_lock);
	lga_aoc_active_dev = aoc;
	mutex_unlock(&lga_aoc_dev_lock);

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
		 "Laguna AoC ready (cfg=%s board=%#x/%#x soc=%u/%#x carveout=%pa+%#zx mboxes=%u)\n",
		 aoc->board_cfg ?: "?", aoc->board_id, aoc->board_rev,
		 aoc->chip_product_id, aoc->chip_rev,
		 &aoc->dram_base, aoc->dram_size, aoc->num_mboxes);

	if (autoload) {
		bool became_online = false;

		mutex_lock(&aoc->lock);
		ret = lga_aoc_load_locked(aoc, aoc->fw_name, true);
		if (!ret)
			ret = lga_aoc_start_locked(aoc, &became_online);
		mutex_unlock(&aoc->lock);
		if (ret)
			dev_warn(dev, "autoload failed: %d\n", ret);
		else if (became_online)
			blocking_notifier_call_chain(&lga_aoc_notifiers,
						     LGA_AOC_EVENT_ONLINE, dev);
	}

	return 0;
}

static void lga_aoc_remove(struct platform_device *pdev)
{
	struct lga_aoc *aoc = platform_get_drvdata(pdev);
	bool was_online = false;

	debugfs_remove_recursive(aoc->debugfs);

	mutex_lock(&lga_aoc_dev_lock);
	if (lga_aoc_active_dev == aoc)
		lga_aoc_active_dev = NULL;
	mutex_unlock(&lga_aoc_dev_lock);

	mutex_lock(&aoc->lock);
	if (aoc->loaded || aoc->online)
		lga_aoc_stop_locked(aoc, &was_online);
	else
		lga_aoc_set_supplies(aoc, false);
	mutex_unlock(&aoc->lock);

	if (was_online)
		blocking_notifier_call_chain(&lga_aoc_notifiers,
					     LGA_AOC_EVENT_OFFLINE, aoc->dev);

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
