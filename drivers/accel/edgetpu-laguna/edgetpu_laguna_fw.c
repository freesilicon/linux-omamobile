// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room firmware loading, GSA authentication, image config mapping,
 * iremap pool management, and power sequencing for Laguna EdgeTPU.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/soc/google/gsa.h>
#include <linux/string.h>

#include "edgetpu_laguna.h"

static char *fw_name = LAGUNA_DEFAULT_FW_NAME;
module_param(fw_name, charp, 0444);
MODULE_PARM_DESC(fw_name, "EdgeTPU firmware image path under /lib/firmware");

static bool allow_nonsecure_fw;
module_param(allow_nonsecure_fw, bool, 0444);
MODULE_PARM_DESC(allow_nonsecure_fw, "Allow running unsigned non-secure firmware images");

static const struct laguna_image_config *
laguna_parse_fw_header(const struct firmware *fw, size_t *hdr_size_out)
{
	const struct laguna_fw_common_header *hdr;
	const struct laguna_fw_legacy_header *leg;

	if (fw->size < LAGUNA_FW_PQ_HDR_SIZE)
		return NULL;

	hdr = (const struct laguna_fw_common_header *)fw->data;
	leg = (const struct laguna_fw_legacy_header *)fw->data;

	if (hdr->format_version == LAGUNA_FW_FMT_RDO &&
	    hdr->rdo.sub_header.magic == LAGUNA_FW_MAGIC) {
		*hdr_size_out = LAGUNA_FW_LEGACY_HDR_SIZE;
		return &hdr->rdo.sub_header.image_config;
	}
	if (hdr->format_version == LAGUNA_FW_FMT_PQ &&
	    hdr->pq.sub_header.magic == LAGUNA_FW_MAGIC) {
		*hdr_size_out = LAGUNA_FW_PQ_HDR_SIZE;
		return &hdr->pq.sub_header.image_config;
	}
	if (leg->common.magic == LAGUNA_FW_MAGIC) {
		*hdr_size_out = LAGUNA_FW_LEGACY_HDR_SIZE;
		if (leg->common.generation == 1)
			return &leg->gen1.image_config;
		if (leg->common.generation == 2)
			return &leg->gen2.image_config;
	}
	return NULL;
}

static phys_addr_t laguna_fixed_addr(const struct laguna_image_config *cfg,
				     u32 addr, bool mmio)
{
	u64 hi = mmio ? FIELD_GET(LAGUNA_IMG_OFF_MMIO_MASK, cfg->mapping_offsets) :
			FIELD_GET(LAGUNA_IMG_OFF_MEM_MASK, cfg->mapping_offsets);

	return (phys_addr_t)((hi << 32) | addr);
}

static dma_addr_t laguna_cfg_daddr(u32 virt_address)
{
	dma_addr_t daddr = virt_address & ~LAGUNA_IMG_MAP_FLAGS_MASK;

	if (virt_address & LAGUNA_IMG_MAP_36BIT)
		daddr <<= 4;
	return daddr;
}

static size_t laguna_cfg_size(u32 val, bool ns_entry)
{
	if (val & LAGUNA_IMG_CFG_SIZE_MODE)
		return (size_t)(val & LAGUNA_IMG_CFG_SIZE_MASK) << 12;

	if (ns_entry)
		return (size_t)(val & LAGUNA_IMG_CFG_SIZE_MASK) << 20;

	return BIT(val & LAGUNA_IMG_CFG_SIZE_MASK) << 12;
}

static void laguna_clear_img_mappings(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_img_mapping *cur, *tmp;
	LIST_HEAD(to_free);

	mutex_lock(&etdev->img_map_lock);
	list_splice_init(&etdev->img_mappings, &to_free);
	mutex_unlock(&etdev->img_map_lock);

	list_for_each_entry_safe(cur, tmp, &to_free, node) {
		list_del(&cur->node);
		if (!(cur->cfg_map_flags & LAGUNA_IMG_MAP_SHARED))
			edgetpu_laguna_iommu_unmap(etdev->default_domain,
						   cur->daddr, cur->size);
		if (cur->sgt) {
			unsigned int i;

			sg_free_table(cur->sgt);
			kfree(cur->sgt);
			for (i = 0; i < cur->nr_pages; i++)
				if (cur->pages[i])
					__free_page(cur->pages[i]);
			kvfree(cur->pages);
		}
		kfree(cur);
	}
}

static int laguna_add_ns_alloc_mapping(struct edgetpu_laguna_dev *etdev,
				       dma_addr_t daddr, size_t size)
{
	unsigned int nr_pages = size >> PAGE_SHIFT;
	struct laguna_img_mapping *map;
	struct page **pages;
	struct sg_table *sgt;
	unsigned int i;
	int ret;

	if (!nr_pages)
		return -EINVAL;

	pages = kvcalloc(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < nr_pages; i++) {
		pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!pages[i]) {
			ret = -ENOMEM;
			goto err_pages;
		}
	}

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto err_pages;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, nr_pages, 0, size, GFP_KERNEL);
	if (ret)
		goto err_sgt_struct;

	ret = edgetpu_laguna_iommu_map_sgt(etdev->default_domain, daddr, sgt, 0);
	if (ret)
		goto err_sgt;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	map->daddr = daddr;
	map->size = size;
	map->sgt = sgt;
	map->pages = pages;
	map->nr_pages = nr_pages;

	mutex_lock(&etdev->img_map_lock);
	list_add_tail(&map->node, &etdev->img_mappings);
	mutex_unlock(&etdev->img_map_lock);
	return 0;

err_unmap:
	edgetpu_laguna_iommu_unmap(etdev->default_domain, daddr, size);
err_sgt:
	sg_free_table(sgt);
err_sgt_struct:
	kfree(sgt);
err_pages:
	while (i--)
		__free_page(pages[i]);
	kvfree(pages);
	return ret;
}

static int laguna_apply_img_config(struct edgetpu_laguna_dev *etdev,
				   const struct laguna_image_config *cfg)
{
	bool is_ns = (cfg->privilege_level == LAGUNA_FW_PRIV_NS);
	u32 i;
	int ret;

	laguna_clear_img_mappings(etdev);

	for (i = 0; i < min_t(u32, cfg->num_iommu_mappings,
			      LAGUNA_IMG_MAX_IOMMU_MAPS); i++) {
		u32 virt = cfg->iommu_mappings[i].virt_address;
		u32 val = cfg->iommu_mappings[i].image_config_value;
		u32 flags = virt & LAGUNA_IMG_MAP_FLAGS_MASK;
		bool mmio = !!(flags & LAGUNA_IMG_MAP_MMIO);
		bool shared = !!(flags & LAGUNA_IMG_MAP_SHARED);
		dma_addr_t daddr = laguna_cfg_daddr(virt);
		size_t size = laguna_cfg_size(val, false);
		phys_addr_t paddr;

		if (!is_ns && !shared)
			continue;
		if (!daddr || !size)
			continue;

		paddr = laguna_fixed_addr(cfg, val & LAGUNA_IMG_CFG_ADDR_MASK, mmio);
		if (shared) {
			struct laguna_img_mapping *map = kzalloc(sizeof(*map), GFP_KERNEL);

			if (!map) {
				ret = -ENOMEM;
				goto err_clear;
			}
			map->daddr = daddr;
			map->paddr = paddr;
			map->size = size;
			map->cfg_map_flags = flags;
			mutex_lock(&etdev->img_map_lock);
			list_add_tail(&map->node, &etdev->img_mappings);
			mutex_unlock(&etdev->img_map_lock);
		} else {
			ret = edgetpu_laguna_iommu_map(etdev->default_domain,
						       daddr, paddr, size, flags);
			if (ret)
				goto err_clear;
		}
	}

	for (i = 0; i < min_t(u32, cfg->num_ns_iommu_mappings,
			      LAGUNA_IMG_MAX_NS_IOMMU_MAPS); i++) {
		u32 entry = cfg->ns_iommu_mappings[i];
		dma_addr_t daddr = entry & LAGUNA_IMG_CFG_ADDR_MASK;
		size_t size = laguna_cfg_size(entry, true);

		if (!daddr || !size)
			continue;
		ret = laguna_add_ns_alloc_mapping(etdev, daddr, size);
		if (ret)
			goto err_clear;
	}

	return 0;

err_clear:
	laguna_clear_img_mappings(etdev);
	return ret;
}

int edgetpu_laguna_fw_shared_context_map(struct edgetpu_laguna_dev *etdev,
					 struct laguna_domain *etdom)
{
	struct laguna_img_mapping *cur, *pos;
	int ret = 0;

	if (etdom->shared_mapped)
		return 0;

	mutex_lock(&etdev->img_map_lock);
	list_for_each_entry(cur, &etdev->img_mappings, node) {
		if (cur->sgt || !(cur->cfg_map_flags & LAGUNA_IMG_MAP_SHARED))
			continue;

		ret = edgetpu_laguna_iommu_map(etdom->domain, cur->daddr,
					       cur->paddr, cur->size,
					       cur->cfg_map_flags);
		if (ret) {
			dev_err(etdev->dev,
				"shared img map %pad failed pasid=%u: %d\n",
				&cur->daddr, etdom->pasid, ret);
			list_for_each_entry(pos, &etdev->img_mappings, node) {
				if (pos == cur)
					break;
				if (!pos->sgt &&
				    (pos->cfg_map_flags & LAGUNA_IMG_MAP_SHARED))
					edgetpu_laguna_iommu_unmap(etdom->domain,
								   pos->daddr,
								   pos->size);
			}
			goto out_unlock;
		}
	}
	etdom->shared_mapped = true;

out_unlock:
	mutex_unlock(&etdev->img_map_lock);
	return ret;
}

void edgetpu_laguna_fw_shared_context_unmap(struct edgetpu_laguna_dev *etdev,
					    struct laguna_domain *etdom)
{
	struct laguna_img_mapping *cur;

	if (!etdom->shared_mapped)
		return;

	mutex_lock(&etdev->img_map_lock);
	list_for_each_entry(cur, &etdev->img_mappings, node) {
		if (!cur->sgt && (cur->cfg_map_flags & LAGUNA_IMG_MAP_SHARED))
			edgetpu_laguna_iommu_unmap(etdom->domain,
						   cur->daddr, cur->size);
	}
	etdom->shared_mapped = false;
	mutex_unlock(&etdev->img_map_lock);
}

static int laguna_setup_shared_region(struct edgetpu_laguna_dev *etdev,
				      const struct laguna_image_config *cfg)
{
	phys_addr_t paddr;
	dma_addr_t daddr;
	size_t size, pool_off;
	u32 tcfg = cfg->telemetry_buffer_config;
	int ret;

	if (tcfg) {
		etdev->num_telemetry = tcfg & 0xffU;
		etdev->log_size = ((tcfg >> 8) & 0xffU) * SZ_4K;
		etdev->trace_size = ((tcfg >> 16) & 0xffU) * SZ_4K;
	} else {
		etdev->num_telemetry = LAGUNA_NUM_CORES;
		etdev->log_size = LAGUNA_FW_LOG_BUF_SIZE;
		etdev->trace_size = LAGUNA_FW_TRACE_BUF_SIZE;
	}

	pool_off = (etdev->log_size + etdev->trace_size) * etdev->num_telemetry;
	daddr = cfg->shared_data_iova ? cfg->shared_data_iova :
		(LAGUNA_FW_IREMAP_BASE + etdev->fw_region_size);
	paddr = (cfg->shared_data_start && cfg->shared_data_size) ?
		laguna_fixed_addr(cfg, cfg->shared_data_start, false) :
		(etdev->fw_paddr + etdev->fw_region_size);
	size = (cfg->shared_data_start && cfg->shared_data_size) ?
	       cfg->shared_data_size : LAGUNA_FW_DEFAULT_DATA_SIZE;

	if (size <= pool_off)
		return -EINVAL;

	if (etdev->shared_vaddr && etdev->shared_paddr == paddr &&
	    etdev->shared_daddr == daddr && etdev->shared_size == size)
		return 0;

	if (etdev->shared_vaddr) {
		edgetpu_laguna_kci_exit(etdev);
		edgetpu_laguna_ikv_exit(etdev);
		gen_pool_destroy(etdev->iremap_pool);
		etdev->iremap_pool = NULL;
		memunmap(etdev->shared_vaddr);
		etdev->shared_vaddr = NULL;
	}

	etdev->shared_vaddr = memremap(paddr, size, MEMREMAP_WC);
	if (!etdev->shared_vaddr)
		return -ENOMEM;

	memset(etdev->shared_vaddr, 0, pool_off);
	etdev->shared_paddr = paddr;
	etdev->shared_daddr = daddr;
	etdev->shared_size = size;

	etdev->iremap_pool = gen_pool_create(ilog2(64), -1);
	if (!etdev->iremap_pool) {
		memunmap(etdev->shared_vaddr);
		etdev->shared_vaddr = NULL;
		return -ENOMEM;
	}

	etdev->iremap_vaddr = (unsigned long)etdev->shared_vaddr + pool_off;
	etdev->iremap_daddr = daddr + pool_off;
	etdev->iremap_paddr = paddr + pool_off;

	ret = gen_pool_add_virt(etdev->iremap_pool, etdev->iremap_vaddr,
				etdev->iremap_paddr, size - pool_off, -1);
	if (ret) {
		gen_pool_destroy(etdev->iremap_pool);
		etdev->iremap_pool = NULL;
		memunmap(etdev->shared_vaddr);
		etdev->shared_vaddr = NULL;
		return ret;
	}

	return 0;
}

int edgetpu_laguna_iremap_alloc(struct edgetpu_laguna_dev *etdev, size_t size,
				struct laguna_iremap_mem *mem)
{
	unsigned long vaddr;

	memset(mem, 0, sizeof(*mem));
	if (!etdev->iremap_pool || !size)
		return -ENODEV;

	vaddr = gen_pool_alloc(etdev->iremap_pool, size);
	if (!vaddr)
		return -ENOMEM;

	mem->vaddr = (void *)vaddr;
	mem->daddr = etdev->iremap_daddr + (vaddr - etdev->iremap_vaddr);
	mem->paddr = etdev->iremap_paddr + (vaddr - etdev->iremap_vaddr);
	mem->size = size;
	return 0;
}

void edgetpu_laguna_iremap_free(struct edgetpu_laguna_dev *etdev,
				struct laguna_iremap_mem *mem)
{
	if (mem->vaddr && etdev->iremap_pool)
		gen_pool_free(etdev->iremap_pool, (unsigned long)mem->vaddr, mem->size);
	memset(mem, 0, sizeof(*mem));
}

static int laguna_gsa_auth(struct edgetpu_laguna_dev *etdev,
			   const struct firmware *fw, size_t hdr_size,
			   void *carveout_vaddr)
{
	void *hdr_va;
	dma_addr_t hdr_da;
	int state, ret;

	if (!etdev->gsa_dev)
		return -ENODEV;

	state = gsa_send_tpu_cmd(etdev->gsa_dev, GSA_TPU_GET_STATE);
	if (state < 0)
		return state;

	if (state > GSA_TPU_STATE_INACTIVE) {
		ret = gsa_unload_tpu_fw_image(etdev->gsa_dev);
		if (ret)
			return ret;
	}

	memcpy(carveout_vaddr, fw->data + hdr_size, fw->size - hdr_size);

	hdr_va = dma_alloc_coherent(etdev->gsa_dev, hdr_size, &hdr_da, GFP_KERNEL);
	if (!hdr_va)
		return -ENOMEM;

	memcpy(hdr_va, fw->data, hdr_size);
	ret = gsa_load_tpu_fw_image(etdev->gsa_dev, hdr_da, etdev->fw_paddr);
	dma_free_coherent(etdev->gsa_dev, hdr_size, hdr_va, hdr_da);
	return ret;
}

static int laguna_reset_cpu(struct edgetpu_laguna_dev *etdev, bool assert_reset)
{
	u32 i;

	if (etdev->fw_is_ns) {
		if (!assert_reset) {
			for (i = 0; i < LAGUNA_NUM_CORES; i++) {
				laguna_write32(etdev, LAGUNA_REG_IREMAP_AXIUSER_CORE0 + i * 8, 0);
				laguna_write32(etdev, LAGUNA_REG_AXIUSER_CORE0 + i * 8, 0);
				laguna_write32(etdev, LAGUNA_REG_IREMAP_NEW_BASE + i * 8,
					       LAGUNA_FW_IREMAP_BASE);
				laguna_write32(etdev, LAGUNA_REG_IREMAP_LIMIT + i * 8,
					       LAGUNA_FW_IREMAP_BASE + SZ_32M);
				laguna_write32(etdev, LAGUNA_REG_IREMAP_CONTROL + i * 8, 1);
			}
		}
		for (i = 0; i < LAGUNA_NUM_CORES; i++)
			laguna_write32(etdev, LAGUNA_REG_RESET_CONTROL + i * 8,
				       assert_reset ? 1U : 0U);
		return 0;
	}

	if (!etdev->gsa_dev)
		return -ENODEV;

	return gsa_send_tpu_cmd(etdev->gsa_dev,
				assert_reset ? GSA_TPU_SHUTDOWN : GSA_TPU_START);
}

int edgetpu_laguna_fw_init(struct edgetpu_laguna_dev *etdev)
{
	struct resource res;
	void *clear_va;
	int ret;

	mutex_init(&etdev->fw_lock);
	mutex_init(&etdev->img_map_lock);
	INIT_LIST_HEAD(&etdev->img_mappings);

	ret = of_reserved_mem_region_to_resource(etdev->dev->of_node, 0, &res);
	if (ret) {
		dev_err(etdev->dev, "missing firmware reserved-memory region: %d\n", ret);
		return ret;
	}

	etdev->fw_paddr = res.start;
	etdev->fw_region_size = LAGUNA_FW_DEFAULT_CODE_LIMIT;

	clear_va = memremap(res.start, resource_size(&res), MEMREMAP_WC);
	if (clear_va) {
		memset(clear_va, 0, resource_size(&res));
		memunmap(clear_va);
	}

	return 0;
}

void edgetpu_laguna_fw_exit(struct edgetpu_laguna_dev *etdev)
{
	laguna_clear_img_mappings(etdev);
	edgetpu_laguna_ikv_exit(etdev);
	edgetpu_laguna_kci_exit(etdev);
	if (etdev->iremap_pool) {
		gen_pool_destroy(etdev->iremap_pool);
		etdev->iremap_pool = NULL;
	}
	if (etdev->shared_vaddr) {
		memunmap(etdev->shared_vaddr);
		etdev->shared_vaddr = NULL;
	}
}

int edgetpu_laguna_fw_boot(struct edgetpu_laguna_dev *etdev)
{
	const struct firmware *fw = NULL;
	const struct laguna_image_config *cfg;
	size_t hdr_size = 0, body_size;
	struct resource rmem;
	void *fw_va;
	int ret;

	ret = of_reserved_mem_region_to_resource(etdev->dev->of_node, 0, &rmem);
	if (ret)
		return ret;

	ret = request_firmware(&fw, fw_name, etdev->dev);
	if (ret) {
		dev_err(etdev->dev, "request_firmware(%s) failed: %d\n", fw_name, ret);
		return ret;
	}

	cfg = laguna_parse_fw_header(fw, &hdr_size);
	if (!cfg) {
		dev_err(etdev->dev, "invalid EdgeTPU firmware header\n");
		ret = -EINVAL;
		goto out_rel;
	}

	body_size = fw->size - hdr_size;
	if (!body_size || body_size > resource_size(&rmem)) {
		ret = -ERANGE;
		goto out_rel;
	}

	etdev->fw_is_ns = (cfg->privilege_level == LAGUNA_FW_PRIV_NS);
	if (etdev->fw_is_ns && !allow_nonsecure_fw) {
		dev_err(etdev->dev, "non-secure firmware rejected (allow_nonsecure_fw=0)\n");
		ret = -EACCES;
		goto out_rel;
	}

	etdev->fw_paddr = laguna_fixed_addr(cfg, cfg->firmware_base, false);
	if (cfg->shared_data_start)
		etdev->fw_region_size = laguna_fixed_addr(cfg, cfg->shared_data_start, false) -
					etdev->fw_paddr;
	else
		etdev->fw_region_size = LAGUNA_FW_DEFAULT_CODE_LIMIT;

	if (body_size > etdev->fw_region_size) {
		ret = -ERANGE;
		goto out_rel;
	}

	memcpy(etdev->fw_versions, cfg->firmware_versions, sizeof(etdev->fw_versions));

	fw_va = memremap(etdev->fw_paddr, etdev->fw_region_size, MEMREMAP_WC);
	if (!fw_va) {
		ret = -ENOMEM;
		goto out_rel;
	}

	if (etdev->gsa_dev) {
		ret = laguna_gsa_auth(etdev, fw, hdr_size, fw_va);
		if (ret && etdev->fw_is_ns) {
			dev_warn(etdev->dev, "GSA auth failed on NS image (%d), loading unsigned\n",
				 ret);
			memcpy(fw_va, fw->data + hdr_size, body_size);
			ret = 0;
		}
	} else if (etdev->fw_is_ns) {
		memcpy(fw_va, fw->data + hdr_size, body_size);
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	memunmap(fw_va);
	if (ret)
		goto out_rel;

	ret = laguna_apply_img_config(etdev, cfg);
	if (ret)
		goto out_rel;

	ret = laguna_setup_shared_region(etdev, cfg);
	if (ret)
		goto out_rel;

	if (!etdev->kci.mbox) {
		ret = edgetpu_laguna_kci_init(etdev);
		if (ret)
			goto out_rel;
	} else {
		ret = edgetpu_laguna_kci_reinit(etdev);
		if (ret)
			goto out_rel;
	}

	if (!etdev->ikv.mbox) {
		ret = edgetpu_laguna_ikv_init(etdev);
		if (ret)
			goto out_rel;
	} else {
		ret = edgetpu_laguna_ikv_reinit(etdev);
		if (ret)
			goto out_rel;
	}

	ret = laguna_reset_cpu(etdev, false);
	if (ret < 0)
		goto out_rel;

	ret = edgetpu_laguna_kci_exchange_info(etdev, &etdev->fw_info);
	if (ret) {
		dev_err(etdev->dev, "KCI firmware handshake failed: %d\n", ret);
		goto out_stop;
	}

	ret = edgetpu_laguna_kci_map_telemetry(etdev);
	if (ret)
		dev_warn(etdev->dev, "KCI telemetry setup warning: %d\n", ret);

	etdev->fw_loaded = true;
	dev_info(etdev->dev, "EdgeTPU firmware ready (version %u.%u, cl=%u)\n",
		 etdev->fw_versions[0], etdev->fw_versions[1],
		 etdev->fw_info.fw_changelist);
	release_firmware(fw);
	return 0;

out_stop:
	laguna_reset_cpu(etdev, true);
out_rel:
	release_firmware(fw);
	return ret;
}

int edgetpu_laguna_fw_restart(struct edgetpu_laguna_dev *etdev)
{
	int ret;

	ret = edgetpu_laguna_kci_reinit(etdev);
	if (ret)
		return ret;
	ret = edgetpu_laguna_ikv_reinit(etdev);
	if (ret)
		return ret;

	ret = laguna_reset_cpu(etdev, false);
	if (ret < 0)
		return ret;

	ret = edgetpu_laguna_kci_exchange_info(etdev, &etdev->fw_info);
	if (ret) {
		laguna_reset_cpu(etdev, true);
		return ret;
	}

	edgetpu_laguna_kci_map_telemetry(etdev);
	return 0;
}

void edgetpu_laguna_fw_stop(struct edgetpu_laguna_dev *etdev)
{
	int tries;

	if (!etdev->fw_loaded)
		return;

	edgetpu_laguna_kci_shutdown(etdev);

	for (tries = 0; tries < 100; tries++) {
		if (!(laguna_read32(etdev, LAGUNA_REG_LPM_CONTROL) &
		      LAGUNA_LPM_CTL_PWR_STATE))
			break;
		usleep_range(200, 250);
	}

	laguna_reset_cpu(etdev, true);
}

int edgetpu_laguna_pm_get(struct edgetpu_laguna_dev *etdev)
{
	int ret = 0;

	mutex_lock(&etdev->pm_lock);
	if (etdev->power_ref++) {
		mutex_unlock(&etdev->pm_lock);
		return 0;
	}

	ret = pm_runtime_resume_and_get(etdev->dev);
	if (ret < 0) {
		etdev->power_ref--;
		mutex_unlock(&etdev->pm_lock);
		return ret;
	}

	mutex_lock(&etdev->fw_lock);
	if (!etdev->fw_loaded)
		ret = edgetpu_laguna_fw_boot(etdev);
	else
		ret = edgetpu_laguna_fw_restart(etdev);
	mutex_unlock(&etdev->fw_lock);

	if (ret) {
		pm_runtime_put_sync(etdev->dev);
		etdev->power_ref--;
	} else {
		etdev->powered = true;
	}
	mutex_unlock(&etdev->pm_lock);
	return ret;
}

void edgetpu_laguna_pm_put(struct edgetpu_laguna_dev *etdev)
{
	mutex_lock(&etdev->pm_lock);
	if (WARN_ON(!etdev->power_ref) || --etdev->power_ref) {
		mutex_unlock(&etdev->pm_lock);
		return;
	}

	mutex_lock(&etdev->fw_lock);
	edgetpu_laguna_fw_stop(etdev);
	mutex_unlock(&etdev->fw_lock);

	etdev->powered = false;
	pm_runtime_put_sync(etdev->dev);
	mutex_unlock(&etdev->pm_lock);
}
