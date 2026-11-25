// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room arm,smmu-v3 PASID and DMA-BUF mapping layer for Laguna EdgeTPU.
 *
 * Allocates isolated per-client SMMU paging domains (PASID 1..15) backed by
 * 32-bit/36-bit IOVA domains within the device tree gcip-dma-window.
 */

#include <linux/bitmap.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "edgetpu_laguna.h"

static enum dma_data_direction laguna_decode_dir(u32 flags)
{
	switch (flags & EDGETPU_MAP_DIR_MASK) {
	case EDGETPU_MAP_DMA_TO_DEVICE:
		return DMA_TO_DEVICE;
	case EDGETPU_MAP_DMA_FROM_DEVICE:
		return DMA_FROM_DEVICE;
	case EDGETPU_MAP_DMA_NONE:
		return DMA_NONE;
	case EDGETPU_MAP_DMA_BIDIRECTIONAL:
	default:
		return DMA_BIDIRECTIONAL;
	}
}

static int laguna_prot_from_dir(struct device *dev, enum dma_data_direction dir,
				bool mmio, bool ro)
{
	int prot = 0;

	if (ro) {
		prot = IOMMU_READ;
	} else {
		switch (dir) {
		case DMA_TO_DEVICE:
			prot = IOMMU_READ;
			break;
		case DMA_FROM_DEVICE:
			prot = IOMMU_WRITE;
			break;
		case DMA_BIDIRECTIONAL:
			prot = IOMMU_READ | IOMMU_WRITE;
			break;
		case DMA_NONE:
		default:
			prot = 0;
			break;
		}
	}

	if (mmio)
		prot |= IOMMU_MMIO;
	else if (device_iommu_capable(dev, IOMMU_CAP_CACHE_COHERENCY) &&
		 of_property_read_bool(dev->of_node, "dma-coherent"))
		prot |= IOMMU_CACHE;

	return prot;
}

static unsigned long laguna_iova_limit_pfn(struct edgetpu_laguna_dev *etdev,
					   u32 flags)
{
	unsigned long end_pfn = PHYS_PFN(etdev->dma_win_end);
	unsigned long limit32 = PHYS_PFN(DMA_BIT_MASK(32));

	if (flags & EDGETPU_MAP_36BIT_IOVA)
		return end_pfn;

	return min(end_pfn, limit32);
}

static dma_addr_t laguna_alloc_iova(struct edgetpu_laguna_dev *etdev,
				    struct laguna_domain *etdom,
				    size_t aligned_size, u32 flags)
{
	unsigned long nr_pages = aligned_size >> PAGE_SHIFT;
	struct iova *iova;

	if (!nr_pages)
		return 0;

	iova = alloc_iova(&etdom->iovad, nr_pages,
			  laguna_iova_limit_pfn(etdev, flags), true);
	if (!iova)
		return 0;

	return iova_dma_addr(&etdom->iovad, iova);
}

static void laguna_free_iova(struct laguna_domain *etdom, dma_addr_t iova_addr)
{
	free_iova(&etdom->iovad, iova_pfn(&etdom->iovad, iova_addr));
}

int edgetpu_laguna_iommu_map(struct iommu_domain *domain, dma_addr_t iova,
			     phys_addr_t paddr, size_t size, u32 flags)
{
	bool mmio = !!(flags & LAGUNA_IMG_MAP_MMIO);
	bool ro = !!(flags & LAGUNA_IMG_MAP_RO);
	int prot = IOMMU_READ | (ro ? 0 : IOMMU_WRITE) | (mmio ? IOMMU_MMIO : 0);

	if (!domain)
		return -ENODEV;

	return iommu_map(domain, iova, paddr, size, prot, GFP_KERNEL);
}

void edgetpu_laguna_iommu_unmap(struct iommu_domain *domain, dma_addr_t iova,
				size_t size)
{
	if (domain && size)
		iommu_unmap(domain, iova, size);
}

int edgetpu_laguna_iommu_map_sgt(struct iommu_domain *domain, dma_addr_t iova,
				 struct sg_table *sgt, u32 flags)
{
	bool ro = !!(flags & LAGUNA_IMG_MAP_RO);
	int prot = IOMMU_READ | (ro ? 0 : IOMMU_WRITE);
	ssize_t mapped;

	if (!domain)
		return -ENODEV;

	mapped = iommu_map_sgtable(domain, iova, sgt, prot);
	if (mapped < 0)
		return (int)mapped;

	return 0;
}

static void laguna_dma_dev_release(struct device *dev)
{
	kfree(dev);
}

int edgetpu_laguna_iommu_init(struct edgetpu_laguna_dev *etdev)
{
	struct device *dma_dev;
	u32 pasid_bits = 4;
	u32 win[3];
	int ret;

	mutex_init(&etdev->mmu_lock);
	spin_lock_init(&etdev->clients_lock);
	bitmap_zero(etdev->pasid_bitmap, LAGUNA_NUM_PASIDS);
	bitmap_zero(etdev->vcid_bitmap, LAGUNA_NUM_VCIDS);
	/* Reserve PASID 0 for the default KCI / firmware domain. */
	set_bit(0, etdev->pasid_bitmap);

	if (!of_property_read_u32(etdev->dev->of_node, "pasid-num-bits", &pasid_bits) &&
	    pasid_bits > 0 && pasid_bits <= 4)
		etdev->max_pasids = 1U << pasid_bits;
	else
		etdev->max_pasids = LAGUNA_NUM_PASIDS;

	if (!of_property_read_u32_array(etdev->dev->of_node, "gcip-dma-window",
					win, ARRAY_SIZE(win))) {
		u64 size = ((u64)win[1] << 32) | win[2];

		etdev->dma_win_start = win[0];
		etdev->dma_win_end = etdev->dma_win_start + size - 1;
	} else {
		etdev->dma_win_start = LAGUNA_DMA_WIN_DEFAULT_START;
		etdev->dma_win_end = LAGUNA_DMA_WIN_DEFAULT_START +
				     LAGUNA_DMA_WIN_DEFAULT_SIZE - 1;
	}

	etdev->default_domain = iommu_get_domain_for_dev(etdev->dev);
	if (!etdev->default_domain) {
		dev_err(etdev->dev, "no default IOMMU domain found\n");
		return -ENODEV;
	}

	dma_dev = kzalloc_obj(*dma_dev);
	if (!dma_dev)
		return -ENOMEM;

	device_initialize(dma_dev);
	dma_dev->release = laguna_dma_dev_release;
	dev_clear_dma_coherent(dma_dev);
	dma_dev->dma_parms = &etdev->dma_parms;
	ret = dev_set_name(dma_dev, "%s-dma", dev_name(etdev->dev));
	if (ret) {
		put_device(dma_dev);
		return ret;
	}

	ret = dma_coerce_mask_and_coherent(dma_dev, DMA_BIT_MASK(64));
	if (ret) {
		put_device(dma_dev);
		return ret;
	}

	dma_set_max_seg_size(dma_dev, UINT_MAX);
	dma_set_seg_boundary(dma_dev, ULONG_MAX);
	etdev->dma_dev = dma_dev;

	dev_info(etdev->dev, "IOMMU window [%pad..%pad], max_pasids=%u\n",
		 &etdev->dma_win_start, &etdev->dma_win_end, etdev->max_pasids);
	return 0;
}

void edgetpu_laguna_iommu_exit(struct edgetpu_laguna_dev *etdev)
{
	if (etdev->dma_dev) {
		put_device(etdev->dma_dev);
		etdev->dma_dev = NULL;
	}
	etdev->default_domain = NULL;
}

struct laguna_domain *edgetpu_laguna_domain_alloc(struct edgetpu_laguna_dev *etdev)
{
	struct laguna_domain *etdom;
	int pasid, ret;

	etdom = kzalloc_obj(*etdom);
	if (!etdom)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&etdev->mmu_lock);
	pasid = find_next_zero_bit(etdev->pasid_bitmap, etdev->max_pasids, 1);
	if (pasid >= etdev->max_pasids) {
		mutex_unlock(&etdev->mmu_lock);
		kfree(etdom);
		return ERR_PTR(-EBUSY);
	}
	set_bit(pasid, etdev->pasid_bitmap);
	mutex_unlock(&etdev->mmu_lock);

	etdom->domain = iommu_paging_domain_alloc(etdev->dev);
	if (IS_ERR(etdom->domain)) {
		ret = PTR_ERR(etdom->domain);
		goto err_clear_pasid;
	}

	ret = iommu_attach_device_pasid(etdom->domain, etdev->dev, pasid, NULL);
	if (ret)
		goto err_free_domain;

	init_iova_domain(&etdom->iovad, PAGE_SIZE, PHYS_PFN(etdev->dma_win_start));
	etdom->etdev = etdev;
	etdom->pasid = pasid;
	etdom->attached = true;
	return etdom;

err_free_domain:
	iommu_domain_free(etdom->domain);
err_clear_pasid:
	mutex_lock(&etdev->mmu_lock);
	clear_bit(pasid, etdev->pasid_bitmap);
	mutex_unlock(&etdev->mmu_lock);
	kfree(etdom);
	return ERR_PTR(ret);
}

void edgetpu_laguna_domain_free(struct edgetpu_laguna_dev *etdev,
				struct laguna_domain *etdom)
{
	if (!etdom)
		return;

	edgetpu_laguna_fw_shared_context_unmap(etdev, etdom);
	put_iova_domain(&etdom->iovad);
	if (etdom->attached)
		iommu_detach_device_pasid(etdom->domain, etdev->dev, etdom->pasid);
	iommu_domain_free(etdom->domain);

	mutex_lock(&etdev->mmu_lock);
	clear_bit(etdom->pasid, etdev->pasid_bitmap);
	mutex_unlock(&etdev->mmu_lock);

	kfree(etdom);
}

static void laguna_destroy_mapping(struct laguna_domain *etdom,
				   struct laguna_buf_mapping *map)
{
	dma_addr_t aligned_iova = map->iova & PAGE_MASK;

	iommu_unmap(etdom->domain, aligned_iova, map->alloc_size);
	laguna_free_iova(etdom, aligned_iova);

	if (map->is_dmabuf) {
		dma_buf_unmap_attachment_unlocked(map->attach, map->sgt, map->dir);
		dma_buf_detach(map->dmabuf, map->attach);
		dma_buf_put(map->dmabuf);
	} else {
		if (!(map->flags & EDGETPU_MAP_SKIP_CPU_SYNC) &&
		    map->dir != DMA_TO_DEVICE)
			dma_sync_sgtable_for_cpu(etdom->etdev->dev,
						 map->sgt, map->dir);
		sg_free_table(map->sgt);
		kfree(map->sgt);
		unpin_user_pages_dirty_lock(map->pages, map->nr_pages,
					    map->dir != DMA_TO_DEVICE);
		kvfree(map->pages);
	}

	kfree(map);
}

static int laguna_map_dmabuf_sgt(struct edgetpu_laguna_dev *etdev,
				 struct laguna_domain *etdom,
				 dma_addr_t iova, size_t offset,
				 size_t aligned_size, struct sg_table *sgt,
				 int prot)
{
	struct scatterlist *sg;
	size_t skip = offset;
	size_t mapped = 0;
	unsigned int i;
	int ret;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		dma_addr_t dma_addr = sg_dma_address(sg);
		size_t seg_len = sg_dma_len(sg);
		dma_addr_t seg_iova;
		phys_addr_t paddr;
		size_t map_len;

		if (!seg_len)
			continue;
		if (skip >= seg_len) {
			skip -= seg_len;
			continue;
		}

		dma_addr += skip;
		seg_len -= skip;
		skip = 0;

		paddr = dma_to_phys(etdev->dma_dev, dma_addr);
		if (!PAGE_ALIGNED(paddr) ||
		    (seg_len < aligned_size - mapped && !PAGE_ALIGNED(seg_len))) {
			dev_err(etdev->dev,
				"dmabuf seg unaligned: paddr=%pa len=%zu\n",
				&paddr, seg_len);
			ret = -EINVAL;
			goto err_unmap;
		}

		map_len = PAGE_ALIGN(min(seg_len, aligned_size - mapped));
		if (!map_len)
			continue;

		seg_iova = iova + mapped;
		ret = iommu_map(etdom->domain, seg_iova, paddr, map_len, prot,
				GFP_KERNEL);
		if (ret) {
			dev_err(etdev->dev,
				"iommu_map(pasid=%u, iova=%pad, paddr=%pa, len=%zu) failed: %d\n",
				etdom->pasid, &seg_iova, &paddr, map_len, ret);
			goto err_unmap;
		}

		mapped += map_len;
		if (mapped >= aligned_size)
			break;
	}

	if (mapped < aligned_size) {
		dev_err(etdev->dev,
			"dmabuf short sgt: mapped=%zu expected=%zu (nents=%u)\n",
			mapped, aligned_size, sgt->nents);
		ret = -EIO;
		goto err_unmap;
	}

	return 0;

err_unmap:
	if (mapped)
		iommu_unmap(etdom->domain, iova, mapped);
	return ret;
}

int edgetpu_laguna_map_dmabuf(struct laguna_client *client,
			      struct edgetpu_map_dmabuf_ioctl *req)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_domain *etdom = client->etdom;
	struct laguna_buf_mapping *map;
	enum dma_data_direction dir;
	struct dma_buf_attachment *attach;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	dma_addr_t iova;
	size_t map_size, aligned_size;
	int prot, ret;

	if (!etdom)
		return -EINVAL;
	if (!etdev->dma_dev)
		return -ENODEV;
	if (req->offset & ~PAGE_MASK)
		return -EINVAL;

	dir = laguna_decode_dir(req->flags);
	if (dir == DMA_NONE)
		return -EINVAL;

	dmabuf = dma_buf_get(req->dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (req->offset >= dmabuf->size ||
	    req->size > dmabuf->size - req->offset) {
		ret = -EINVAL;
		goto err_put;
	}

	map_size = req->size ? req->size : (dmabuf->size - req->offset);
	aligned_size = PAGE_ALIGN(map_size);
	if (!aligned_size) {
		ret = -EINVAL;
		goto err_put;
	}

	attach = dma_buf_attach(dmabuf, etdev->dma_dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		dev_err(etdev->dev, "dma_buf_attach failed: %d\n", ret);
		goto err_put;
	}

	sgt = dma_buf_map_attachment_unlocked(attach, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(etdev->dev, "dma_buf_map_attachment failed: %d\n", ret);
		goto err_detach;
	}

	iova = laguna_alloc_iova(etdev, etdom, aligned_size, req->flags);
	if (!iova) {
		ret = -ENOSPC;
		dev_err(etdev->dev, "laguna_alloc_iova(%zu) failed\n",
			aligned_size);
		goto err_unmap_attach;
	}

	prot = laguna_prot_from_dir(etdev->dev, dir, false, false);
	ret = laguna_map_dmabuf_sgt(etdev, etdom, iova, req->offset,
				    aligned_size, sgt, prot);
	if (ret)
		goto err_free_iova;

	map = kzalloc_obj(*map);
	if (!map) {
		ret = -ENOMEM;
		goto err_iommu_unmap;
	}

	map->iova = iova;
	map->size = map_size;
	map->alloc_size = aligned_size;
	map->flags = req->flags;
	map->dir = dir;
	map->is_dmabuf = true;
	map->dmabuf = dmabuf;
	map->attach = attach;
	map->sgt = sgt;

	mutex_lock(&client->map_lock);
	list_add_tail(&map->node, &client->mappings);
	mutex_unlock(&client->map_lock);

	req->device_address = iova;
	req->size = map_size;
	return 0;

err_iommu_unmap:
	iommu_unmap(etdom->domain, iova, aligned_size);
err_free_iova:
	laguna_free_iova(etdom, iova);
err_unmap_attach:
	dma_buf_unmap_attachment_unlocked(attach, sgt, dir);
err_detach:
	dma_buf_detach(dmabuf, attach);
err_put:
	dma_buf_put(dmabuf);
	return ret;
}

int edgetpu_laguna_unmap_dmabuf(struct laguna_client *client,
				const struct edgetpu_map_dmabuf_ioctl *req)
{
	struct laguna_buf_mapping *cur, *found = NULL;

	if (!client->etdom)
		return -EINVAL;

	mutex_lock(&client->map_lock);
	list_for_each_entry(cur, &client->mappings, node) {
		if (cur->is_dmabuf && cur->iova == req->device_address) {
			list_del(&cur->node);
			found = cur;
			break;
		}
	}
	mutex_unlock(&client->map_lock);

	if (!found)
		return -EINVAL;

	laguna_destroy_mapping(client->etdom, found);
	return 0;
}

int edgetpu_laguna_map_user(struct laguna_client *client,
			    struct edgetpu_map_ioctl *req)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_domain *etdom = client->etdom;
	struct laguna_buf_mapping *map;
	enum dma_data_direction dir;
	unsigned long uaddr = req->host_address;
	unsigned long page_off = uaddr & ~PAGE_MASK;
	unsigned int nr_pages, gup_flags;
	struct page **pages;
	struct sg_table *sgt;
	dma_addr_t base_iova;
	size_t aligned_size;
	ssize_t mapped;
	int pinned, prot, ret;

	if (!etdom || !req->size)
		return -EINVAL;

	dir = laguna_decode_dir(req->flags);
	if (dir == DMA_NONE)
		return -EINVAL;

	aligned_size = PAGE_ALIGN(page_off + req->size);
	if (!aligned_size || aligned_size < req->size)
		return -EINVAL;

	nr_pages = aligned_size >> PAGE_SHIFT;
	pages = kvcalloc(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	gup_flags = FOLL_LONGTERM | (dir != DMA_TO_DEVICE ? FOLL_WRITE : 0);
	pinned = pin_user_pages_fast(uaddr & PAGE_MASK, nr_pages, gup_flags, pages);
	if (pinned < 0) {
		ret = pinned;
		goto err_free_pages_arr;
	}
	if ((unsigned int)pinned != nr_pages) {
		unpin_user_pages(pages, pinned);
		ret = -EFAULT;
		goto err_free_pages_arr;
	}

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto err_unpin;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, nr_pages, 0,
					(size_t)nr_pages << PAGE_SHIFT, GFP_KERNEL);
	if (ret)
		goto err_free_sgt_struct;

	base_iova = laguna_alloc_iova(etdev, etdom, aligned_size, req->flags);
	if (!base_iova) {
		ret = -ENOSPC;
		goto err_free_sgt;
	}

	if (!(req->flags & EDGETPU_MAP_SKIP_CPU_SYNC))
		dma_sync_sgtable_for_device(etdev->dev, sgt, dir);

	prot = laguna_prot_from_dir(etdev->dev, dir, false, false);
	mapped = iommu_map_sgtable(etdom->domain, base_iova, sgt, prot);
	if (mapped < 0 || (size_t)mapped < aligned_size) {
		if (mapped > 0)
			iommu_unmap(etdom->domain, base_iova, (size_t)mapped);
		ret = mapped < 0 ? (int)mapped : -EIO;
		goto err_free_iova;
	}

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map) {
		ret = -ENOMEM;
		goto err_iommu_unmap;
	}

	map->iova = base_iova + page_off;
	map->size = req->size;
	map->alloc_size = aligned_size;
	map->flags = req->flags;
	map->dir = dir;
	map->is_dmabuf = false;
	map->sgt = sgt;
	map->pages = pages;
	map->nr_pages = nr_pages;

	mutex_lock(&client->map_lock);
	list_add_tail(&map->node, &client->mappings);
	mutex_unlock(&client->map_lock);

	req->device_address = map->iova;
	return 0;

err_iommu_unmap:
	iommu_unmap(etdom->domain, base_iova, aligned_size);
err_free_iova:
	laguna_free_iova(etdom, base_iova);
err_free_sgt:
	sg_free_table(sgt);
err_free_sgt_struct:
	kfree(sgt);
err_unpin:
	unpin_user_pages(pages, nr_pages);
err_free_pages_arr:
	kvfree(pages);
	return ret;
}

int edgetpu_laguna_unmap_user(struct laguna_client *client,
			      const struct edgetpu_map_ioctl *req)
{
	struct laguna_buf_mapping *cur, *found = NULL;

	if (!client->etdom)
		return -EINVAL;

	mutex_lock(&client->map_lock);
	list_for_each_entry(cur, &client->mappings, node) {
		if (!cur->is_dmabuf && cur->iova == req->device_address) {
			list_del(&cur->node);
			found = cur;
			break;
		}
	}
	mutex_unlock(&client->map_lock);

	if (!found)
		return -EINVAL;

	laguna_destroy_mapping(client->etdom, found);
	return 0;
}

int edgetpu_laguna_sync_buffer(struct laguna_client *client,
			       const struct edgetpu_sync_ioctl *req)
{
	struct edgetpu_laguna_dev *etdev = client->etdev;
	struct laguna_buf_mapping *cur;
	bool found = false;
	bool for_cpu = (req->flags & EDGETPU_SYNC_FOR_CPU) != 0;
	struct dma_buf *dmabuf = NULL;
	enum dma_data_direction dir = DMA_BIDIRECTIONAL;

	mutex_lock(&client->map_lock);
	list_for_each_entry(cur, &client->mappings, node) {
		if (req->device_address >= cur->iova &&
		    req->size <= cur->size &&
		    req->device_address - cur->iova <= cur->size - req->size) {
			found = true;
			dir = cur->dir;
			if (cur->is_dmabuf) {
				get_dma_buf(cur->dmabuf);
				dmabuf = cur->dmabuf;
			} else if (for_cpu) {
				dma_sync_sgtable_for_cpu(etdev->dev, cur->sgt,
							 dir);
			} else {
				dma_sync_sgtable_for_device(etdev->dev,
							    cur->sgt, dir);
			}
			break;
		}
	}
	mutex_unlock(&client->map_lock);

	if (!found)
		return -EINVAL;

	if (dmabuf) {
		int ret = for_cpu ? dma_buf_begin_cpu_access(dmabuf, dir)
				  : dma_buf_end_cpu_access(dmabuf, dir);

		dma_buf_put(dmabuf);
		return ret;
	}

	dma_mb();
	return 0;
}

void edgetpu_laguna_unmap_all_client(struct laguna_client *client)
{
	struct laguna_buf_mapping *cur, *tmp;
	LIST_HEAD(to_free);

	if (!client->etdom)
		return;

	mutex_lock(&client->map_lock);
	list_splice_init(&client->mappings, &to_free);
	mutex_unlock(&client->map_lock);

	list_for_each_entry_safe(cur, tmp, &to_free, node) {
		list_del(&cur->node);
		laguna_destroy_mapping(client->etdom, cur);
	}
}
