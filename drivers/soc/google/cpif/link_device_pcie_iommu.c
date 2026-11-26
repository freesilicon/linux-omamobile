// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include "link_device_pcie_iommu.h"

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)

void cpif_pcie_iommu_enable_regions(struct mem_link_device *mld)
{
	/*
	 * Neither gs201 (no SysMMU) nor Laguna (ARM SMMUv3 mapped statically
	 * via setup_iommu_mapping()) uses Exynos dynamic SysMMU regions.
	 */
	(void)mld;
}

int cpif_pcie_iommu_init(struct pktproc_queue *q)
{
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	size_t size = sizeof(void *) * q->num_desc;

	mif_info("iommu init num_desc:%u\n", q->num_desc);

	if (ioc->pf_buf) {
		memset(ioc->pf_buf, 0, size);
	} else {
		ioc->pf_buf = kvzalloc(size, GFP_KERNEL);
		if (!ioc->pf_buf)
			return -ENOMEM;
	}

	return 0;
}

void cpif_pcie_iommu_reset(struct pktproc_queue *q)
{
	struct modem_ctl *mc = dev_get_drvdata(q->ppa->dev);
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	unsigned int usage, idx;

	if (!ioc->pf_buf)
		return;

	usage = circ_get_usage(q->num_desc, ioc->curr_fore, q->done_ptr);
	idx = q->done_ptr;

	mif_info("iommu reset done:%u curr_fore:%u usage:%u, fore:%u\n",
		 q->done_ptr, ioc->curr_fore, usage, *q->fore_ptr);

	while (usage--) {
		unsigned long src_pa;

		src_pa = desc[idx].cp_data_paddr - q->cp_buff_pbase +
			 q->q_buff_pbase - q->ppa->skb_padding_size;
		cpif_pcie_iommu_try_ummap_va(q, src_pa, ioc->pf_buf[idx], idx);

		/* Just free the frags if not mapped yet */
		if (idx == *q->fore_ptr)
			break;

		idx = circ_new_ptr(q->num_desc, idx, 1);
	}

	cpif_iommu_tlb_invalidate_all(mc);

	usage = circ_get_usage(q->num_desc, ioc->curr_fore, q->done_ptr);
	idx = q->done_ptr;

	while (usage--) {
		page_frag_free(ioc->pf_buf[idx]);
		idx = circ_new_ptr(q->num_desc, idx, 1);
	}

	/* Initialize */
	page_frag_cache_drain(&ioc->pf_cache);
	memset(&q->ioc, 0, offsetof(struct cpif_pcie_iommu_ctrl, pf_buf));
}

void *cpif_pcie_iommu_map_va(struct pktproc_queue *q, unsigned long src_pa,
			     u32 idx, u32 *map_cnt)
{
	struct modem_ctl *mc = dev_get_drvdata(q->ppa->dev);
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	const size_t pf_size = q->ppa->true_packet_size;
	void *addr, *page_va;

	addr = page_frag_alloc(&ioc->pf_cache, pf_size, GFP_ATOMIC);
	if (!addr) {
		mif_err_limited("failed to alloc page frag\n");
		return NULL;
	}

	page_va = page_address(virt_to_head_page(addr));

	/* Map the last page */
	*map_cnt = 0;
	if (ioc->map_page_va != page_va) {
		unsigned long map_size, tailroom;
		int ret;

		if (!ioc->map_src_pa)
			goto set_map;

		map_size = page_size(virt_to_page(ioc->map_page_va));
		tailroom = q->q_buff_pbase + q->q_buff_size - ioc->map_src_pa;
		if (map_size > tailroom)
			map_size = tailroom;

		ret = cpif_iommu_map(ioc->map_src_pa, virt_to_phys(ioc->map_page_va),
				     map_size, 0, mc);
		if (ret) {
			mif_err("map failure idx:%u src_pa:0x%lX va:0x%p size:0x%lX\n",
				ioc->map_idx, ioc->map_src_pa, ioc->map_page_va, map_size);
			page_frag_free(addr);
			page_frag_cache_drain(&ioc->pf_cache);
			return NULL;
		}
		ioc->mapped_cnt++;
		ioc->mapped_size += map_size;

		/* Store the last mapping size */
		if (!idx)
			ioc->end_map_size = (u32)map_size;

		*map_cnt = circ_get_usage(q->num_desc, idx, ioc->map_idx);

set_map:
		ioc->map_src_pa = src_pa;
		ioc->map_page_va = page_va;
		ioc->map_idx = idx;
	}

	ioc->pf_buf[idx] = addr;
	ioc->curr_fore = idx;

	if (idx == (q->num_desc - 1))
		page_frag_cache_drain(&ioc->pf_cache);

	return addr;
}

void cpif_pcie_iommu_try_ummap_va(struct pktproc_queue *q, unsigned long src_pa,
				  void *addr, u32 idx)
{
	struct modem_ctl *mc = dev_get_drvdata(q->ppa->dev);
	struct cpif_pcie_iommu_ctrl *ioc = &q->ioc;
	u32 unmap_size;
	size_t ret;

	if (!ioc->unmap_src_pa)
		goto set_unmap;

	unmap_size = !idx ? ioc->end_map_size : ioc->unmap_page_size;

	if (src_pa >= ioc->unmap_src_pa && src_pa < ioc->unmap_src_pa + unmap_size)
		return;

	ret = cpif_iommu_unmap(ioc->unmap_src_pa, unmap_size, mc);
	if (ret != unmap_size)
		mif_err("invalid unmap size:0x%zX expected:0x%X src_pa:0x%lX\n",
			ret, unmap_size, ioc->unmap_src_pa);
	ioc->mapped_cnt--;
	ioc->mapped_size -= unmap_size;

set_unmap:
	ioc->unmap_src_pa = src_pa;
	ioc->unmap_page_size = page_size(virt_to_head_page(addr));
}

#endif /* CONFIG_LINK_DEVICE_PCIE_IOMMU */
