// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe modem control driver for S51xx series
 *
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/version.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/dma-map-ops.h>
#include <linux/exynos-pci-ctrl.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_ctrl.h"
#include "link_device_memory.h"
#include "s51xx_pcie.h"

#define PCIE_ACK_F_ASPM_CONTROL		0x70C
#define PCIE_L1_ENTRANCE_LATENCY	(0x7 << 27)
#define PCIE_L1_ENTRANCE_LATENCY_64us	(0x7 << 27)

static bool cpif_google_pcie;

void cpif_pcie_set_google(bool google)
{
	cpif_google_pcie = google;
}

bool cpif_pcie_is_google(void)
{
	return cpif_google_pcie;
}

static void s51xx_pcie_chk_ep_conf(struct pci_dev *pdev)
{
	int i;
	u32 val1, val2, val3, val4;

	/* EP config. full dump: */
	for (i = 0x0; i < 0x50; i += 0x10) {
		pci_read_config_dword(pdev, i, &val1);
		pci_read_config_dword(pdev, i + 0x4, &val2);
		pci_read_config_dword(pdev, i + 0x8, &val3);
		pci_read_config_dword(pdev, i + 0xC, &val4);
		dev_dbg(&pdev->dev, "0x%02x:  %08x  %08x  %08x  %08x\n",
				i, val1, val2, val3, val4);
	}
}

inline int s51xx_pcie_send_doorbell_int(struct pci_dev *pdev, int int_num)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);
	u32 reg, count = 0;
	int cnt = 0;
	u16 cmd;

	if (s51xx_pcie->link_status == 0) {
		mif_err_limited("Can't send Interrupt(not enabled)!!!\n");
		return -EAGAIN;
	}

	if (pcie_get_cpl_timeout_state(s51xx_pcie->pcie_channel_num)) {
		mif_err_limited("Can't send Interrupt(cto_retry_cnt: %d)!!!\n",
				mc->pcie_cto_retry_cnt);
		return 0;
	}

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err_limited("Can't send Interrupt(not linked)!!!\n");
		goto check_cpl_timeout;
	}

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	if ((((cmd & PCI_COMMAND_MEMORY) == 0) ||
			(cmd & PCI_COMMAND_MASTER) == 0) || (cmd == 0xffff)) {
		mif_err_limited("Can't send Interrupt(not setted bme_en, 0x%04x)!!!\n", cmd);

		do {
			cnt++;

			/* set bme bit */
			pci_set_master(pdev);

			pci_read_config_word(pdev, PCI_COMMAND, &cmd);
			mif_info("cmd reg = 0x%04x\n", cmd);

			/* set mse bit */
			cmd |= PCI_COMMAND_MEMORY;
			pci_write_config_word(pdev, PCI_COMMAND, cmd);

			pci_read_config_word(pdev, PCI_COMMAND, &cmd);
			mif_info("cmd reg = 0x%04x\n", cmd);

			if ((cmd & PCI_COMMAND_MEMORY) &&
					(cmd & PCI_COMMAND_MASTER) && (cmd != 0xffff))
				break;
		} while (cnt < 10);

		if (cnt >= 10) {
			mif_err_limited("BME is not set(cnt=%d)\n", cnt);
			pcie_register_dump(s51xx_pcie->pcie_channel_num);
			goto check_cpl_timeout;
		}
	}

send_doorbell_again:
	if (mc->variant != MODEM_SEC_5400) {
		/*
		 * For gs201, go through the link driver under the lock its
		 * link-down takes, so power-off cannot take the link down
		 * between the check above and the doorbell MMIO access.
		 */
		if (cpif_gs201_pcie_doorbell(s51xx_pcie->pcie_channel_num,
					     s51xx_pcie->doorbell_addr, int_num, &reg)) {
			mif_info_limited("Reserve doorbell interrupt: link went down\n");
			mc->reserve_doorbell_int = true;
			return 0;
		}
	} else {
		iowrite32(int_num, s51xx_pcie->doorbell_addr);
		reg = ioread32(s51xx_pcie->doorbell_addr);
	}

	/* debugging:
	 * mif_info("s51xx_pcie.doorbell_addr = 0x%p -
	 * written(int_num=0x%x) read(reg=0x%x)\n", \
	 *	s51xx_pcie->doorbell_addr, int_num, reg);
	 */

	if (reg == 0xffffffff) {
		count++;
		if (count < 100) {
			if (!in_interrupt())
				udelay(1000); /* 1ms */
			else {
				mif_err_limited("Can't send doorbell in interrupt mode (0x%08X)\n",
						reg);
				return 0;
			}

			goto send_doorbell_again;
		}
		mif_err("[Need to CHECK] Can't send doorbell int (0x%x)\n", reg);
		pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &reg);
		mif_err("Check BAR0 register : %#x\n", reg);
		pcie_register_dump(s51xx_pcie->pcie_channel_num);

		goto check_cpl_timeout;
	}

	return 0;

check_cpl_timeout:
	if (pcie_get_cpl_timeout_state(s51xx_pcie->pcie_channel_num) ||
			pcie_get_sudden_linkdown_state(s51xx_pcie->pcie_channel_num))
		mif_err_limited("Can't send Interrupt(link_down_retry_cnt: %d, cto_retry_cnt: %d)!!!\n",
				mc->pcie_linkdown_retry_cnt, mc->pcie_cto_retry_cnt);
	else
		pcie_force_linkdown_work(s51xx_pcie->pcie_channel_num);

	return 0;
}

void first_save_s51xx_status(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore saving the s5100\n");
		return;
	}

	pci_save_state(pdev);
	s51xx_pcie->first_pci_saved_configs = pci_store_saved_state(pdev);
	if (!s51xx_pcie->first_pci_saved_configs)
		mif_err("MSI-DBG: s51xx pcie.first_pci_saved_configs is NULL(s51xx config NOT saved)\n");
	else
		mif_info("first s51xx config status save: done\n");
}

void s51xx_pcie_save_state(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore save state!!!\n");
		return;
	}

	/* Disable L1.2 before PCIe power off */
	s51xx_pcie_l1ss_ctrl(0, s51xx_pcie);

	pci_clear_master(pdev);

	if (s51xx_pcie->pci_saved_configs)
		kfree(s51xx_pcie->pci_saved_configs);

	pci_save_state(pdev);

	s51xx_pcie->pci_saved_configs = pci_store_saved_state(pdev);

	s51xx_pcie_chk_ep_conf(pdev);

	disable_msi_int(pdev);

	pci_disable_device(pdev);

	pci_wake_from_d3(pdev, false);
	if (pci_set_power_state(pdev, PCI_D3hot))
		mif_err("Can't set D3 state!!!!\n");
}

/*
 * BARs 2-5 on BAR0's bus address, as stock's probe leaves them on gs201.
 */
static void s51xx_pcie_fold_bars(struct pci_dev *pdev, u32 base)
{
	int i;
	u32 val;

	for (i = 2; i <= 5; i++) {
		pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4, &val);
		if ((val & PCI_BASE_ADDRESS_MEM_MASK) != base)
			pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4, base);
	}
}

static void s51xx_pcie_release_bars(struct pci_dev *pdev)
{
	int i;

	for (i = 2; i <= 5; i++) {
		struct resource *res = &pdev->resource[i];

		if (!(res->flags & IORESOURCE_MEM))
			continue;
		if (res->parent)
			pci_release_resource(pdev, i);
		res->start = 0;
		res->end = 0;
		res->flags = 0;
	}
}

void s51xx_pcie_restore_state(struct pci_dev *pdev, bool boot_on,
			      enum modem_variant variant)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);
	int ret;
	u32 val;

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	if (s51xx_check_pcie_link_status(s51xx_pcie->pcie_channel_num) == 0) {
		mif_err("It's not Linked - Ignore restore state!!!\n");
		return;
	}

	if (pci_set_power_state(pdev, PCI_D0))
		mif_err("Can't set D0 state!!!!\n");

	if (!s51xx_pcie->pci_saved_configs &&
	    !s51xx_pcie->first_pci_saved_configs)
		dev_err(&pdev->dev, "[%s] s51xx pcie saved configs is NULL\n", __func__);

	if (boot_on || !s51xx_pcie->pci_saved_configs)
		pci_load_saved_state(pdev, s51xx_pcie->first_pci_saved_configs);
	else
		pci_load_saved_state(pdev, s51xx_pcie->pci_saved_configs);
	pci_restore_state(pdev);

	pci_enable_wake(pdev, PCI_D0, 0);

	ret = pci_enable_device(pdev);
	if (ret)
		mif_err("Can't enable PCIe Device after linkup!\n");

	dev_dbg(&pdev->dev, "[%s] PCIe RC bme bit setting\n", __func__);
	pci_set_master(pdev);

	s51xx_pcie_chk_ep_conf(pdev);

	if (variant != MODEM_SEC_5400) {
		pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val);
		dev_dbg(&pdev->dev, "restored:PCI_BASE_ADDRESS_0 = %#x\n", val);
		if ((val & PCI_BASE_ADDRESS_MEM_MASK) != s51xx_pcie->dbaddr_changed_base) {
			pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0,
						s51xx_pcie->dbaddr_changed_base);
			pci_write_config_dword(pdev, PCI_BASE_ADDRESS_1, 0x0);
			mif_info("write BAR0 value: %#x\n", s51xx_pcie->dbaddr_changed_base);
			s51xx_pcie_chk_ep_conf(pdev);
		}
		s51xx_pcie_fold_bars(pdev, s51xx_pcie->dbaddr_changed_base);
	}

	if ((variant == MODEM_SEC_5400) ? mc->l1ss_disable : boot_on)
		s51xx_pcie_l1ss_ctrl(0, s51xx_pcie);
	else
		s51xx_pcie_l1ss_ctrl(1, s51xx_pcie);

	s51xx_pcie->link_status = 1;
}

int s51xx_check_pcie_link_status(int ch_num)
{
	return pcie_check_link_status(ch_num);
}

void s51xx_pcie_l1ss_ctrl(int enable, struct s51xx_pcie *s51xx_pcie)
{
	int aspm_state = 0;
	u32 val;

	if (!cpif_pcie_is_google()) {
		cpif_gs201_pcie_l1ss_ctrl(enable, PCIE_L1SS_CTRL_MODEM_IF,
					  s51xx_pcie->pcie_channel_num);
		return;
	}

	if (enable) {
		if (s51xx_pcie->l1ss_force) {
			aspm_state = PCIE_LINK_STATE_L1;
			if (s51xx_pcie->l11_enable)
				aspm_state |= PCIE_LINK_STATE_L1_1;
			if (s51xx_pcie->l12_enable)
				aspm_state |= PCIE_LINK_STATE_L1_2;
			dev_dbg(&s51xx_pcie->s51xx_pdev->dev,
				"force l1ss_enable=%#x\n", aspm_state);
		} else {
			aspm_state = PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_CLKPM |
				     PCIE_LINK_STATE_L1_2;
		}
		pci_read_config_dword(s51xx_pcie->s51xx_pdev,
				      PCIE_ACK_F_ASPM_CONTROL, &val);
		val &= ~PCIE_L1_ENTRANCE_LATENCY;
		val |= PCIE_L1_ENTRANCE_LATENCY_64us;
		pci_write_config_dword(s51xx_pcie->s51xx_pdev,
				       PCIE_ACK_F_ASPM_CONTROL, val);
	}

	cpif_google_pcie_l1ss_ctrl(aspm_state, s51xx_pcie->pcie_channel_num);
}

void disable_msi_int(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "[%s]\n", __func__);

	s51xx_pcie->link_status = 0;
	/* It's not needed now...
	 * pci_disable_msi(s51xx_pcie.s51xx_pdev);
	 * pci_config_pm_runtime_put(&s51xx_pcie.s51xx_pdev->dev);
	 */
}

int s51xx_pcie_request_msi_int(struct pci_dev *pdev, int int_num,
				bool use_exclusive_irq)
{
	int err = -EFAULT;
	struct irq_affinity irq_affinity = {
		.pre_vectors = 2,
		.post_vectors = 0,
	};
	struct irq_affinity *irq_affinity_ptr = NULL;
	unsigned int flags = PCI_IRQ_MSI;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (use_exclusive_irq && cpif_pcie_is_google()) {
		irq_affinity_ptr = &irq_affinity;
		flags |= PCI_IRQ_AFFINITY;
	}
#endif

	if (int_num > MAX_MSI_NUM) {
		mif_err("Too many MSI interrupts are requested(<=16)!!!\n");
		return -EFAULT;
	}

	err = pci_alloc_irq_vectors_affinity(pdev, int_num, int_num,
					     flags, irq_affinity_ptr);
	if (err < int_num) {
		mif_err("Can't get msi IRQ!!!!! err %d\n", err);
		return -EFAULT;
	}

	return pdev->irq;
}

#ifdef PIXEL_IOMMU
int setup_iommu_mapping(struct modem_ctl *mc, bool boot_on)
{
	struct iommu_domain *domain;
	int rc, prot;
	u32 size, id, aoc_addr;
	unsigned long long phys_addr;
	int atu_entry = 0;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
#endif

	if (!mc->s51xx_pdev || mc->variant != MODEM_SEC_5400)
		return 0;

	domain = iommu_get_domain_for_dev(&mc->s51xx_pdev->dev);
	if (!domain) {
		mif_err("no IOMMU domain for s51xx_pdev\n");
		return -ENODEV;
	}

	for (id = 0; id < MAX_CP_SHMEM; id++) {
		phys_addr = cp_shmem_get_base(mc->mdm_data->cp_num, id);
		size = cp_shmem_get_size(mc->mdm_data->cp_num, id);
		if (!phys_addr)
			continue;

		/* Skip MSI region after BL2 stage to receive interrupts */
		if (!boot_on && (id == SHMEM_MSI))
			continue;

		/*
		 * Skip GNSS_FW ATU mapping during boot, since we only have
		 * limited ATU entries. GNSS is loaded after modem boot.
		 */
		if (boot_on && (id == SHMEM_GNSS_FW))
			continue;

		/* AOC SRAM is not mapped through IOMMU, we just need an ATU entry */
		if (id == SHMEM_VSS) {
			if (of_property_read_u32(mc->dev->of_node, "pci_aoc_addr", &aoc_addr)) {
				mif_err("CP AoC base address is not defined in dts!\n");
				continue;
			}
			rc = google_pcie_inbound_atu_cfg(mc->pcie_ch_num, aoc_addr,
							 phys_addr, size, atu_entry++);
			if (rc) {
				mif_err("RC ATU mapping failed for %#x -> %#llx (rc: %d)\n",
					aoc_addr, phys_addr, rc);
			}
			continue;
		}

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
		if (pcie_is_sysmmu_enabled(mc->pcie_ch_num) && (id == SHMEM_PKTPROC))
			size = ppa->buff_rgn_offset;
#endif
		if (iommu_iova_to_phys(domain, phys_addr)) {
			mif_info("Region %#llx (sz: %#x) already mapped!\n", phys_addr, size);
			continue;
		}

		if (dev_is_dma_coherent(&mc->s51xx_pdev->dev))
			prot = IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE;
		else
			prot = IOMMU_READ | IOMMU_WRITE;

		rc = iommu_map(domain, phys_addr, phys_addr, size, prot, GFP_KERNEL);
		if (rc) {
			mif_err("iommu_map failed for %#llx (sz: %#x, rc: %d)\n",
				phys_addr, size, rc);
			return rc;
		}
	}

	return 0;
}

int reset_iommu_mapping(struct modem_ctl *mc)
{
	struct iommu_domain *domain;
	u32 size, id;
	unsigned long long phys_addr;
	size_t rc;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
	struct link_device *ld = get_current_link(mc->bootd);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
#endif

	if (!mc->s51xx_pdev || mc->variant != MODEM_SEC_5400)
		return 0;

	domain = iommu_get_domain_for_dev(&mc->s51xx_pdev->dev);
	if (!domain)
		return 0;

	for (id = 0; id < MAX_CP_SHMEM; id++) {
		phys_addr = cp_shmem_get_base(mc->mdm_data->cp_num, id);
		size = cp_shmem_get_size(mc->mdm_data->cp_num, id);

		if (!phys_addr || id == SHMEM_VSS)
			continue;

		if (!iommu_iova_to_phys(domain, phys_addr))
			continue;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
		if (pcie_is_sysmmu_enabled(mc->pcie_ch_num) && (id == SHMEM_PKTPROC))
			size = ppa->buff_rgn_offset;
#endif
		rc = iommu_unmap(domain, phys_addr, size);
		if (rc != size) {
			mif_err("iommu_unmap failed: %#llx, sz:%#x, rc:%#zx\n",
				phys_addr, size, rc);
		}
	}

	return 0;
}
#endif

static void s51xx_pcie_event_cb(pcie_notify_t *noti)
{
	struct pci_dev *pdev = noti->user;
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);
	enum pcie_event event = noti->event;

	mif_err("0x%X pcie event received!\n", event);

	if (event & PCIE_EVENT_LINKDOWN) {
		if (!mc->pcie_powered_on) {
			mif_info("skip cp crash during dislink sequence\n");
			pcie_set_perst_gpio(mc->pcie_ch_num, 0);
			return;
		}

		mif_err("s51xx LINK_DOWN notification callback function!!!\n");
		mif_err("LINK_DOWN: a=%d c=%d\n", mc->pcie_linkdown_retry_cnt_all++,
				mc->pcie_linkdown_retry_cnt);

		if (mc->pcie_linkdown_retry_cnt++ < 10) {
			mif_err("[%d] retry pcie poweron !!!\n", mc->pcie_linkdown_retry_cnt);
			queue_work_on(2, mc->wakeup_wq, &mc->wakeup_work);
		} else {
			mif_err("[%d] force crash !!!\n", mc->pcie_linkdown_retry_cnt);
			pcie_dump_all_status(mc->pcie_ch_num);
			s5100_force_crash_exit_ext(CRASH_REASON_PCIE_LINKDOWN_ERROR);
		}
	} else if (event & PCIE_EVENT_CPL_TIMEOUT) {
		mif_err("s51xx CPL_TIMEOUT notification callback function!!!\n");
		mif_err("CPL: a=%d c=%d\n", mc->pcie_cto_retry_cnt_all++, mc->pcie_cto_retry_cnt);

		if (mc->pcie_cto_retry_cnt++ < 10) {
			mif_err("[%d] retry pcie poweron !!!\n", mc->pcie_cto_retry_cnt);
			queue_work_on(2, mc->wakeup_wq, &mc->wakeup_work);
		} else {
			mif_err("[%d] force crash !!!\n", mc->pcie_cto_retry_cnt);
			pcie_dump_all_status(mc->pcie_ch_num);
			s5100_force_crash_exit_ext(CRASH_REASON_PCIE_CPL_TIMEOUT_ERROR);
		}
	} else if (event & PCIE_EVENT_LINKDOWN_RECOVERY_FAIL) {
		mif_err("Link Down recovery fail force crash !!!\n");
		s5100_force_crash_exit_ext(CRASH_REASON_PCIE_LINKDOWN_RECOVERY_FAILURE);
	}
}

static ssize_t l1ss_force_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", s51xx_pcie->l1ss_force);
}

static ssize_t l1ss_force_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);
	int ret;

	if (!buf)
		return -EINVAL;

	ret = kstrtobool(buf, &s51xx_pcie->l1ss_force);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(l1ss_force);

static ssize_t l11_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", s51xx_pcie->l11_enable);
}

static ssize_t l11_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);
	int ret;

	if (!buf)
		return -EINVAL;

	ret = kstrtobool(buf, &s51xx_pcie->l11_enable);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(l11_enable);

static ssize_t l12_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", s51xx_pcie->l12_enable);
}

static ssize_t l12_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s51xx_pcie *s51xx_pcie = dev_get_drvdata(dev);
	int ret;

	if (!buf)
		return -EINVAL;

	ret = kstrtobool(buf, &s51xx_pcie->l12_enable);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(l12_enable);

static struct attribute *l1ss_attrs[] = {
	&dev_attr_l1ss_force.attr,
	&dev_attr_l11_enable.attr,
	&dev_attr_l12_enable.attr,
	NULL,
};

static const struct attribute_group l1ss_group = {
	.attrs = l1ss_attrs,
	.name = "l1ss",
};

static int s51xx_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
	struct s51xx_pcie *s51xx_pcie;
	struct device *dev = &pdev->dev;
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);
	struct device *mc_dev = mc->dev;
	struct resource *bar0;
	struct pci_bus_region bar0_bus;
	u32 val, db_addr = 0;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	dev_info(dev, "%s EP driver Probe(%s), chNum: %d\n",
			driver->name, __func__, mc->pcie_ch_num);

	s51xx_pcie = devm_kzalloc(dev, sizeof(*s51xx_pcie), GFP_KERNEL);
	if (!s51xx_pcie)
		return -ENOMEM;
	s51xx_pcie->s51xx_pdev = pdev;
	s51xx_pcie->irq_num_base = pdev->irq;
	s51xx_pcie->link_status = 1;
	s51xx_pcie->pcie_channel_num = mc->pcie_ch_num;
	pci_set_drvdata(pdev, s51xx_pcie);

	ret = sysfs_create_group(&pdev->dev.kobj, &l1ss_group);
	if (ret) {
		dev_err(dev, "couldn't create sysfs group for l1ss(%d)\n", ret);
		return ret;
	}

	cpif_pcie_set_google(mc->variant == MODEM_SEC_5400);
	mc->s51xx_pdev = pdev;

	if (mc->variant == MODEM_SEC_5400) {
		ret = setup_iommu_mapping(mc, true);
		if (ret) {
			dev_err(dev, "setup_iommu_mapping failed: %d\n", ret);
			goto err_rm_sysfs;
		}
	}

	bar0 = &pdev->resource[0];
	if (resource_type(bar0) != IORESOURCE_MEM || !resource_assigned(bar0)) {
		dev_err(dev, "BAR0 %pR not assigned: no doorbell\n", bar0);
		ret = -ENODEV;
		goto err_reset_iommu;
	}

	if (of_property_read_u32(mc_dev->of_node, "pci_db_addr", &db_addr))
		dev_info(dev, "EP DB base address is not defined!\n");

	if (mc->variant != MODEM_SEC_5400) {
		if (!db_addr) {
			dev_err(dev, "Failed to parse the EP DB base address\n");
			ret = -EINVAL;
			goto err_reset_iommu;
		}
		s51xx_pcie->dbaddr_offset = db_addr & (resource_size(bar0) - 1);
		if (s51xx_pcie->dbaddr_offset + SZ_4 > resource_size(bar0)) {
			dev_err(dev, "doorbell %#x outside BAR0 %pR\n", db_addr, bar0);
			ret = -ENODEV;
			goto err_reset_iommu;
		}
		pcibios_resource_to_bus(pdev->bus, &bar0_bus, bar0);
		s51xx_pcie->dbaddr_base = bar0->start;
		s51xx_pcie->dbaddr_changed_base = lower_32_bits(bar0_bus.start);
		pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val);
		dev_info(dev, "doorbell: BAR0 %pR (bus %#llx, reads %#x) + %#x; pci_db_addr %#x\n",
			 bar0, (unsigned long long)bar0_bus.start, val,
			 s51xx_pcie->dbaddr_offset, db_addr);

		s51xx_pcie_release_bars(pdev);
		s51xx_pcie_fold_bars(pdev, s51xx_pcie->dbaddr_changed_base);
		dev_info(dev, "BARs 2-5 on BAR0's bus address %#x, as stock\n",
			 s51xx_pcie->dbaddr_changed_base);

		mif_info("Set Doorbell register address.\n");
		s51xx_pcie->doorbell_addr = devm_ioremap(&pdev->dev,
				s51xx_pcie->dbaddr_base + s51xx_pcie->dbaddr_offset, SZ_4);
		mif_info("s51xx_pcie.doorbell_addr = %p  (start 0x%lx offset : %lx)\n",
			s51xx_pcie->doorbell_addr, (unsigned long)s51xx_pcie->dbaddr_base,
						(unsigned long)s51xx_pcie->dbaddr_offset);
	} else {
		resource_size_t db_len = min_t(resource_size_t, resource_size(bar0), SZ_64K);

		s51xx_pcie->dbaddr_base = bar0->start;
		s51xx_pcie->doorbell_addr = devm_ioremap(&pdev->dev,
				s51xx_pcie->dbaddr_base, db_len);
		dev_info(dev, "s51xx_pcie.doorbell_addr = %p (phys %#llx, size %#llx)\n",
			 s51xx_pcie->doorbell_addr,
			 (unsigned long long)s51xx_pcie->dbaddr_base,
			 (unsigned long long)db_len);
	}

	if (!s51xx_pcie->doorbell_addr) {
		mif_err("Can't ioremap doorbell address!!!\n");
		ret = -ENOMEM;
		goto err_reset_iommu;
	}

	mif_info("Register PCIE notification LINKDOWN, CPL_TIMEOUT and LINKDOWN_RECOVERY_FAIL events...\n");
	s51xx_pcie->pcie_event.events =
		PCIE_EVENT_LINKDOWN | PCIE_EVENT_CPL_TIMEOUT | PCIE_EVENT_LINKDOWN_RECOVERY_FAIL;
	s51xx_pcie->pcie_event.user = pdev;
	s51xx_pcie->pcie_event.mode = PCIE_TRIGGER_CALLBACK;
	s51xx_pcie->pcie_event.callback = s51xx_pcie_event_cb;
	pcie_register_event(&s51xx_pcie->pcie_event);

	mif_info("Enable PCI device...\n");
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		mif_err("pci_enable_device() failed, rc:%d\n", ret);
		pcie_deregister_event(&s51xx_pcie->pcie_event);
		goto err_reset_iommu;
	}

	pci_set_master(pdev);

	return 0;

err_reset_iommu:
	if (mc->variant == MODEM_SEC_5400)
		reset_iommu_mapping(mc);
	mc->s51xx_pdev = NULL;
err_rm_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &l1ss_group);
	return ret;
}

void print_msi_register(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	u32 msi_val;

	pci_read_config_dword(pdev, 0x50, &msi_val);
	mif_debug("MSI Control Reg(0x50) : 0x%x\n", msi_val);
	pci_read_config_dword(pdev, 0x54, &msi_val);
	mif_debug("MSI Message Reg(0x54) : 0x%x\n", msi_val);
	pci_read_config_dword(pdev, 0x58, &msi_val);
	mif_debug("MSI MsgData Reg(0x58) : 0x%x\n", msi_val);

	if (msi_val == 0x0) {
		mif_debug("MSI Message Reg == 0x0 - set MSI again!!!\n");

		if (s51xx_pcie->pci_saved_configs != NULL) {
			mif_debug("msi restore\n");
			pci_restore_msi_state(pdev);
		} else {
			mif_debug("[skip] msi restore: saved configs is NULL\n");
		}

		mif_debug("exynos_pcie_msi_init_ext is not implemented\n");
		/* exynos_pcie_msi_init_ext(s51xx_pcie.pcie_channel_num); */

		pci_read_config_dword(pdev, 0x50, &msi_val);
		mif_debug("Recheck - MSI Control Reg : 0x%x (0x50)\n", msi_val);
		pci_read_config_dword(pdev, 0x54, &msi_val);
		mif_debug("Recheck - MSI Message Reg : 0x%x (0x54)\n", msi_val);
		pci_read_config_dword(pdev, 0x58, &msi_val);
		mif_debug("Recheck - MSI MsgData Reg : 0x%x (0x58)\n", msi_val);
	}
}

static void s51xx_pcie_remove(struct pci_dev *pdev)
{
	struct s51xx_pcie *s51xx_pcie = pci_get_drvdata(pdev);
	struct pci_driver *driver = pdev->driver;
	struct modem_ctl *mc = container_of(driver, struct modem_ctl, pci_driver);

	mif_err("s51xx PCIe Remove!!!\n");

	/*
	 * The MSI vectors are the endpoint's: its per-device MSI domain goes
	 * when the driver core releases the device's resources, right after
	 * this, and must be empty by then. Free the handlers on them, then the
	 * vectors. With the link down the PCI core's config writes and the
	 * controller's DBI writes are dropped by the host driver.
	 */
	pcie_deregister_event(&s51xx_pcie->pcie_event);
	s5100_free_pcie_int(mc);
	pci_free_irq_vectors(pdev);

	/* the pci_dev goes with the bus: nothing may use it after this */
	mc->s51xx_pdev = NULL;

	kfree(s51xx_pcie->first_pci_saved_configs);
	kfree(s51xx_pcie->pci_saved_configs);
	sysfs_remove_group(&pdev->dev.kobj, &l1ss_group);
}

/*
 * Match Samsung modem endpoints across gs201 (0xa5a5) and Laguna S5400i;
 * s51xx_pcie_probe() rejects PCIe Root Port bridges (such as 144d:ecec).
 */
static struct pci_device_id s51xx_pci_id_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_SAMSUNG, PCI_ANY_ID) },
	{ }
};

MODULE_DEVICE_TABLE(pci, s51xx_pci_id_tbl);

static struct pci_driver s51xx_driver = {
	.name = "s51xx",
	.id_table = s51xx_pci_id_tbl,
	.probe = s51xx_pcie_probe,
	.remove = s51xx_pcie_remove,
};

/*
 * Initialize PCIe s51xx EP driver.
 */
int s51xx_pcie_init(struct modem_ctl *mc)
{
	int ret;
	int ch_num = mc->pcie_ch_num;

	mif_info("Register PCIE drvier for s51xx.(chNum: %d, mc: 0x%p)\n", ch_num, mc);

	mc->pci_driver = s51xx_driver;

	ret = pci_register_driver(&mc->pci_driver);
	if (ret < 0) {
		mif_err("pci_register_driver() failed, rc:%d\n", ret);
		return ret;
	}

	/*
	 * The endpoint probes from pci_register_driver(). If it did not bind
	 * (no endpoint, or the probe failed), take the driver away again:
	 * register_pcie() calls this on every power-on until the endpoint
	 * has bound, and a second registration of the same driver fails
	 * with -EBUSY and never probes.
	 */
	if (!mc->s51xx_pdev) {
		mif_err("no s51xx endpoint bound; driver unregistered\n");
		pci_unregister_driver(&mc->pci_driver);
		return -ENODEV;
	}

	return 0;
}
