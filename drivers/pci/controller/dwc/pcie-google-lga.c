// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for Google Laguna (Tensor G5) SoCs.
 *
 * Clean-room DesignWare PCIe Root Complex glue managing CPM clocks/resets,
 * HSIO power domains, PHY SRAM initialization handshake, endpoint pwrctrl,
 * on-demand endpoint link lifecycle, inbound ATU windows, and LTSSM bring-up.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci-pwrctrl.h>
#include <linux/pci_regs.h>
#define __PCIE_GOOGLE_LGA_IMPL
#include <linux/pcie_google_if.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/google/clk-cpm.h>

#include "../../pci.h"
#include "pcie-designware.h"

#define LGA_PCIE_SII_RST_STS		0x1c
#define LGA_PCIE_SII_SMLH_REQ_RST_NOT	BIT(1)
#define LGA_PCIE_SII_PM_MSG		0x24
#define LGA_PCIE_SII_PME_TURNOFF	BIT(0)
#define LGA_PCIE_SII_PME_TO_ACK		BIT(7)
#define LGA_PCIE_SII_PM_STATE		0x28
#define LGA_PCIE_SII_LTSSM_MASK		GENMASK(8, 3)
#define LGA_PCIE_SII_IN_L2		BIT(14)
#define LGA_PCIE_SII_PWR_CTRL		0x2c
#define LGA_PCIE_SII_L1_PWRGATE		BIT(11)

#define LGA_PCIE_TOP_CTRL		0x00
#define LGA_PCIE_TOP_PHY_RST_DONE	BIT(0)
#define LGA_PCIE_TOP_EXT_LD_DONE	BIT(23)

#define LGA_PCIE_TOP_STATUS		0x10
#define LGA_PCIE_TOP_PHY_INIT_DONE	BIT(2)

#define LGA_PCIE_TOP_LTSSM_CTRL		0x14
#define LGA_PCIE_TOP_LTSSM_EN		BIT(0)

#define LGA_PCIE_TOP_LINK_STS		0x18
#define LGA_PCIE_TOP_LINK_UP_MASK	GENMASK(7, 6)

#define LGA_PCIE_TOP_AXI_CTRL		0x24
#define LGA_PCIE_TOP_AXI_MASK		GENMASK(5, 0)
#define LGA_PCIE_TOP_AXI_VAL		0x9

#define LGA_PCIE_TOP_RESET_STATUS	0x38
#define LGA_PCIE_TOP_APP_INIT_RST	BIT(27)

#define LGA_PCIE_DBI_RX_Q_CTRL		0x748
#define LGA_PCIE_DBI_RX_Q_CREDITS	GENMASK(11, 0)
#define LGA_PCIE_DBI_LINK_TIMEOUT	0x8d4

#define LGA_PCIE_L1SS_MASK		0xff0400ffU
#define LGA_PCIE_L1SS_VAL		0x00513c00U

#define LGA_PCIE_PHY_INIT_POLL_US	100
#define LGA_PCIE_PHY_INIT_TIMEOUT_US	USEC_PER_SEC
#define LGA_PCIE_DEFAULT_PERST_US	20000
#define LGA_PCIE_LINK_RETRIES		10
#define LGA_PCIE_LINK_WAIT_RETRIES	4800
#define LGA_PCIE_LINK_WAIT_US		10
#define LGA_PCIE_RETRAIN_TIMEOUT_US	USEC_PER_SEC
#define LGA_PCIE_SPEED_WAIT_US		10
#define LGA_PCIE_SPEED_WAIT_RETRIES	10000
#define LGA_PCIE_L2_ACK_POLL_US		10
#define LGA_PCIE_L2_ACK_TIMEOUT_US	20000
#define LGA_PCIE_L2_STATE_POLL_US	10
#define LGA_PCIE_L2_STATE_TIMEOUT_US	USEC_PER_SEC
#define LGA_PCIE_L0_STATE_TIMEOUT_US	100000

enum lga_pcie_link_state {
	LGA_LINK_L0 = 0,
	LGA_LINK_RECOVERY = 1,
	LGA_LINK_L0S = 2,
	LGA_LINK_L1 = 3,
	LGA_LINK_L11 = 4,
	LGA_LINK_L12 = 5,
	LGA_LINK_L2 = 6,
	LGA_LINK_UNKNOWN = 7,
};

enum lga_pcie_recovery_flags {
	LGA_REC_CPL_TIMEOUT = 0,
	LGA_REC_LINK_DOWN,
};

struct google_lga_pcie {
	struct dw_pcie pci;
	struct list_head node;
	void __iomem *sii;
	void __iomem *top;
	struct clk *phy_fw_clk;
	struct clk *aux_clk;
	struct reset_control *rst_init;
	struct reset_control *rst_perst;
	struct reset_control *rst_app_hold;
	struct reset_control *rst_power_up;
	struct gpio_desc *perstn_gpio;
	struct mutex link_lock; /* serializes RC poweron/poweroff/speed transitions */
	spinlock_t state_lock; /* guards powered_on, is_link_up, recovery_flags */
	raw_spinlock_t cfg_lock; /* serializes on-demand config access with powerdown */
	struct pci_ops *orig_root_ops;
	struct pci_ops *orig_child_ops;
	struct pci_saved_state *saved_state;
	google_pcie_callback_func cb_func;
	void *cb_priv;
	int domain;
	u8 max_link_speed;
	u8 max_link_width;
	u8 target_link_speed;
	u8 current_link_speed;
	u8 current_link_width;
	u32 perst_delay_us;
	u32 posted_rx_q_credits;
	u32 link_timeout_ms;
	bool has_link_timeout;
	bool has_phy_sram;
	bool skip_link_eq;
	bool l1_pwrgate_disable;
	bool allow_suspend_in_linkup;
	bool on_demand;
	bool poweron_active;
	bool enumeration_done;
	bool auto_gate_enabled;
	bool clocks_enabled;
	bool powered_on;
	bool cfg_powered;
	bool is_link_up;
	bool msi_irqs_enabled;
	bool cpl_irq_enabled;
	bool link_down_irq_enabled;
	bool pwrctrl_created;
	unsigned long recovery_flags;
	int cpl_timeout_irq;
	int link_down_irq;
	atomic_t cpl_timeout_cnt;
	atomic_t link_down_cnt;
};

#define to_google_lga_pcie(x) container_of((x), struct google_lga_pcie, pci)

static LIST_HEAD(google_lga_pcie_list);
static DEFINE_SPINLOCK(google_lga_pcie_list_lock);

static struct google_lga_pcie *google_lga_pcie_find(int domain)
{
	struct google_lga_pcie *lga;
	unsigned long flags;

	spin_lock_irqsave(&google_lga_pcie_list_lock, flags);
	list_for_each_entry(lga, &google_lga_pcie_list, node) {
		if (lga->domain == domain) {
			spin_unlock_irqrestore(&google_lga_pcie_list_lock, flags);
			return lga;
		}
	}
	spin_unlock_irqrestore(&google_lga_pcie_list_lock, flags);

	return NULL;
}

static bool google_lga_pcie_in_recovery(const struct google_lga_pcie *lga)
{
	return test_bit(LGA_REC_CPL_TIMEOUT, &lga->recovery_flags) ||
	       test_bit(LGA_REC_LINK_DOWN, &lga->recovery_flags);
}

static u32 google_lga_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
				    u32 reg, size_t size)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	unsigned long flags;
	u32 val = ~0U;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (lga->powered_on)
		dw_pcie_read(base + reg, size, &val);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	return val;
}

static void google_lga_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				      u32 reg, size_t size, u32 val)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	unsigned long flags;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (lga->powered_on)
		dw_pcie_write(base + reg, size, val);
	spin_unlock_irqrestore(&lga->state_lock, flags);
}

static enum lga_pcie_link_state
google_lga_pcie_get_link_state(struct google_lga_pcie *lga)
{
	unsigned long flags;
	u32 pm_state;
	u32 ltssm;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->powered_on) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return LGA_LINK_L2;
	}
	pm_state = readl(lga->sii + LGA_PCIE_SII_PM_STATE);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	if (pm_state & LGA_PCIE_SII_IN_L2)
		return LGA_LINK_L2;

	ltssm = FIELD_GET(LGA_PCIE_SII_LTSSM_MASK, pm_state);
	if (ltssm == DW_PCIE_LTSSM_L0)
		return LGA_LINK_L0;
	if (ltssm >= DW_PCIE_LTSSM_CFG_LINKWD_START &&
	    ltssm <= DW_PCIE_LTSSM_CFG_IDLE)
		return LGA_LINK_RECOVERY;
	if (ltssm >= DW_PCIE_LTSSM_RCVRY_LOCK &&
	    ltssm <= DW_PCIE_LTSSM_RCVRY_EQ3)
		return LGA_LINK_RECOVERY;
	if (ltssm == DW_PCIE_LTSSM_L0S)
		return LGA_LINK_L0S;
	if (ltssm == DW_PCIE_LTSSM_L1_IDLE ||
	    ltssm == DW_PCIE_LTSSM_L123_SEND_EIDLE)
		return LGA_LINK_L1;
	if (ltssm == DW_PCIE_LTSSM_L2_IDLE || ltssm == DW_PCIE_LTSSM_L2_WAKE)
		return LGA_LINK_L2;

	return LGA_LINK_UNKNOWN;
}

static enum dw_pcie_ltssm google_lga_pcie_get_ltssm(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	unsigned long flags;
	u32 pm_state;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->powered_on) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return DW_PCIE_LTSSM_DETECT_QUIET;
	}
	pm_state = readl(lga->sii + LGA_PCIE_SII_PM_STATE);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	return FIELD_GET(LGA_PCIE_SII_LTSSM_MASK, pm_state);
}

static void google_lga_pcie_set_auto_gate(struct google_lga_pcie *lga, bool enable)
{
	int ret;

	if (!IS_ENABLED(CONFIG_GOOGLE_CPM_CLK) || !lga->phy_fw_clk)
		return;
	if (lga->auto_gate_enabled == enable)
		return;

	ret = goog_cpm_clk_toggle_auto_gate(lga->phy_fw_clk, enable);
	if (ret) {
		dev_warn(lga->pci.dev, "CPM auto-gate(%d) failed: %d\n",
			 enable, ret);
		return;
	}

	lga->auto_gate_enabled = enable;
}

static void google_lga_pcie_set_msi_irqs(struct google_lga_pcie *lga, bool enable)
{
	struct dw_pcie_rp *pp = &lga->pci.pp;
	unsigned int num_ctrls;
	unsigned int i;

	if (!lga->on_demand || lga->msi_irqs_enabled == enable)
		return;

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;
	for (i = 0; i < num_ctrls && i < MAX_MSI_CTRLS; i++) {
		if (pp->msi_irq[i] <= 0)
			continue;
		if (enable)
			enable_irq(pp->msi_irq[i]);
		else
			disable_irq(pp->msi_irq[i]);
	}
	lga->msi_irqs_enabled = enable;
}

static void google_lga_pcie_set_err_irqs(struct google_lga_pcie *lga, bool enable)
{
	if (enable) {
		if (lga->cpl_timeout_irq > 0 && !lga->cpl_irq_enabled) {
			irq_set_irqchip_state(lga->cpl_timeout_irq,
					      IRQCHIP_STATE_PENDING, false);
			enable_irq(lga->cpl_timeout_irq);
			lga->cpl_irq_enabled = true;
		}
		if (lga->link_down_irq > 0 && !lga->link_down_irq_enabled) {
			irq_set_irqchip_state(lga->link_down_irq,
					      IRQCHIP_STATE_PENDING, false);
			enable_irq(lga->link_down_irq);
			lga->link_down_irq_enabled = true;
		}
		return;
	}

	if (lga->cpl_timeout_irq > 0 && lga->cpl_irq_enabled) {
		disable_irq(lga->cpl_timeout_irq);
		lga->cpl_irq_enabled = false;
	}
	if (lga->link_down_irq > 0 && lga->link_down_irq_enabled) {
		disable_irq(lga->link_down_irq);
		lga->link_down_irq_enabled = false;
	}
}

static void google_lga_pcie_assert_resets(struct google_lga_pcie *lga)
{
	reset_control_assert(lga->rst_app_hold);
	reset_control_assert(lga->rst_perst);
	reset_control_assert(lga->rst_power_up);
	reset_control_assert(lga->rst_init);
}

static void google_lga_pcie_disable_clocks(struct google_lga_pcie *lga)
{
	if (!lga->clocks_enabled)
		return;

	clk_disable_unprepare(lga->phy_fw_clk);
	clk_disable_unprepare(lga->aux_clk);
	lga->clocks_enabled = false;
}

static void google_lga_pcie_configure_dbi(struct google_lga_pcie *lga)
{
	struct dw_pcie *pci = &lga->pci;
	u16 l1ss;
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);

	if (lga->has_link_timeout)
		dw_pcie_writel_dbi(pci, LGA_PCIE_DBI_LINK_TIMEOUT,
				   lga->link_timeout_ms);

	l1ss = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_L1SS);
	if (l1ss) {
		val = dw_pcie_readl_dbi(pci, l1ss + PCI_L1SS_CTL1);
		val = (val & LGA_PCIE_L1SS_MASK) | LGA_PCIE_L1SS_VAL;
		dw_pcie_writel_dbi(pci, l1ss + PCI_L1SS_CTL1, val);
	}

	if (lga->skip_link_eq) {
		val = dw_pcie_readl_dbi(pci, GEN3_RELATED_OFF);
		val |= GEN3_RELATED_OFF_GEN3_EQ_DISABLE;
		dw_pcie_writel_dbi(pci, GEN3_RELATED_OFF, val);
	}

	if (lga->posted_rx_q_credits) {
		val = dw_pcie_readl_dbi(pci, LGA_PCIE_DBI_RX_Q_CTRL);
		val &= ~LGA_PCIE_DBI_RX_Q_CREDITS;
		val |= FIELD_PREP(LGA_PCIE_DBI_RX_Q_CREDITS,
				  lga->posted_rx_q_credits);
		dw_pcie_writel_dbi(pci, LGA_PCIE_DBI_RX_Q_CTRL, val);
	}

	dw_pcie_dbi_ro_wr_dis(pci);
}

static int google_lga_pcie_power_up_hw(struct google_lga_pcie *lga)
{
	struct device *dev = lga->pci.dev;
	unsigned long flags;
	u32 val;
	int ret;

	ret = clk_prepare_enable(lga->aux_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(lga->phy_fw_clk);
	if (ret) {
		clk_disable_unprepare(lga->aux_clk);
		return ret;
	}
	lga->clocks_enabled = true;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		goto err_disable_clks;

	google_lga_pcie_assert_resets(lga);

	ret = reset_control_deassert(lga->rst_init);
	if (ret)
		goto err_assert_resets;

	ret = reset_control_deassert(lga->rst_power_up);
	if (ret)
		goto err_assert_resets;

	ret = reset_control_deassert(lga->rst_perst);
	if (ret)
		goto err_assert_resets;

	if (lga->has_phy_sram) {
		ret = readl_poll_timeout(lga->top + LGA_PCIE_TOP_STATUS, val,
					 val & LGA_PCIE_TOP_PHY_INIT_DONE,
					 LGA_PCIE_PHY_INIT_POLL_US,
					 LGA_PCIE_PHY_INIT_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "PHY SRAM init timeout (status=%#x)\n", val);
			goto err_assert_resets;
		}

		val = readl(lga->top + LGA_PCIE_TOP_CTRL);
		val |= LGA_PCIE_TOP_EXT_LD_DONE;
		writel(val, lga->top + LGA_PCIE_TOP_CTRL);
	}

	val = readl(lga->top + LGA_PCIE_TOP_CTRL);
	val |= LGA_PCIE_TOP_PHY_RST_DONE;
	writel(val, lga->top + LGA_PCIE_TOP_CTRL);

	ret = reset_control_deassert(lga->rst_app_hold);
	if (ret)
		goto err_assert_resets;

	val = readl(lga->top + LGA_PCIE_TOP_AXI_CTRL);
	val = (val & ~LGA_PCIE_TOP_AXI_MASK) | LGA_PCIE_TOP_AXI_VAL;
	writel(val, lga->top + LGA_PCIE_TOP_AXI_CTRL);

	if (lga->l1_pwrgate_disable) {
		val = readl(lga->sii + LGA_PCIE_SII_PWR_CTRL);
		val &= ~LGA_PCIE_SII_L1_PWRGATE;
		writel(val, lga->sii + LGA_PCIE_SII_PWR_CTRL);
	}

	spin_lock_irqsave(&lga->state_lock, flags);
	lga->powered_on = true;
	spin_unlock_irqrestore(&lga->state_lock, flags);

	raw_spin_lock_irqsave(&lga->cfg_lock, flags);
	WRITE_ONCE(lga->cfg_powered, true);
	raw_spin_unlock_irqrestore(&lga->cfg_lock, flags);

	google_lga_pcie_configure_dbi(lga);
	return 0;

err_assert_resets:
	google_lga_pcie_assert_resets(lga);
	pinctrl_pm_select_idle_state(dev);
err_disable_clks:
	google_lga_pcie_disable_clocks(lga);
	return ret;
}

static struct google_lga_pcie *google_lga_pcie_from_bus(struct pci_bus *bus)
{
	struct dw_pcie_rp *pp = bus->sysdata;

	return to_google_lga_pcie(to_dw_pcie_from_pp(pp));
}

static struct pci_ops *google_lga_pcie_orig_ops(struct google_lga_pcie *lga,
						struct pci_bus *bus)
{
	return pci_is_root_bus(bus) ? lga->orig_root_ops : lga->orig_child_ops;
}

static void __iomem *google_lga_pcie_pci_map_bus(struct pci_bus *bus,
						 unsigned int devfn, int where)
{
	struct google_lga_pcie *lga = google_lga_pcie_from_bus(bus);
	struct pci_ops *orig_ops = google_lga_pcie_orig_ops(lga, bus);

	if (!READ_ONCE(lga->cfg_powered) || !orig_ops || !orig_ops->map_bus)
		return NULL;

	return orig_ops->map_bus(bus, devfn, where);
}

static int google_lga_pcie_pci_cfg_access(struct pci_bus *bus,
					  unsigned int devfn, int where,
					  int size, u32 *val, bool write)
{
	struct google_lga_pcie *lga = google_lga_pcie_from_bus(bus);
	struct pci_ops *orig_ops = google_lga_pcie_orig_ops(lga, bus);
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&lga->cfg_lock, flags);
	if (!lga->cfg_powered || !orig_ops ||
	    (write ? !orig_ops->write : !orig_ops->read)) {
		raw_spin_unlock_irqrestore(&lga->cfg_lock, flags);
		if (!write)
			*val = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	ret = write ? orig_ops->write(bus, devfn, where, size, *val) :
		      orig_ops->read(bus, devfn, where, size, val);
	raw_spin_unlock_irqrestore(&lga->cfg_lock, flags);

	return ret;
}

static int google_lga_pcie_pci_cfg_read(struct pci_bus *bus, unsigned int devfn,
					int where, int size, u32 *val)
{
	return google_lga_pcie_pci_cfg_access(bus, devfn, where, size, val,
						false);
}

static int google_lga_pcie_pci_cfg_write(struct pci_bus *bus,
					 unsigned int devfn, int where,
					 int size, u32 val)
{
	return google_lga_pcie_pci_cfg_access(bus, devfn, where, size, &val,
						true);
}

static struct pci_ops google_lga_pcie_pci_ops = {
	.map_bus = google_lga_pcie_pci_map_bus,
	.read = google_lga_pcie_pci_cfg_read,
	.write = google_lga_pcie_pci_cfg_write,
};

static void google_lga_pcie_override_child_bus_ops(struct pci_bus *bus)
{
	struct pci_bus *child;

	list_for_each_entry(child, &bus->children, node) {
		pci_bus_set_ops(child, &google_lga_pcie_pci_ops);
		google_lga_pcie_override_child_bus_ops(child);
	}
}

static int google_lga_pcie_setup_rc(struct google_lga_pcie *lga)
{
	struct dw_pcie_rp *pp = &lga->pci.pp;
	int ret;

	if (!lga->orig_child_ops)
		return dw_pcie_setup_rc(pp);

	/*
	 * dw_pcie_setup_rc() skips dw_pcie_iatu_setup() unless child_ops is
	 * dw_child_pcie_ops; temporarily expose orig_child_ops under
	 * pci_rescan_remove_lock so ATU windows are programmed on power-on.
	 */
	pp->bridge->child_ops = lga->orig_child_ops;
	ret = dw_pcie_setup_rc(pp);
	pp->bridge->child_ops = &google_lga_pcie_pci_ops;

	return ret;
}

static void google_lga_pcie_power_down_hw(struct google_lga_pcie *lga)
{
	struct device *dev = lga->pci.dev;
	unsigned long flags;

	raw_spin_lock_irqsave(&lga->cfg_lock, flags);
	WRITE_ONCE(lga->cfg_powered, false);
	raw_spin_unlock_irqrestore(&lga->cfg_lock, flags);

	spin_lock_irqsave(&lga->state_lock, flags);
	lga->powered_on = false;
	spin_unlock_irqrestore(&lga->state_lock, flags);

	google_lga_pcie_set_auto_gate(lga, false);

	if (lga->perstn_gpio)
		gpiod_set_value_cansleep(lga->perstn_gpio, 1);

	pinctrl_pm_select_idle_state(dev);
	google_lga_pcie_assert_resets(lga);
	google_lga_pcie_disable_clocks(lga);
}

static irqreturn_t google_lga_pcie_cpl_timeout_isr(int irq, void *data)
{
	struct google_lga_pcie *lga = data;
	unsigned long flags;

	atomic_inc(&lga->cpl_timeout_cnt);

	if (!lga->on_demand) {
		dev_warn_ratelimited(lga->pci.dev,
				     "PCIe completion timeout event (%d)\n",
				     atomic_read(&lga->cpl_timeout_cnt));
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->is_link_up) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return IRQ_NONE;
	}
	if (google_lga_pcie_in_recovery(lga)) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return IRQ_HANDLED;
	}

	lga->is_link_up = false;
	set_bit(LGA_REC_CPL_TIMEOUT, &lga->recovery_flags);
	if (lga->link_down_irq > 0)
		disable_irq_nosync(lga->link_down_irq);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t google_lga_pcie_cpl_timeout_thread(int irq, void *data)
{
	struct google_lga_pcie *lga = data;

	dev_warn(lga->pci.dev, "PCIe completion timeout event (%d)\n",
		 atomic_read(&lga->cpl_timeout_cnt));
	if (lga->cb_func)
		lga->cb_func(GPCIE_CB_CPL_TIMEOUT, lga->cb_priv);

	return IRQ_HANDLED;
}

static irqreturn_t google_lga_pcie_link_down_isr(int irq, void *data)
{
	struct google_lga_pcie *lga = data;
	unsigned long flags;
	u32 val;

	atomic_inc(&lga->link_down_cnt);

	if (!lga->on_demand) {
		dev_warn_ratelimited(lga->pci.dev,
				     "PCIe link down event (%d)\n",
				     atomic_read(&lga->link_down_cnt));
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->is_link_up || !lga->powered_on) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return IRQ_NONE;
	}

	val = readl(lga->sii + LGA_PCIE_SII_RST_STS);
	if (val & LGA_PCIE_SII_SMLH_REQ_RST_NOT) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return IRQ_NONE;
	}

	if (google_lga_pcie_in_recovery(lga)) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return IRQ_HANDLED;
	}

	lga->is_link_up = false;
	set_bit(LGA_REC_LINK_DOWN, &lga->recovery_flags);
	disable_irq_nosync(lga->link_down_irq);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t google_lga_pcie_link_down_thread(int irq, void *data)
{
	struct google_lga_pcie *lga = data;

	dev_warn(lga->pci.dev, "PCIe link down event (%d)\n",
		 atomic_read(&lga->link_down_cnt));
	if (lga->cb_func)
		lga->cb_func(GPCIE_CB_LINK_DOWN, lga->cb_priv);

	return IRQ_HANDLED;
}

static int google_lga_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	struct device *dev = pci->dev;
	int ret;

	/* Hold PERST# active (low) while bringing up supplies, clocks, and PHY. */
	if (lga->perstn_gpio)
		gpiod_set_value_cansleep(lga->perstn_gpio, 1);

	if (!lga->on_demand) {
		ret = pci_pwrctrl_create_devices(dev);
		if (ret)
			return ret;
		lga->pwrctrl_created = true;

		if (!pp->skip_pwrctrl_off) {
			ret = pci_pwrctrl_power_on_devices(dev);
			if (ret)
				goto err_pwrctrl_destroy;
		}
	}

	ret = google_lga_pcie_power_up_hw(lga);
	if (ret)
		goto err_pwrctrl_off;

	if (lga->on_demand && !pp->ecam_enabled) {
		lga->orig_root_ops = pp->bridge->ops;
		lga->orig_child_ops = pp->bridge->child_ops;
		pp->bridge->ops = &google_lga_pcie_pci_ops;
	}

	return 0;

err_pwrctrl_off:
	if (lga->pwrctrl_created && !pp->skip_pwrctrl_off)
		pci_pwrctrl_power_off_devices(dev);
err_pwrctrl_destroy:
	if (lga->pwrctrl_created && ret != -EPROBE_DEFER) {
		pci_pwrctrl_destroy_devices(dev);
		lga->pwrctrl_created = false;
	}
	return ret;
}

static void google_lga_pcie_host_post_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);

	if (lga->on_demand) {
		if (!pp->ecam_enabled && lga->orig_child_ops) {
			pci_lock_rescan_remove();
			pp->bridge->child_ops = &google_lga_pcie_pci_ops;
			if (pp->bridge->bus)
				google_lga_pcie_override_child_bus_ops(pp->bridge->bus);
			pci_unlock_rescan_remove();
		}
		lga->msi_irqs_enabled = true;
		return;
	}

	google_lga_pcie_set_auto_gate(lga, true);
	google_lga_pcie_set_err_irqs(lga, true);
}

static void google_lga_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	struct device *dev = pci->dev;

	google_lga_pcie_set_err_irqs(lga, false);
	google_lga_pcie_set_msi_irqs(lga, false);
	if (lga->powered_on)
		google_lga_pcie_power_down_hw(lga);

	if (lga->pwrctrl_created) {
		if (!pp->skip_pwrctrl_off)
			pci_pwrctrl_power_off_devices(dev);
		pci_pwrctrl_destroy_devices(dev);
		lga->pwrctrl_created = false;
	}
}

static bool google_lga_pcie_link_up(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	unsigned long flags;
	u32 top_sts;
	u32 dbg1;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->powered_on) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return false;
	}
	top_sts = readl(lga->top + LGA_PCIE_TOP_LINK_STS);
	dbg1 = readl(pci->dbi_base + PCIE_PORT_DEBUG1);
	spin_unlock_irqrestore(&lga->state_lock, flags);

	if ((top_sts & LGA_PCIE_TOP_LINK_UP_MASK) != LGA_PCIE_TOP_LINK_UP_MASK)
		return false;

	return !(dbg1 & PCIE_PORT_DEBUG1_LINK_IN_TRAINING);
}

static int google_lga_pcie_start_link(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	u32 val;

	if (lga->on_demand && !lga->poweron_active)
		return 0;

	if (lga->perstn_gpio) {
		gpiod_set_value_cansleep(lga->perstn_gpio, 0);
		usleep_range(lga->perst_delay_us, lga->perst_delay_us + 2000);
	}

	val = readl(lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
	val |= LGA_PCIE_TOP_LTSSM_EN;
	writel(val, lga->top + LGA_PCIE_TOP_LTSSM_CTRL);

	return 0;
}

static void google_lga_pcie_stop_link(struct dw_pcie *pci)
{
	struct google_lga_pcie *lga = to_google_lga_pcie(pci);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (!lga->powered_on) {
		spin_unlock_irqrestore(&lga->state_lock, flags);
		return;
	}
	val = readl(lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
	val &= ~LGA_PCIE_TOP_LTSSM_EN;
	writel(val, lga->top + LGA_PCIE_TOP_LTSSM_CTRL);
	spin_unlock_irqrestore(&lga->state_lock, flags);
}

static void google_lga_pcie_fetch_speed_width(struct google_lga_pcie *lga)
{
	struct dw_pcie *pci = &lga->pci;
	u8 cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u16 lnksta = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKSTA);

	lga->current_link_speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta);
	lga->current_link_width = FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta);
}

static int google_lga_pcie_retrain_link(struct dw_pcie *pci)
{
	u8 cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u16 lnkctl, lnksta;
	int ret;

	lnkctl = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL);
	lnkctl |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL, lnkctl);

	ret = read_poll_timeout(dw_pcie_readw_dbi, lnksta,
				!(lnksta & PCI_EXP_LNKSTA_LT), 100,
				LGA_PCIE_RETRAIN_TIMEOUT_US, false, pci,
				cap + PCI_EXP_LNKSTA);
	return ret;
}

static int google_lga_pcie_wait_link_fast(struct google_lga_pcie *lga)
{
	unsigned int i;

	for (i = 0; i < LGA_PCIE_LINK_WAIT_RETRIES; i++) {
		if (google_lga_pcie_link_up(&lga->pci))
			return 0;
		usleep_range(LGA_PCIE_LINK_WAIT_US, LGA_PCIE_LINK_WAIT_US + 2);
	}

	return -ETIMEDOUT;
}

static void google_lga_pcie_send_pme_turn_off(struct google_lga_pcie *lga)
{
	u32 val;
	int ret;

	val = readl(lga->sii + LGA_PCIE_SII_PM_MSG);
	val |= LGA_PCIE_SII_PME_TURNOFF;
	writel(val, lga->sii + LGA_PCIE_SII_PM_MSG);
	val &= ~LGA_PCIE_SII_PME_TURNOFF;
	writel(val, lga->sii + LGA_PCIE_SII_PM_MSG);

	ret = readl_poll_timeout(lga->sii + LGA_PCIE_SII_PM_MSG, val,
				 val & LGA_PCIE_SII_PME_TO_ACK,
				 LGA_PCIE_L2_ACK_POLL_US,
				 LGA_PCIE_L2_ACK_TIMEOUT_US);
	if (!ret)
		writel(LGA_PCIE_SII_PME_TO_ACK, lga->sii + LGA_PCIE_SII_PM_MSG);

	read_poll_timeout(google_lga_pcie_get_link_state, val,
			  val == LGA_LINK_L2, LGA_PCIE_L2_STATE_POLL_US,
			  LGA_PCIE_L2_STATE_TIMEOUT_US, false, lga);
}

int google_pcie_rc_poweron(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);
	struct device *dev;
	struct dw_pcie *pci;
	struct pci_bus *bus;
	struct pci_dev *rc_dev;
	unsigned long flags;
	bool rescan;
	bool link_up = false;
	int attempt;
	int ret;

	if (!lga || !lga->on_demand)
		return -EINVAL;

	dev = lga->pci.dev;
	pci = &lga->pci;

	rescan = !READ_ONCE(lga->enumeration_done) || lga->orig_child_ops;
	if (rescan)
		pci_lock_rescan_remove();

	mutex_lock(&lga->link_lock);
	if (lga->is_link_up) {
		ret = -EINVAL;
		goto unlock;
	}

	bus = pci->pp.bridge->bus;
	rc_dev = pci_get_slot(bus, 0);
	if (!rc_dev) {
		ret = -ENODEV;
		goto unlock;
	}

	if (lga->perstn_gpio)
		gpiod_set_value_cansleep(lga->perstn_gpio, 1);

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto put_rc_dev;

	ret = google_lga_pcie_power_up_hw(lga);
	if (ret)
		goto rpm_put;

	if (lga->saved_state) {
		ret = pci_load_saved_state(rc_dev, lga->saved_state);
		if (ret)
			goto hw_down;
		pci_restore_state(rc_dev);
	}

	pci->max_link_speed = lga->target_link_speed ?: lga->max_link_speed;
	ret = google_lga_pcie_setup_rc(lga);
	if (ret)
		goto hw_down;
	google_lga_pcie_configure_dbi(lga);
	google_lga_pcie_set_msi_irqs(lga, true);

	lga->poweron_active = true;
	for (attempt = 0; attempt < LGA_PCIE_LINK_RETRIES; attempt++) {
		if (!link_up)
			ret = google_lga_pcie_start_link(pci);
		else
			ret = google_lga_pcie_retrain_link(pci);

		if (!ret)
			ret = google_lga_pcie_wait_link_fast(lga);

		if (ret) {
			link_up = false;
			if (lga->perstn_gpio)
				gpiod_set_value_cansleep(lga->perstn_gpio, 1);
			google_lga_pcie_stop_link(pci);
			usleep_range(lga->perst_delay_us,
				     lga->perst_delay_us + 2000);
			continue;
		}

		link_up = true;
		google_lga_pcie_fetch_speed_width(lga);
		if (!lga->target_link_speed ||
		    lga->current_link_speed == lga->target_link_speed)
			break;
	}
	lga->poweron_active = false;

	if (!link_up) {
		dev_err(dev, "PCIe link failed to train\n");
		ret = -EPIPE;
		goto irq_off;
	}

	if (lga->cpl_timeout_irq > 0)
		synchronize_irq(lga->cpl_timeout_irq);

	spin_lock_irqsave(&lga->state_lock, flags);
	lga->is_link_up = true;
	spin_unlock_irqrestore(&lga->state_lock, flags);

	google_lga_pcie_set_err_irqs(lga, true);
	google_lga_pcie_set_auto_gate(lga, true);

	if (!lga->enumeration_done) {
		pci_rescan_bus(bus);
		WRITE_ONCE(lga->enumeration_done, true);
	}

	pci_dev_put(rc_dev);
	mutex_unlock(&lga->link_lock);
	if (rescan)
		pci_unlock_rescan_remove();
	return 0;

irq_off:
	google_lga_pcie_set_msi_irqs(lga, false);
hw_down:
	google_lga_pcie_power_down_hw(lga);
rpm_put:
	lga->current_link_speed = 0;
	lga->current_link_width = 0;
	pm_runtime_put_sync(dev);
put_rc_dev:
	pci_dev_put(rc_dev);
unlock:
	mutex_unlock(&lga->link_lock);
	if (rescan)
		pci_unlock_rescan_remove();
	return ret;
}
EXPORT_SYMBOL_GPL(google_pcie_rc_poweron);

int google_pcie_rc_poweroff(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);
	struct dw_pcie *pci;
	struct pci_dev *rc_dev;
	unsigned long flags;
	bool in_rec;
	bool cpl_to;
	bool disable_ltssm = false;
	bool restore_link_down_irq = false;

	if (!lga || !lga->on_demand)
		return -EINVAL;

	pci = &lga->pci;

	mutex_lock(&lga->link_lock);
	in_rec = google_lga_pcie_in_recovery(lga);
	if (!lga->is_link_up && !in_rec) {
		mutex_unlock(&lga->link_lock);
		return -EINVAL;
	}

	rc_dev = pci_get_slot(pci->pp.bridge->bus, 0);
	if (rc_dev && lga->powered_on) {
		pci_save_state(rc_dev);
		kfree(lga->saved_state);
		lga->saved_state = pci_store_saved_state(rc_dev);
	}
	if (rc_dev)
		pci_dev_put(rc_dev);

	google_lga_pcie_set_auto_gate(lga, false);

	spin_lock_irqsave(&lga->state_lock, flags);
	lga->is_link_up = false;
	spin_unlock_irqrestore(&lga->state_lock, flags);

	google_lga_pcie_set_err_irqs(lga, false);
	google_lga_pcie_set_msi_irqs(lga, false);

	lga->current_link_speed = 0;
	lga->current_link_width = 0;

	cpl_to = test_bit(LGA_REC_CPL_TIMEOUT, &lga->recovery_flags);
	if (!in_rec && lga->powered_on) {
		google_lga_pcie_send_pme_turn_off(lga);
		usleep_range(1000, 1200);
		disable_ltssm = true;
	} else if (cpl_to && lga->powered_on) {
		u32 val = readl(lga->top + LGA_PCIE_TOP_RESET_STATUS);

		val |= LGA_PCIE_TOP_APP_INIT_RST;
		writel(val, lga->top + LGA_PCIE_TOP_RESET_STATUS);
	}

	if (disable_ltssm)
		google_lga_pcie_stop_link(pci);

	google_lga_pcie_power_down_hw(lga);
	pm_runtime_put_sync(pci->dev);

	spin_lock_irqsave(&lga->state_lock, flags);
	if (test_and_clear_bit(LGA_REC_LINK_DOWN, &lga->recovery_flags))
		restore_link_down_irq = true;
	if (test_and_clear_bit(LGA_REC_CPL_TIMEOUT, &lga->recovery_flags))
		restore_link_down_irq = true;
	spin_unlock_irqrestore(&lga->state_lock, flags);

	/*
	 * Recovery ISRs called disable_irq_nosync(link_down_irq) before
	 * google_lga_pcie_set_err_irqs() checked link_down_irq_enabled; undo any
	 * recovery-path disable reference so enable_irq() stays balanced.
	 */
	if (restore_link_down_irq && lga->link_down_irq > 0)
		enable_irq(lga->link_down_irq);

	mutex_unlock(&lga->link_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(google_pcie_rc_poweroff);

void google_pcie_rc_prepare_for_forced_poweroff(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);
	unsigned long flags;

	if (!lga)
		return;

	spin_lock_irqsave(&lga->state_lock, flags);
	lga->is_link_up = false;
	set_bit(LGA_REC_LINK_DOWN, &lga->recovery_flags);
	spin_unlock_irqrestore(&lga->state_lock, flags);
}
EXPORT_SYMBOL_GPL(google_pcie_rc_prepare_for_forced_poweroff);

int google_pcie_poweron_withspeed(int num, unsigned int speed)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;
	if (!speed || speed > lga->max_link_speed)
		return -EINVAL;

	lga->target_link_speed = speed;
	return google_pcie_rc_poweron(num);
}
EXPORT_SYMBOL_GPL(google_pcie_poweron_withspeed);

int google_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;

	lga->pci.pp.msi_data = msi_ctrl_addr;
	if (lga->powered_on)
		dw_pcie_msi_init(&lga->pci.pp);

	return 0;
}
EXPORT_SYMBOL_GPL(google_pcie_set_msi_ctrl_addr);

int google_pcie_link_state(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;

	return google_lga_pcie_get_link_state(lga);
}
EXPORT_SYMBOL_GPL(google_pcie_link_state);

int google_pcie_link_status(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;

	return google_lga_pcie_get_link_state(lga) < LGA_LINK_L2;
}
EXPORT_SYMBOL_GPL(google_pcie_link_status);

int google_pcie_inbound_atu_cfg(int ch_num, unsigned long long src_addr,
				unsigned long long dst_addr,
				unsigned long size, int atu_index)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(ch_num);

	if (!lga)
		return -EINVAL;
	if (!lga->powered_on)
		return -ENODEV;

	return dw_pcie_prog_inbound_atu(&lga->pci, atu_index,
					PCIE_TLP_TYPE_MEM_RDWR,
					dst_addr, src_addr, size);
}
EXPORT_SYMBOL_GPL(google_pcie_inbound_atu_cfg);

int google_pcie_get_max_link_speed(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	return lga ? lga->max_link_speed : -EINVAL;
}
EXPORT_SYMBOL_GPL(google_pcie_get_max_link_speed);

int google_pcie_get_max_link_width(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	return lga ? lga->max_link_width : -EINVAL;
}
EXPORT_SYMBOL_GPL(google_pcie_get_max_link_width);

int google_pcie_rc_change_link_speed(int num, unsigned int speed)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);
	struct dw_pcie *pci;
	u32 old_speed;
	u8 cap;
	u16 lnkctl2, lnksta;
	u32 port_ctrl;
	enum lga_pcie_link_state state;
	unsigned int i;
	int ret = 0;

	if (!lga)
		return -EINVAL;
	if (!speed || speed > lga->max_link_speed)
		return -EINVAL;
	if (speed == lga->current_link_speed)
		return 0;

	mutex_lock(&lga->link_lock);
	old_speed = lga->target_link_speed;
	lga->target_link_speed = speed;
	if (!lga->is_link_up)
		goto unlock;

	if (google_lga_pcie_get_link_state(lga) != LGA_LINK_L0) {
		lga->target_link_speed = old_speed;
		ret = -EINVAL;
		goto unlock;
	}

	pci = &lga->pci;
	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	lnkctl2 = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL2);
	lnkctl2 &= ~PCI_EXP_LNKCTL2_TLS;
	lnkctl2 |= speed;
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL2, lnkctl2);

	port_ctrl = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	port_ctrl &= ~PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, port_ctrl);
	port_ctrl |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, port_ctrl);

	for (i = 0; i < LGA_PCIE_SPEED_WAIT_RETRIES; i++) {
		lnksta = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKSTA);
		if (FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta) == speed)
			break;
		udelay(LGA_PCIE_SPEED_WAIT_US);
	}

	if (FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta) != speed) {
		lga->target_link_speed = old_speed;
		ret = -EINVAL;
		goto unlock;
	}

	read_poll_timeout(google_lga_pcie_get_link_state, state,
			  state == LGA_LINK_L0, LGA_PCIE_L2_STATE_POLL_US,
			  LGA_PCIE_L0_STATE_TIMEOUT_US, false, lga);

	lga->current_link_speed = speed;
unlock:
	mutex_unlock(&lga->link_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(google_pcie_rc_change_link_speed);

int google_pcie_register_callback(int num, google_pcie_callback_func cb_func,
				  void *priv)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;

	lga->cb_func = cb_func;
	lga->cb_priv = priv;
	return 0;
}
EXPORT_SYMBOL_GPL(google_pcie_register_callback);

int google_pcie_unregister_callback(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	if (!lga)
		return -EINVAL;

	lga->cb_func = NULL;
	lga->cb_priv = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(google_pcie_unregister_callback);

void google_pcie_dump_debug(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);
	unsigned long flags;
	u32 pm_sts = 0, link_sts = 0, rst_sts = 0;

	if (!lga)
		return;

	spin_lock_irqsave(&lga->state_lock, flags);
	if (lga->powered_on) {
		pm_sts = readl(lga->sii + LGA_PCIE_SII_PM_STATE);
		rst_sts = readl(lga->sii + LGA_PCIE_SII_RST_STS);
		link_sts = readl(lga->top + LGA_PCIE_TOP_LINK_STS);
	}
	spin_unlock_irqrestore(&lga->state_lock, flags);

	dev_info(lga->pci.dev,
		 "PCIe dom%d: powered=%d link_up=%d pm=%#x link_sts=%#x rst=%#x\n",
		 lga->domain, lga->powered_on, lga->is_link_up,
		 pm_sts, link_sts, rst_sts);
}
EXPORT_SYMBOL_GPL(google_pcie_dump_debug);

bool google_pcie_is_link_down(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	return lga ? test_bit(LGA_REC_LINK_DOWN, &lga->recovery_flags) : false;
}
EXPORT_SYMBOL_GPL(google_pcie_is_link_down);

bool google_pcie_is_cpl_timeout(int num)
{
	struct google_lga_pcie *lga = google_lga_pcie_find(num);

	return lga ? test_bit(LGA_REC_CPL_TIMEOUT, &lga->recovery_flags) : false;
}
EXPORT_SYMBOL_GPL(google_pcie_is_cpl_timeout);

static const struct dw_pcie_ops google_lga_pcie_ops = {
	.read_dbi = google_lga_pcie_read_dbi,
	.write_dbi = google_lga_pcie_write_dbi,
	.link_up = google_lga_pcie_link_up,
	.get_ltssm = google_lga_pcie_get_ltssm,
	.start_link = google_lga_pcie_start_link,
	.stop_link = google_lga_pcie_stop_link,
};

static const struct dw_pcie_host_ops google_lga_pcie_host_ops = {
	.init = google_lga_pcie_host_init,
	.post_init = google_lga_pcie_host_post_init,
	.deinit = google_lga_pcie_host_deinit,
};

static int google_lga_pcie_request_irqs(struct platform_device *pdev,
					struct google_lga_pcie *lga)
{
	struct device *dev = &pdev->dev;
	int ret;

	lga->cpl_timeout_irq = platform_get_irq_byname_optional(pdev, "cpl_timeout");
	if (lga->cpl_timeout_irq > 0) {
		ret = devm_request_threaded_irq(dev, lga->cpl_timeout_irq,
						google_lga_pcie_cpl_timeout_isr,
						google_lga_pcie_cpl_timeout_thread,
						IRQF_NO_AUTOEN,
						"lga-pcie-cpl-timeout", lga);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request cpl_timeout irq\n");
	}

	lga->link_down_irq = platform_get_irq_byname_optional(pdev, "link_down");
	if (lga->link_down_irq > 0) {
		ret = devm_request_threaded_irq(dev, lga->link_down_irq,
						google_lga_pcie_link_down_isr,
						google_lga_pcie_link_down_thread,
						IRQF_NO_AUTOEN,
						"lga-pcie-link-down", lga);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request link_down irq\n");
	}

	return 0;
}

static int google_lga_pcie_get_resources(struct platform_device *pdev,
					 struct google_lga_pcie *lga)
{
	struct device *dev = &pdev->dev;

	lga->sii = devm_platform_ioremap_resource_byname(pdev, "sii");
	if (IS_ERR(lga->sii))
		return PTR_ERR(lga->sii);

	lga->top = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(lga->top))
		return PTR_ERR(lga->top);

	lga->has_phy_sram = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							 "phy_sram") != NULL;

	lga->aux_clk = devm_clk_get_optional(dev, "aux_clk");
	if (IS_ERR(lga->aux_clk))
		return dev_err_probe(dev, PTR_ERR(lga->aux_clk),
				     "failed to get aux_clk\n");

	lga->phy_fw_clk = devm_clk_get(dev, "phy_fw");
	if (IS_ERR(lga->phy_fw_clk))
		return dev_err_probe(dev, PTR_ERR(lga->phy_fw_clk),
				     "failed to get phy_fw clock\n");

	lga->rst_init = devm_reset_control_get_exclusive(dev, "init");
	if (IS_ERR(lga->rst_init))
		return dev_err_probe(dev, PTR_ERR(lga->rst_init),
				     "failed to get init reset\n");

	lga->rst_perst = devm_reset_control_get_exclusive(dev, "perst");
	if (IS_ERR(lga->rst_perst))
		return dev_err_probe(dev, PTR_ERR(lga->rst_perst),
				     "failed to get perst reset\n");

	lga->rst_app_hold = devm_reset_control_get_exclusive(dev, "app_hold_phy");
	if (IS_ERR(lga->rst_app_hold))
		return dev_err_probe(dev, PTR_ERR(lga->rst_app_hold),
				     "failed to get app_hold_phy reset\n");

	lga->rst_power_up = devm_reset_control_get_exclusive(dev, "power_up");
	if (IS_ERR(lga->rst_power_up))
		return dev_err_probe(dev, PTR_ERR(lga->rst_power_up),
				     "failed to get power_up reset\n");

	lga->perstn_gpio = devm_gpiod_get_optional(dev, "perstn", GPIOD_OUT_HIGH);
	if (IS_ERR(lga->perstn_gpio))
		return dev_err_probe(dev, PTR_ERR(lga->perstn_gpio),
				     "failed to get perstn gpio\n");

	return 0;
}

static void google_lga_pcie_parse_dt(struct device *dev,
				     struct google_lga_pcie *lga)
{
	u32 val;

	lga->perst_delay_us = LGA_PCIE_DEFAULT_PERST_US;
	device_property_read_u32(dev, "google,perst-delay-us", &lga->perst_delay_us);
	device_property_read_u32(dev, "google,posted-rx-q-data-credits",
				 &lga->posted_rx_q_credits);
	lga->has_link_timeout = !device_property_read_u32(dev, "link-timeout-ms",
							  &lga->link_timeout_ms);
	lga->skip_link_eq = device_property_read_bool(dev, "google,skip-link-equalization");
	lga->l1_pwrgate_disable = device_property_read_bool(dev, "google,l1-pwrgate-disable");
	lga->allow_suspend_in_linkup =
		device_property_read_bool(dev, "google,allow-suspend-in-linkup");
	lga->on_demand = lga->allow_suspend_in_linkup ||
			 !of_get_available_child_count(dev->of_node);

	lga->domain = of_get_pci_domain_nr(dev->of_node);
	if (lga->domain < 0)
		lga->domain = 0;

	lga->max_link_speed = 4;
	if (!device_property_read_u32(dev, "max-link-speed", &val))
		lga->max_link_speed = val;
	lga->target_link_speed = lga->max_link_speed;

	lga->max_link_width = 1;
	if (!device_property_read_u32(dev, "num-lanes", &val))
		lga->max_link_width = val;
}

static int google_lga_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct google_lga_pcie *lga;
	struct pci_dev *rc_dev;
	unsigned long flags;
	int ret;

	lga = devm_kzalloc(dev, sizeof(*lga), GFP_KERNEL);
	if (!lga)
		return -ENOMEM;

	lga->pci.dev = dev;
	lga->pci.ops = &google_lga_pcie_ops;
	lga->pci.pp.ops = &google_lga_pcie_host_ops;
	mutex_init(&lga->link_lock);
	spin_lock_init(&lga->state_lock);
	raw_spin_lock_init(&lga->cfg_lock);
	INIT_LIST_HEAD(&lga->node);
	platform_set_drvdata(pdev, lga);

	ret = google_lga_pcie_get_resources(pdev, lga);
	if (ret)
		return ret;

	google_lga_pcie_parse_dt(dev, lga);

	ret = google_lga_pcie_request_irqs(pdev, lga);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "failed to runtime-resume PCIe RC\n");
	}

	ret = dw_pcie_host_init(&lga->pci.pp);
	if (ret) {
		pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	spin_lock_irqsave(&google_lga_pcie_list_lock, flags);
	list_add_tail(&lga->node, &google_lga_pcie_list);
	spin_unlock_irqrestore(&google_lga_pcie_list_lock, flags);

	if (lga->on_demand) {
		rc_dev = pci_get_slot(lga->pci.pp.bridge->bus, 0);
		if (rc_dev) {
			pci_save_state(rc_dev);
			lga->saved_state = pci_store_saved_state(rc_dev);
			pci_dev_put(rc_dev);
		}
		google_lga_pcie_set_err_irqs(lga, false);
		google_lga_pcie_set_msi_irqs(lga, false);
		google_lga_pcie_power_down_hw(lga);
		pm_runtime_put_sync(dev);
	}

	return 0;
}

static void google_lga_pcie_remove(struct platform_device *pdev)
{
	struct google_lga_pcie *lga = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	unsigned long flags;

	spin_lock_irqsave(&google_lga_pcie_list_lock, flags);
	list_del_init(&lga->node);
	spin_unlock_irqrestore(&google_lga_pcie_list_lock, flags);

	if (lga->on_demand && lga->is_link_up)
		google_pcie_rc_poweroff(lga->domain);

	if (lga->on_demand) {
		/*
		 * dw_pcie_host_deinit() runs stop_link() and deinit(); keep the
		 * runtime PM reference active while tearing down the bridge.
		 */
		if (pm_runtime_resume_and_get(dev))
			dev_warn(dev, "failed to runtime-resume during remove\n");
		dw_pcie_host_deinit(&lga->pci.pp);
		kfree(lga->saved_state);
		pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return;
	}

	dw_pcie_host_deinit(&lga->pci.pp);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

static const struct of_device_id google_lga_pcie_of_match[] = {
	{ .compatible = "google,lga-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, google_lga_pcie_of_match);

static struct platform_driver google_lga_pcie_driver = {
	.probe = google_lga_pcie_probe,
	.remove = google_lga_pcie_remove,
	.driver = {
		.name = "google-lga-pcie",
		.of_match_table = google_lga_pcie_of_match,
	},
};
module_platform_driver(google_lga_pcie_driver);

MODULE_AUTHOR("Bobby Bones <saariuscrypto@gmail.com>");
MODULE_DESCRIPTION("Google Laguna (Tensor G5) PCIe host controller driver");
MODULE_LICENSE("GPL");
