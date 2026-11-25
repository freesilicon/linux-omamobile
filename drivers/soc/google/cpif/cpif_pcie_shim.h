/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Unified PCIe shim dispatching between gs201 (pcie-gs201-cp) and
 * Laguna/LGA (pcie-google-lga) at runtime.
 */

#ifndef __CPIF_PCIE_SHIM_H__
#define __CPIF_PCIE_SHIM_H__

#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/pci.h>
#include <linux/pcie_google_if.h>
#include <linux/preempt.h>

struct modem_ctl;

enum pcie_event {
	PCIE_EVENT_INVALID = 0,
	PCIE_EVENT_LINKDOWN = BIT(0),
	PCIE_EVENT_LINKUP = BIT(1),
	PCIE_EVENT_WAKEUP = BIT(2),
	PCIE_EVENT_WAKE_RECOVERY = BIT(3),
	PCIE_EVENT_NO_ACCESS = BIT(4),
	PCIE_EVENT_CPL_TIMEOUT = BIT(5),
	PCIE_EVENT_LINKDOWN_RECOVERY_FAIL = BIT(6),
};

enum pcie_trigger {
	PCIE_TRIGGER_CALLBACK,
	PCIE_TRIGGER_COMPLETION,
};

typedef struct cpif_pcie_notify {
	enum pcie_event event;
	struct pci_dev *user;
	void *data;
	u32 options;
} pcie_notify_t;

typedef struct cpif_pcie_register_event {
	u32 events;
	struct pci_dev *user;
	enum pcie_trigger mode;
	void (*callback)(pcie_notify_t *notify);
	pcie_notify_t notify;
	struct completion *completion;
	u32 options;
} pcie_register_event_t;

void cpif_pcie_set_google(bool google);
bool cpif_pcie_is_google(void);

#if IS_REACHABLE(CONFIG_PCIE_GS201_CP)
int cpif_gs201_pcie_register_event(pcie_register_event_t *reg);
int cpif_gs201_pcie_deregister_event(pcie_register_event_t *reg);
void cpif_gs201_pcie_register_dump(int ch_num);
void cpif_gs201_pcie_dump_all_status(int ch_num);
void cpif_gs201_pcie_print_msi_register(int ch_num);
int cpif_gs201_pcie_set_outbound_atu(int ch_num, u32 target_addr, u32 offset, u32 size);
bool cpif_gs201_pcie_get_cpl_timeout_state(int ch_num);
void cpif_gs201_pcie_set_cpl_timeout_state(int ch_num, bool recovery);
bool cpif_gs201_pcie_get_sudden_linkdown_state(int ch_num);
void cpif_gs201_pcie_set_sudden_linkdown_state(int ch_num, bool recovery);
void cpif_gs201_pcie_force_linkdown_work(int ch_num);
int cpif_gs201_pcie_doorbell(int ch_num, void __iomem *addr, u32 val, u32 *readback);
int cpif_gs201_pcie_chk_link_status(int ch_num);
int cpif_gs201_pcie_l1ss_ctrl(int enable, int id, int ch_num);
int cpif_gs201_pcie_poweron(int ch_num, int spd, int width);
int cpif_gs201_pcie_poweroff(int ch_num);
void cpif_gs201_pcie_set_perst_gpio(int ch_num, bool on);
void cpif_gs201_pcie_set_ready_cto_recovery(int ch_num);
int cpif_gs201_pcie_register_separated_msi_vector(int ch_num,
						  irq_handler_t handler,
						  void *context, int *irq_num);
int cpif_gs201_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr);
#else
static inline int cpif_gs201_pcie_register_event(pcie_register_event_t *reg) { return -ENODEV; }
static inline int cpif_gs201_pcie_deregister_event(pcie_register_event_t *reg) { return -ENODEV; }
static inline void cpif_gs201_pcie_register_dump(int ch_num) {}
static inline void cpif_gs201_pcie_dump_all_status(int ch_num) {}
static inline void cpif_gs201_pcie_print_msi_register(int ch_num) {}
static inline int cpif_gs201_pcie_set_outbound_atu(int ch_num, u32 target_addr, u32 offset,
						    u32 size) { return -EOPNOTSUPP; }
static inline bool cpif_gs201_pcie_get_cpl_timeout_state(int ch_num) { return false; }
static inline void cpif_gs201_pcie_set_cpl_timeout_state(int ch_num, bool recovery) {}
static inline bool cpif_gs201_pcie_get_sudden_linkdown_state(int ch_num) { return false; }
static inline void cpif_gs201_pcie_set_sudden_linkdown_state(int ch_num, bool recovery) {}
static inline void cpif_gs201_pcie_force_linkdown_work(int ch_num) {}
static inline int cpif_gs201_pcie_doorbell(int ch_num, void __iomem *addr, u32 val,
					   u32 *readback) { return -ENODEV; }
static inline int cpif_gs201_pcie_chk_link_status(int ch_num) { return 0; }
static inline int cpif_gs201_pcie_l1ss_ctrl(int enable, int id, int ch_num) { return -ENODEV; }
static inline int cpif_gs201_pcie_poweron(int ch_num, int spd, int width) { return -ENODEV; }
static inline int cpif_gs201_pcie_poweroff(int ch_num) { return -ENODEV; }
static inline void cpif_gs201_pcie_set_perst_gpio(int ch_num, bool on) {}
static inline void cpif_gs201_pcie_set_ready_cto_recovery(int ch_num) {}
static inline int cpif_gs201_pcie_register_separated_msi_vector(int ch_num,
								irq_handler_t handler,
								void *context,
								int *irq_num) { return -ENODEV; }
static inline int cpif_gs201_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr) { return -ENODEV; }
#endif

#if IS_REACHABLE(CONFIG_PCIE_GOOGLE_LGA)
int cpif_google_pcie_register_event(pcie_register_event_t *reg);
int cpif_google_pcie_deregister_event(pcie_register_event_t *reg);
int cpif_google_pcie_l1ss_ctrl(int aspm_state, int ch_num);
void cpif_google_pcie_check_pending_msi(struct modem_ctl *mc);
#else
static inline int cpif_google_pcie_register_event(pcie_register_event_t *reg) { return -ENODEV; }
static inline int cpif_google_pcie_deregister_event(pcie_register_event_t *reg) { return -ENODEV; }
static inline int cpif_google_pcie_l1ss_ctrl(int aspm_state, int ch_num) { return -ENODEV; }
static inline void cpif_google_pcie_check_pending_msi(struct modem_ctl *mc) {}
#endif

static inline int pcie_register_event(pcie_register_event_t *reg)
{
	return cpif_pcie_is_google() ? cpif_google_pcie_register_event(reg)
				     : cpif_gs201_pcie_register_event(reg);
}

static inline int pcie_deregister_event(pcie_register_event_t *reg)
{
	return cpif_pcie_is_google() ? cpif_google_pcie_deregister_event(reg)
				     : cpif_gs201_pcie_deregister_event(reg);
}

static inline void pcie_register_dump(int ch)
{
	if (cpif_pcie_is_google()) {
		if (in_interrupt() || irqs_disabled())
			return;
		google_pcie_dump_debug(ch);
		return;
	}
	cpif_gs201_pcie_register_dump(ch);
}

static inline void pcie_dump_all_status(int ch)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_dump_all_status(ch);
}

static inline void pcie_print_rc_msi_register(int ch)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_print_msi_register(ch);
}

static inline int pcie_set_outbound_atu(int ch, u32 target_addr, u32 offset, u32 size)
{
	return cpif_pcie_is_google() ? 0
				     : cpif_gs201_pcie_set_outbound_atu(ch, target_addr, offset, size);
}

static inline bool pcie_get_cpl_timeout_state(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_is_cpl_timeout(ch)
				     : cpif_gs201_pcie_get_cpl_timeout_state(ch);
}

static inline void pcie_set_cpl_timeout_state(int ch, bool recovery)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_set_cpl_timeout_state(ch, recovery);
}

static inline bool pcie_get_sudden_linkdown_state(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_is_link_down(ch)
				     : cpif_gs201_pcie_get_sudden_linkdown_state(ch);
}

static inline void pcie_set_sudden_linkdown_state(int ch, bool recovery)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_set_sudden_linkdown_state(ch, recovery);
}

static inline void pcie_force_linkdown_work(int ch)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_force_linkdown_work(ch);
}

static inline int pcie_check_link_status(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_link_status(ch)
				     : cpif_gs201_pcie_chk_link_status(ch);
}

static inline int pcie_poweron(int ch, int speed, int width)
{
	return cpif_pcie_is_google() ? google_pcie_poweron_withspeed(ch, speed)
				     : cpif_gs201_pcie_poweron(ch, speed, width);
}

static inline int pcie_poweroff(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_rc_poweroff(ch)
				     : cpif_gs201_pcie_poweroff(ch);
}

static inline int pcie_get_max_link_speed(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_get_max_link_speed(ch) : 0;
}

static inline int pcie_get_max_link_width(int ch)
{
	return cpif_pcie_is_google() ? google_pcie_get_max_link_width(ch) : 0;
}

static inline int pcie_change_link_speed(int ch, int spd)
{
	return cpif_pcie_is_google() ? google_pcie_rc_change_link_speed(ch, spd) : 0;
}

static inline void pcie_set_perst_gpio(int ch, bool on)
{
	if (!cpif_pcie_is_google())
		cpif_gs201_pcie_set_perst_gpio(ch, on);
}

static inline void pcie_set_ready_cto_recovery(int ch)
{
	if (cpif_pcie_is_google()) {
		if (google_pcie_is_cpl_timeout(ch) || google_pcie_is_link_down(ch))
			google_pcie_rc_poweroff(ch);
		return;
	}
	cpif_gs201_pcie_set_ready_cto_recovery(ch);
}

static inline int pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr)
{
	return cpif_pcie_is_google() ? google_pcie_set_msi_ctrl_addr(num, msi_ctrl_addr)
				     : cpif_gs201_pcie_set_msi_ctrl_addr(num, msi_ctrl_addr);
}

static inline int pcie_register_separated_msi_vector(int ch, irq_handler_t handler,
						     void *context, int *irq_num)
{
	return cpif_pcie_is_google()
		? 0
		: cpif_gs201_pcie_register_separated_msi_vector(ch, handler, context, irq_num);
}

static inline void pcie_check_pending_msi(struct modem_ctl *mc)
{
	if (cpif_pcie_is_google())
		cpif_google_pcie_check_pending_msi(mc);
}

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_IOMMU)
static inline bool pcie_is_sysmmu_enabled(int ch_num)
{
	(void)ch_num;
	return false;
}

static inline void cpif_iommu_tlb_invalidate_all(struct modem_ctl *mc) {}

static inline int cpif_iommu_map(unsigned long iova, phys_addr_t paddr, size_t size,
				 int prot, struct modem_ctl *mc)
{
	return 0;
}

static inline size_t cpif_iommu_unmap(unsigned long iova, size_t size, struct modem_ctl *mc)
{
	return 0;
}
#endif

#endif /* __CPIF_PCIE_SHIM_H__ */
