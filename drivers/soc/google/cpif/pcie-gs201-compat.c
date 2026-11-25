// SPDX-License-Identifier: GPL-2.0
/*
 * The pcie_*() calls CPIF makes into Google's gs201 modem link driver
 * (pcie-gs201-cp), routed via cpif_pcie_shim.h when MODEM_SEC_5300 is used.
 */

#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/soc/google/gs201-pcie-cp.h>
#include <linux/types.h>

#include "modem_prj.h"
#include "s51xx_pcie.h"

#define CP_PCIE_CH	0

static bool cp_ch_ok(int ch_num)
{
	if (ch_num == CP_PCIE_CH)
		return true;
	pr_err_once("cpif: PCIe channel %d does not exist, only %d\n",
		    ch_num, CP_PCIE_CH);
	return false;
}

static void cp_compat_event(struct gs201_pcie_cp_event *ev, u32 event)
{
	pcie_register_event_t *reg = ev->priv;

	reg->notify.event = event & GS201_PCIE_CP_EVENT_LINKDOWN ?
			    PCIE_EVENT_LINKDOWN :
			    PCIE_EVENT_CPL_TIMEOUT;
	reg->notify.user = reg->user;
	if (reg->mode == PCIE_TRIGGER_CALLBACK && reg->callback)
		reg->callback(&reg->notify);
	else if (reg->completion)
		complete(reg->completion);
}

static struct gs201_pcie_cp_event cp_compat_ev = {
	.callback = cp_compat_event,
};

int cpif_gs201_pcie_register_event(pcie_register_event_t *reg)
{
	if (!reg || !reg->user)
		return -ENODEV;
	cp_compat_ev.events = 0;
	if (reg->events & PCIE_EVENT_LINKDOWN)
		cp_compat_ev.events |= GS201_PCIE_CP_EVENT_LINKDOWN;
	if (reg->events & PCIE_EVENT_CPL_TIMEOUT)
		cp_compat_ev.events |= GS201_PCIE_CP_EVENT_CPL_TIMEOUT;
	cp_compat_ev.priv = reg;
	return gs201_pcie_cp_register_event(&cp_compat_ev);
}

int cpif_gs201_pcie_deregister_event(pcie_register_event_t *reg)
{
	if (cp_compat_ev.priv != reg)
		return -ENODEV;
	gs201_pcie_cp_deregister_event(&cp_compat_ev);
	cp_compat_ev.priv = NULL;
	return 0;
}

void cpif_gs201_pcie_register_dump(int ch_num)
{
	gs201_pcie_cp_dump();
}

void cpif_gs201_pcie_dump_all_status(int ch_num)
{
	gs201_pcie_cp_dump();
}

void cpif_gs201_pcie_print_msi_register(int ch_num)
{
	gs201_pcie_cp_dump();
}

/* BTL only: not ported */
int cpif_gs201_pcie_set_outbound_atu(int ch_num, u32 target_addr, u32 offset,
				     u32 size)
{
	return -EOPNOTSUPP;
}

bool cpif_gs201_pcie_get_cpl_timeout_state(int ch_num)
{
	return gs201_pcie_cp_faults() & GS201_PCIE_CP_EVENT_CPL_TIMEOUT;
}

void cpif_gs201_pcie_set_cpl_timeout_state(int ch_num, bool recovery)
{
	if (!recovery)
		gs201_pcie_cp_clear_faults(GS201_PCIE_CP_EVENT_CPL_TIMEOUT);
}

bool cpif_gs201_pcie_get_sudden_linkdown_state(int ch_num)
{
	return gs201_pcie_cp_faults() & GS201_PCIE_CP_EVENT_LINKDOWN;
}

void cpif_gs201_pcie_set_sudden_linkdown_state(int ch_num, bool recovery)
{
	if (!recovery)
		gs201_pcie_cp_clear_faults(GS201_PCIE_CP_EVENT_LINKDOWN);
}

void cpif_gs201_pcie_force_linkdown_work(int ch_num)
{
	gs201_pcie_cp_force_linkdown();
}

/* The doorbell write under the controller's link lock (see gs201_pcie_cp_doorbell()) */
int cpif_gs201_pcie_doorbell(int ch_num, void __iomem *addr, u32 val, u32 *readback)
{
	return cp_ch_ok(ch_num) ? gs201_pcie_cp_doorbell(addr, val, readback) : -ENODEV;
}

int cpif_gs201_pcie_chk_link_status(int ch_num)
{
	return cp_ch_ok(ch_num) && gs201_pcie_cp_link_ok();
}

int cpif_gs201_pcie_l1ss_ctrl(int enable, int id, int ch_num)
{
	return cp_ch_ok(ch_num) ? gs201_pcie_cp_l1ss(enable) : -ENODEV;
}

/* The width is the modem's to choose, as in Google's driver */
int cpif_gs201_pcie_poweron(int ch_num, int spd, int width)
{
	return cp_ch_ok(ch_num) ? gs201_pcie_cp_poweron(spd) : -ENODEV;
}

int cpif_gs201_pcie_poweroff(int ch_num)
{
	if (!cp_ch_ok(ch_num))
		return -ENODEV;
	gs201_pcie_cp_poweroff();
	return 0;
}

void cpif_gs201_pcie_set_perst_gpio(int ch_num, bool on)
{
	if (cp_ch_ok(ch_num))
		gs201_pcie_cp_set_perst_level(on);
}

void cpif_gs201_pcie_set_ready_cto_recovery(int ch_num)
{
	if (cp_ch_ok(ch_num))
		gs201_pcie_cp_prepare_recovery();
}

/* The packet processor's own MSI blocks, one per downlink queue */
int cpif_gs201_pcie_register_separated_msi_vector(int ch_num, irq_handler_t handler,
						  void *context, int *irq_num)
{
	return cp_ch_ok(ch_num) ?
	       gs201_pcie_cp_register_queue(handler, context, irq_num) :
	       -ENODEV;
}

/* The target is the link's memory-region; check that both nodes agree. */
int cpif_gs201_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr)
{
	return cp_ch_ok(num) ? gs201_pcie_cp_check_msi_addr(msi_ctrl_addr) :
			       -ENODEV;
}
