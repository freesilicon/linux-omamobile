// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPIF shim layer for Google SoC PCIe (Laguna / S5400i).
 *
 * Copyright 2023, Google LLC
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include "modem_v1.h"
#include "cpif_pcie_shim.h"

static void cpif_pcie_callback(enum google_pcie_callback_type type, void *priv)
{
	pcie_register_event_t *reg = priv;
	enum pcie_event cpif_event_type;

	switch (type) {
	case GPCIE_CB_CPL_TIMEOUT:
		cpif_event_type = PCIE_EVENT_CPL_TIMEOUT;
		break;
	case GPCIE_CB_LINK_DOWN:
		cpif_event_type = PCIE_EVENT_LINKDOWN;
		break;
	default:
		cpif_event_type = PCIE_EVENT_INVALID;
		break;
	}

	if (reg && reg->callback && (reg->mode == PCIE_TRIGGER_CALLBACK) &&
	    (reg->events & cpif_event_type)) {
		struct pci_dev *pci_dev = reg->user;
		pcie_notify_t *notify = &reg->notify;

		notify->event = cpif_event_type;
		notify->user = reg->user;
		dev_info(&pci_dev->dev, "Callback for event %#x\n", notify->event);
		reg->callback(notify);
	}
}

int cpif_google_pcie_register_event(pcie_register_event_t *reg)
{
	if (reg && reg->user && reg->user->bus)
		return google_pcie_register_callback(reg->user->bus->domain_nr,
						     cpif_pcie_callback, reg);

	mif_err("Unable to determine PCIE controller\n");
	return -EINVAL;
}

int cpif_google_pcie_deregister_event(pcie_register_event_t *reg)
{
	if (reg && reg->user && reg->user->bus)
		return google_pcie_unregister_callback(reg->user->bus->domain_nr);

	mif_err("Unable to determine PCIE controller\n");
	return -EINVAL;
}

int cpif_google_pcie_l1ss_ctrl(int aspm_state, int ch_num)
{
	int ret;
	struct pci_bus *pci_bus;
	struct pci_dev *pci_dev;

	pci_bus = pci_find_bus(ch_num, 1);
	if (!pci_bus) {
		mif_err("Child bus-1 for channel %d not found\n", ch_num);
		return -ENODEV;
	}

	pci_dev = pci_get_slot(pci_bus, PCI_DEVFN(0, 0));
	if (!pci_dev) {
		mif_err("Endpoint device for channel %d not found\n", ch_num);
		return -ENODEV;
	}

	ret = pci_enable_link_state(pci_dev, aspm_state);
	pci_dev_put(pci_dev);
	return ret;
}

void cpif_google_pcie_check_pending_msi(struct modem_ctl *mc)
{
	/*
	 * Pending MSI status on LGA is already captured safely through
	 * google_pcie_dump_debug() when DBI clocks are guaranteed active.
	 */
	(void)mc;
}
