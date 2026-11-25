// SPDX-License-Identifier: GPL-2.0-only
/*
 * Notifier chains for cellular modem lifecycle, voice-call, and crash events.
 */

#include <linux/module.h>
#include <linux/notifier.h>
#define __MODEM_NOTIFIER_IMPL
#include <soc/google/modem_notifier.h>

static ATOMIC_NOTIFIER_HEAD(modem_event_notifier);
static BLOCKING_NOTIFIER_HEAD(modem_voice_call_event_notifier);
static BLOCKING_NOTIFIER_HEAD(modem_force_crash_notifier);

int register_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return atomic_notifier_chain_register(&modem_event_notifier, nb);
}
EXPORT_SYMBOL_GPL(register_modem_event_notifier);

void modem_notify_event(enum modem_event evt, void *mc)
{
	atomic_notifier_call_chain(&modem_event_notifier, evt, mc);
}
EXPORT_SYMBOL_GPL(modem_notify_event);

int register_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return blocking_notifier_chain_register(&modem_voice_call_event_notifier,
						nb);
}
EXPORT_SYMBOL_GPL(register_modem_voice_call_event_notifier);

void unregister_modem_voice_call_event_notifier(struct notifier_block *nb)
{
	if (nb)
		blocking_notifier_chain_unregister(&modem_voice_call_event_notifier,
						   nb);
}
EXPORT_SYMBOL_GPL(unregister_modem_voice_call_event_notifier);

void modem_voice_call_notify_event(enum modem_voice_call_event evt, void *data)
{
	blocking_notifier_call_chain(&modem_voice_call_event_notifier,
				     evt, data);
}
EXPORT_SYMBOL_GPL(modem_voice_call_notify_event);

int register_modem_force_crash_handler(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;

	return blocking_notifier_chain_register(&modem_force_crash_notifier, nb);
}
EXPORT_SYMBOL_GPL(register_modem_force_crash_handler);

void unregister_modem_force_crash_handler(struct notifier_block *nb)
{
	if (nb)
		blocking_notifier_chain_unregister(&modem_force_crash_notifier,
						   nb);
}
EXPORT_SYMBOL_GPL(unregister_modem_force_crash_handler);

int modem_force_crash_exit_ext(const char *buf)
{
	return blocking_notifier_call_chain(&modem_force_crash_notifier,
					    0, (void *)buf);
}
EXPORT_SYMBOL_GPL(modem_force_crash_exit_ext);

MODULE_DESCRIPTION("Google modem event notifier chains");
MODULE_LICENSE("GPL");
