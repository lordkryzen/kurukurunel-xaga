/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_MONITOR_STATE_H
#define MTK_MONITOR_STATE_H

#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct monitor_control_state {
	bool enabled, trace, poll, zygote_stopped, reboot, first_done, fatal_claimed;
	int timeout, counter;
	u64 epoch;
};

struct monitor_control_event {
	struct monitor_control_state state;
	bool active, notify, expired;
};

static DEFINE_RAW_SPINLOCK(monitor_control_lock);
static struct monitor_control_state monitor_control = {
	.timeout = INT_MAX,
	.counter = INT_MAX,
};

static inline struct monitor_control_state monitor_control_snapshot(void)
{
	struct monitor_control_state state;
	unsigned long flags;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	state = monitor_control;
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return state;
}

static inline void monitor_control_reset(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	monitor_control.first_done = false;
	monitor_control.fatal_claimed = false;
	monitor_control.epoch++;
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
}

static inline bool monitor_control_kick(int parameter)
{
	unsigned long flags;
	bool accepted = false;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	if (monitor_control.reboot)
		goto out;
	if (!parameter) {
		monitor_control.enabled = false;
		monitor_control.counter = monitor_control.timeout;
	} else if (parameter > 0) {
		int seconds = parameter & 0x1000 ? parameter & 0x0fff : parameter;
		int ticks = ((long)seconds + HD_INTER - 1) / HD_INTER;

		if (!(parameter & 0x1000))
			monitor_control.enabled = true;
		monitor_control.timeout = max(ticks, 3);
		monitor_control.counter = monitor_control.timeout;
	}
	monitor_control.first_done = false;
	monitor_control.fatal_claimed = false;
	monitor_control.epoch++;
	accepted = true;
out:
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return accepted;
}

static inline bool monitor_control_zygote(bool stopped)
{
	unsigned long flags;
	bool changed;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	changed = stopped || monitor_control.zygote_stopped;
	if (changed) {
		monitor_control.enabled = !stopped;
		monitor_control.zygote_stopped = stopped;
		monitor_control.epoch++;
	}
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return changed;
}

static inline void monitor_control_start(bool reboot)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	if (reboot)
		monitor_control.reboot = true;
	monitor_control.timeout = reboot ? 5 : 9;
	monitor_control.counter = monitor_control.timeout;
	monitor_control.enabled = true;
	monitor_control.fatal_claimed = false;
	monitor_control.epoch++;
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
}

static inline void monitor_control_poll_arm(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	monitor_control.poll = true;
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
}

static inline bool monitor_control_poll_ready(void)
{
	unsigned long flags;
	bool ready;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	ready = monitor_control.enabled && monitor_control.trace;
	if (ready)
		monitor_control.trace = false;
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return ready;
}

static inline struct monitor_control_event monitor_control_tick(bool eligible)
{
	struct monitor_control_event event = {};
	unsigned long flags;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	event.state = monitor_control;
	event.active = monitor_control.enabled && eligible;
	if (event.active) {
		if (monitor_control.poll && monitor_control.counter == 2) {
			monitor_control.trace = true;
			event.notify = true;
		}
		event.expired = monitor_control.counter <= 0;
		/* Claim this tick before diagnostics, so a concurrent kick survives. */
		if (monitor_control.counter > INT_MIN)
			monitor_control.counter--;
	}
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return event;
}

static inline bool monitor_control_event_live(const struct monitor_control_event *event)
{
	struct monitor_control_state state = monitor_control_snapshot();

	return event->expired && state.enabled && !state.fatal_claimed &&
		state.epoch == event->state.epoch && state.first_done == event->state.first_done;
}

static inline bool monitor_control_commit(const struct monitor_control_event *event)
{
	unsigned long flags;
	bool accepted;

	raw_spin_lock_irqsave(&monitor_control_lock, flags);
	accepted = event->expired && monitor_control.enabled &&
		!monitor_control.fatal_claimed && monitor_control.epoch == event->state.epoch &&
		monitor_control.first_done == event->state.first_done;
	if (accepted) {
		if (event->state.first_done)
			monitor_control.fatal_claimed = true;
		else
			monitor_control.first_done = true;
	}
	raw_spin_unlock_irqrestore(&monitor_control_lock, flags);
	return accepted;
}

#endif
