/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_MRDUMP_NAMED_DUMP_H
#define MTK_MRDUMP_NAMED_DUMP_H

/* Process context only. Stop buffer writers, unregister with fn=NULL, then free.
 * Original TIMER_LIST, HWT_IRQ and HANG_DETECT names are accepted.
 * Getters may run during registration and panic; they must not update this API. Existing
 * enum-based client IDs and their exported interface remain unchanged.
 */
int mrdump_set_extra_dump_named(const char *name,
		void (*fn)(unsigned long *vaddr, unsigned long *size));

#endif
