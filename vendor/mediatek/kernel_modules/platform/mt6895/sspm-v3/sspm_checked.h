/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SSPM_SHARED_MEMORY_CHECKED_H
#define SSPM_SHARED_MEMORY_CHECKED_H
#include <linux/types.h>
static inline bool sspm_range_fits(u64 base, u64 size, u64 reserved_base, u64 reserved_size)
{
	return base && size && base >= reserved_base && size <= reserved_size &&
		base - reserved_base <= reserved_size - size;
}
static inline bool sspm_budget_fits(u64 used, u64 next, u64 total)
{
	return used <= total && next <= total - used;
}
#endif
