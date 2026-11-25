/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPU_PMA_CHECKED_H
#define GPU_PMA_CHECKED_H
#include <linux/types.h>
static inline bool pma_gpr_fits(u64 size, u32 offset, u32 index)
{
	u64 address = (u64)offset + (u64)index * 4;
	return offset && !(offset & 3) && size >= 4 && address <= size - 4;
}
static inline bool pma_range_fits(u64 base, u64 size, u64 reserved_base, u64 reserved_size)
{
	return base && size && base >= reserved_base && size <= reserved_size &&
		base - reserved_base <= reserved_size - size;
}
#endif
