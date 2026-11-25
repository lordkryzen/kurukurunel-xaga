/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SWPM_SHARED_RANGE_H
#define SWPM_SHARED_RANGE_H
/* Reject wraparound, inclusive-end provider semantics and misaligned wire records. */
static inline bool swpm_shared_range_valid(u32 base, u32 size, u32 offset, size_t bytes)
{
	if (!bytes || offset < base || (offset & 3) || bytes > size)
		return false;
	if (offset - base > size - bytes)
		return false;
	return bytes - 1 <= U32_MAX - offset;
}
static inline bool swpm_reserved_region_valid(u64 phys, u64 virt, u64 size, size_t record)
{
 return phys && virt && phys <= U32_MAX && size == 8192 &&
        size <= (u64)U32_MAX - phys + 1 && record <= size;
}
#endif
