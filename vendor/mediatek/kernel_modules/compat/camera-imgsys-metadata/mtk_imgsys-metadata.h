/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_IMGSYS_METADATA_H
#define MTK_IMGSYS_METADATA_H

#include <linux/types.h>

struct swfrm_info_t;

/* Subtraction avoids overflow; check before constructing a shared pointer. */
static inline bool imgsys_gce_span(size_t size, u64 offset, size_t length,
				 size_t alignment)
{
	return alignment && !(offset % alignment) &&
		offset <= size && length <= size - offset;
}

int imgsys_cmdq_validate_frame(struct swfrm_info_t *frame, void *base, size_t size);

#endif
