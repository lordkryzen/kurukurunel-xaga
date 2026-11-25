/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_BUFFER_BRIDGE_H_
#define _MTK_VCODEC_BUFFER_BRIDGE_H_

#include <linux/types.h>
#include <linux/errno.h>

struct mtk_codec_bitstream_region {
	__u64 dma;
	unsigned long va;
	__u32 size;
	__u32 length;
};

/* V4L2 bytesused includes data_offset. The legacy firmware takes a start
 * address and payload size, with no separate bitstream-offset IPI field.
 * Protected offsets cannot be assumed to be arithmetic on a secure token.
 */
static inline int mtk_codec_bitstream_region(__u64 dma, unsigned long va,
		__u32 length, __u32 used, __u32 offset, bool secure,
		struct mtk_codec_bitstream_region *out)
{
	struct mtk_codec_bitstream_region region;

	if (used > length || offset > used || (offset && offset == used))
		return -EINVAL;
	if (secure && offset)
		return -EOPNOTSUPP;
	if (length && (__u64)(length - 1) > ~(__u64)0 - dma)
		return -EOVERFLOW;
	if (!secure && va && offset > ~0UL - va)
		return -EOVERFLOW;
	if (!secure && used && ((dma + offset) & 63))
		return -EINVAL;

	region.dma = dma + offset;
	region.va = secure || !va ? 0 : va + offset;
	region.size = used - offset;
	region.length = length - offset;
	*out = region;
	return 0;
}

#endif
