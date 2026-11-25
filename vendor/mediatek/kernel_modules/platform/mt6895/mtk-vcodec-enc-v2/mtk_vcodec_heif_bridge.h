/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_HEIF_BRIDGE_H_
#define _MTK_VCODEC_HEIF_BRIDGE_H_

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/unaligned.h>

/* Retained libheichal uses these four original scalar controls. Their native
 * counterparts have the same ranges and values, but different numeric IDs.
 * Keep this list encoder-local: other legacy IDs can collide with native UAPI.
 */
static inline int mtk_codec_heif_legacy_control(u32 id)
{
	return id == 0x0099201b || /* ENCODE_GRID_SIZE */
	       id == 0x0099201f || /* ENCODE_RC_I_FRAME_QP */
	       id == 0x00992020 || /* ENCODE_RC_P_FRAME_QP */
	       id == 0x00992021;   /* ENCODE_RC_B_FRAME_QP */
}

/* Original MT6895 firmware appends 4096 bytes: (tile index, byte count)
 * pairs, zero padding, and the sequence-header byte count in the final u32.
 * The paired newer Codec2 HeifImpl instead locates a type1/8192-byte record
 * through an offset+META trailer. Translate only the metadata container;
 * preserve every encoded tile byte and its firmware-provided boundary.
 * Actual firmware fixtures and the native parser establish this wire layout.
 */
static inline int mtk_codec_heif_grid_bridge(void *buffer, u32 capacity,
		u32 used, u32 columns, u32 rows, u32 *new_used)
{
	u8 *data = buffer;
	u32 count, payload, total, i, sum = 0;
	u32 header_size;

	if (!buffer || !new_used || used > capacity)
		return -EINVAL;
	/* Header/EOS buffers have no old grid footer. */
	if (used < 4096) {
		*new_used = used;
		return 0;
	}
	/* No interpretation or alteration of an already tagged native container. */
	if (get_unaligned_le32(data + used - 4) == 0x4154454d) {
		*new_used = used;
		return 0;
	}
	if (!columns || !rows || columns > 511 || rows > 511 / columns)
		return -EINVAL;
	count = columns * rows;
	payload = used - 4096;
	header_size = get_unaligned_le32(data + used - 4);
	if (!payload || !header_size || header_size > 4096)
		return -EINVAL;
	/* Validate the complete table before any write; malformed/partial output
	 * must not become an apparently successful but corrupt HEIF image.
	 */
	for (i = 0; i < count; i++) {
		u32 index = get_unaligned_le32(data + payload + i * 8);
		u32 bytes = get_unaligned_le32(data + payload + i * 8 + 4);

		if (index != i || !bytes || bytes > payload - sum)
			return -EINVAL;
		sum += bytes;
	}
	if (sum != payload)
		return -EINVAL;
	for (i = count * 8; i < 4092; i++)
		if (data[payload + i])
			return -EINVAL;
	if (capacity - payload < 8208)
		return -ENOSPC;
	total = payload + 8208;
	memmove(data + payload + 8, data + payload, count * 8);
	memset(data + payload + 8 + count * 8, 0, 8192 - count * 8);
	put_unaligned_le32(1, data + payload);
	put_unaligned_le32(8192, data + payload + 4);
	put_unaligned_le32(payload, data + total - 8);
	put_unaligned_le32(0x4154454d, data + total - 4);
	*new_used = total;
	return 1;
}

#endif
