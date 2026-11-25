/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_CONTROL_BRIDGE_H_
#define _MTK_VCODEC_CONTROL_BRIDGE_H_

#include <linux/types.h>
#include <linux/errno.h>
#include "mtk_vcodec_native_ids.h"

#define MTK_CODEC_DECODER 1U
#define MTK_CODEC_ENCODER 2U
#define MTK_CODEC_CONTEXT_ID 0xfffffff0U

struct mtk_codec_control_pair {
	__u32 legacy;
	__u32 native;
	unsigned int instances;
};

static const struct mtk_codec_control_pair mtk_codec_controls[] = {
#include "mtk_vcodec_control_map.h"
};

static inline __u32 mtk_codec_native_control(__u32 id, unsigned int instance)
{
	unsigned int i;

	for (i = 0; i < sizeof(mtk_codec_controls) / sizeof(mtk_codec_controls[0]); i++)
		if (mtk_codec_controls[i].legacy == id &&
		    (mtk_codec_controls[i].instances & instance))
			return mtk_codec_controls[i].native;
	/* A standard control can use the custom registration API too. */
	if (id < V4L2_CID_CODEC_MTK_BASE || id >= V4L2_CID_CODEC_MTK_BASE + 0x100)
		return id;
	return 0;
}

static inline __u32 mtk_codec_legacy_control(__u32 id, unsigned int instance)
{
	unsigned int i;

	if (id == V4L2_CID_MTK_VIDEO_CONTEXT_ID)
		return MTK_CODEC_CONTEXT_ID;
	for (i = 0; i < sizeof(mtk_codec_controls) / sizeof(mtk_codec_controls[0]); i++)
		if (mtk_codec_controls[i].native == id &&
		    (mtk_codec_controls[i].instances & instance))
			return mtk_codec_controls[i].legacy;
	/* Never let an unhandled native ID alias an unrelated legacy control. */
	if (id >= V4L2_CID_CODEC_MTK_BASE && id < V4L2_CID_CODEC_MTK_BASE + 0x300)
		return ~0U;
	return id; /* Standard V4L2 controls retain their ABI. */
}

/* The original 17 words are shared fields, including the raw HDR indicator.
 * Do not infer HDR/Dolby capability or change the indicator's bit values here.
 * The native-only reserved word must never expose uninitialized kernel data.
 */
static inline void mtk_codec_color_to_native(__u32 *native, const __u32 *firmware)
{
	unsigned int i;

	for (i = 0; i < 17; i++)
		native[i] = firmware[i];
	native[17] = 0;
}

static inline int mtk_codec_color_to_firmware(__u32 *firmware, const __u32 *native)
{
	unsigned int i;

	if (native[17])
		return -EINVAL;
	for (i = 0; i < 17; i++)
		firmware[i] = native[i];
	return 0;
}

static inline int mtk_codec_fixed_size_to_firmware(__u32 *size, const __u32 *native)
{
	/* Firmware has width/height only, with no modern buffer-mode field. */
	if (native[2])
		return -EOPNOTSUPP;
	if (native[0] > 65535 || native[1] > 65535)
		return -EINVAL;
	size[0] = native[0];
	size[1] = native[1];
	return 0;
}

static inline int mtk_codec_qp_to_firmware(int *qp, const int *native)
{
	/* This firmware has one limit, not independent I/P/B-frame limits. */
	if (native[0] != native[1] || native[0] != native[2])
		return -EOPNOTSUPP;
	if (native[0] < -1 || native[0] > 51)
		return -EINVAL;
	*qp = native[0];
	return 0;
}

#endif
