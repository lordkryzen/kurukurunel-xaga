/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_NATIVE_CTRLS_H_
#define _MTK_VCODEC_NATIVE_CTRLS_H_

#include "mtk_vcodec_control_bridge.h"

static inline u32 mtk_codec_control_id(u32 id, unsigned int instance)
{
	return IS_ENABLED(CONFIG_VIDEO_MEDIATEK_NATIVE_UAPI) ?
		mtk_codec_legacy_control(id, instance) : id;
}

static inline void mtk_codec_new_custom(struct v4l2_ctrl_handler *handler,
				       const struct v4l2_ctrl_config *old,
				       unsigned int instance)
{
	struct v4l2_ctrl_config native = *old;

	if (!IS_ENABLED(CONFIG_VIDEO_MEDIATEK_NATIVE_UAPI)) {
		v4l2_ctrl_new_custom(handler, old, NULL);
		return;
	}
	native.id = mtk_codec_native_control(old->id, instance);
	if (!native.id)
		return; /* No native equivalent; do not expose a conflicting old ID. */
	switch (native.id) {
	case V4L2_CID_MTK_VIDEO_COLOR_DESC:
		native.dims[0] = 18;
		break;
	case V4L2_CID_MTK_VIDEO_DEC_FIXED_MAX_FRAME_BUFFER:
		native.type = V4L2_CTRL_TYPE_U32;
		native.dims[0] = 3;
		break;
	case V4L2_CID_MTK_VIDEO_ENC_RC_MAX_QP:
	case V4L2_CID_MTK_VIDEO_ENC_RC_MIN_QP:
		native.dims[0] = 3;
		break;
	}
	/* Keep the real device's ranges, rather than advertising donor maxima. */
	v4l2_ctrl_new_custom(handler, &native, NULL);
}

static inline void mtk_codec_add_context_control(struct v4l2_ctrl_handler *handler,
					       const struct v4l2_ctrl_ops *ops)
{
	const struct v4l2_ctrl_config config = {
		.id = V4L2_CID_MTK_VIDEO_CONTEXT_ID,
		.name = "Video codec context ID",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.max = S32_MAX,
		.step = 1,
		.ops = ops,
	};

	if (IS_ENABLED(CONFIG_VIDEO_MEDIATEK_NATIVE_UAPI))
		v4l2_ctrl_new_custom(handler, &config, NULL);
}

#endif
