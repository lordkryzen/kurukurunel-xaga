/* Translate the public HEVC fourcc at the ioctl boundary only. Firmware and
 * internal format tables retain their original H265 contract.
 */
#ifndef __MTK_VCODEC_FORMAT_BRIDGE_H__
#define __MTK_VCODEC_FORMAT_BRIDGE_H__
static inline __u32 mtk_codec_format_to_firmware(__u32 format)
{
	return IS_ENABLED(CONFIG_VIDEO_MEDIATEK_NATIVE_UAPI) &&
		format == V4L2_PIX_FMT_HEVC ? V4L2_PIX_FMT_H265 : format;
}

static inline __u32 mtk_codec_format_to_native(__u32 format, __u32 requested)
{
	/* Explicit legacy callers can still request H265. */
	return IS_ENABLED(CONFIG_VIDEO_MEDIATEK_NATIVE_UAPI) &&
		format == V4L2_PIX_FMT_H265 && requested != V4L2_PIX_FMT_H265 ?
		V4L2_PIX_FMT_HEVC : format;
}

#define MTK_CODEC_FORMAT_WRAPPER(name, handler) \
static int name(struct file *file, void *priv, struct v4l2_format *format) \
{ \
	__u32 requested = format->fmt.pix_mp.pixelformat; \
	int ret; \
	format->fmt.pix_mp.pixelformat = mtk_codec_format_to_firmware(requested); \
	ret = handler(file, priv, format); \
	format->fmt.pix_mp.pixelformat = mtk_codec_format_to_native( \
		format->fmt.pix_mp.pixelformat, requested); \
	return ret; \
}

#define MTK_CODEC_ENUM_WRAPPER(name, handler) \
static int name(struct file *file, void *priv, struct v4l2_fmtdesc *format) \
{ \
	int ret = handler(file, priv, format); \
	if (!ret) format->pixelformat = mtk_codec_format_to_native(format->pixelformat, 0); \
	return ret; \
}

#define MTK_CODEC_SIZE_WRAPPER(name, handler) \
static int name(struct file *file, void *priv, struct v4l2_frmsizeenum *sizes) \
{ \
	__u32 requested = sizes->pixel_format; \
	int ret; \
	sizes->pixel_format = mtk_codec_format_to_firmware(requested); \
	ret = handler(file, priv, sizes); \
	sizes->pixel_format = requested; \
	return ret; \
}

#define MTK_CODEC_CREATE_WRAPPER(name, handler) \
static int name(struct file *file, void *priv, struct v4l2_create_buffers *buffers) \
{ \
	__u32 requested = buffers->format.fmt.pix_mp.pixelformat; \
	int ret; \
	buffers->format.fmt.pix_mp.pixelformat = mtk_codec_format_to_firmware(requested); \
	ret = handler(file, priv, buffers); \
	buffers->format.fmt.pix_mp.pixelformat = mtk_codec_format_to_native( \
		buffers->format.fmt.pix_mp.pixelformat, requested); \
	return ret; \
}
#endif
