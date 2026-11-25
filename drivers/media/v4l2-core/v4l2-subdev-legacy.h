/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __V4L2_SUBDEV_LEGACY_H__
#define __V4L2_SUBDEV_LEGACY_H__

/*
 * The original xaga HAL uses the pre-streams reserved tail for request FDs
 * and metadata pointer halves. Translate only at the opted-in ioctl boundary;
 * internal pad operations always receive native stream/which fields.
 * Run in the normal sleepable ioctl context: sensor callbacks may wait for I2C.
 */
static inline int v4l2_subdev_legacy_set_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state, struct v4l2_subdev_format *format)
{
	u32 tail[8];
	u8 *wire = (u8 *)format + offsetof(struct v4l2_subdev_format, stream);
	int ret;

	might_sleep();

	static_assert(sizeof(*format) - offsetof(struct v4l2_subdev_format, stream) == sizeof(tail));
	memcpy(tail, wire, sizeof(tail));
	format->stream = 0;
	memcpy(format->reserved, tail, sizeof(format->reserved));
	ret = v4l2_subdev_call(sd, pad, set_fmt, state, format);
	memcpy(tail, format->reserved, sizeof(format->reserved));
	memcpy(wire, tail, sizeof(tail));
	return ret;
}

static inline int v4l2_subdev_legacy_set_selection(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
	u32 tail[8];
	u8 *wire = (u8 *)sel + offsetof(struct v4l2_subdev_selection, stream);
	int ret;

	might_sleep();

	static_assert(sizeof(*sel) - offsetof(struct v4l2_subdev_selection, stream) == sizeof(tail));
	memcpy(tail, wire, sizeof(tail));
	sel->stream = 0;
	memcpy(sel->reserved, tail, sizeof(sel->reserved));
	ret = v4l2_subdev_call(sd, pad, set_selection, state, sel);
	memcpy(tail, sel->reserved, sizeof(sel->reserved));
	memcpy(wire, tail, sizeof(tail));
	return ret;
}

static inline int v4l2_subdev_legacy_set_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	u32 tail[9];
	u8 *wire = (u8 *)fi + offsetof(struct v4l2_subdev_frame_interval, stream);
	int ret;

	might_sleep();

	static_assert(sizeof(*fi) - offsetof(struct v4l2_subdev_frame_interval, stream) == sizeof(tail));
	memcpy(tail, wire, sizeof(tail));
	fi->stream = 0;
	fi->which = tail[0];
	if ((fi->which != V4L2_SUBDEV_FORMAT_TRY &&
	     fi->which != V4L2_SUBDEV_FORMAT_ACTIVE) ||
	    (sd->entity.num_pads ? fi->pad >= sd->entity.num_pads : fi->pad > 0))
		ret = -EINVAL;
	else if (!sd->ops || !sd->ops->pad || !sd->ops->pad->set_frame_interval)
		ret = -ENOIOCTLCMD;
	else
		/* Legacy TRY lives in the ISP's try_res_config, not pad state. */
		ret = sd->ops->pad->set_frame_interval(sd, NULL, fi);
	tail[0] = fi->which;
	memcpy(wire, tail, sizeof(tail));
	return ret;
}
#endif
