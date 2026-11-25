// SPDX-License-Identifier: GPL-2.0
/* Kernel-only marshalling for the retained Microtrust400 transport.
 * No user-pointer entry point calls this path. Firmware records stay unchanged.
 */
#include <linux/fs.h>
#include <linux/slab.h>
#include <tee_drv.h>
#include "tee_private.h"
#include "tee_kernel_ioctl.h"

static void tee_kernel_put_params(struct tee_param *params, size_t count)
{
	size_t n;

	for (n = 0; n < count; n++)
		if (tee_param_is_memref(params + n) && params[n].u.memref.shm)
			isee_shm_put(params[n].u.memref.shm);
	kfree(params);
}

static int tee_kernel_get_params(struct tee_context *ctx,
		struct tee_param *params, size_t count,
		const struct tee_ioctl_param *input)
{
	size_t n;

	for (n = 0; n < count; n++) {
		const struct tee_ioctl_param *ip = input + n;
		struct tee_param *p = params + n;
		struct tee_shm *shm;

		if (ip->attr & ~TEE_IOCTL_PARAM_ATTR_MASK)
			return -EINVAL;
		p->attr = ip->attr;
		switch (ip->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_NONE:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			p->u.value.a = ip->a;
			p->u.value.b = ip->b;
			p->u.value.c = ip->c;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			/* IDs are positive signed ints; reject truncation of u64. */
			if (!ip->c || ip->c > INT_MAX)
				return -EINVAL;
			shm = isee_shm_get_from_id(ctx, ip->c);
			if (IS_ERR(shm))
				return PTR_ERR(shm);
			/* Publish the reference before any later failure. */
			p->u.memref.shm = shm;
			if (ip->a > shm->size || ip->b > shm->size - ip->a)
				return -EINVAL;
			p->u.memref.shm_offs = ip->a;
			p->u.memref.size = ip->b;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static void tee_kernel_output_params(struct tee_ioctl_param *output,
		size_t count, const struct tee_param *params)
{
	size_t n;

	for (n = 0; n < count; n++) {
		const struct tee_param *p = params + n;

		switch (p->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			output[n].a = p->u.value.a;
			output[n].b = p->u.value.b;
			output[n].c = p->u.value.c;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			output[n].b = p->u.memref.size;
			break;
		}
	}
}

static int tee_kernel_call(struct tee_context *ctx,
		const struct tee_ioctl_buf_data *buf, bool open)
{
	union {
		struct tee_ioctl_open_session_arg session;
		struct tee_ioctl_invoke_arg invoke;
	} arg;
	const struct tee_driver_ops *ops = ctx->teedev->desc->ops;
	size_t header = open ? sizeof(arg.session) : sizeof(arg.invoke);
	size_t count;
	void *data;
	struct tee_ioctl_param *wire;
	struct tee_param *params = NULL;
	int ret;

	if ((open && !ops->open_session) || (!open && !ops->invoke_func))
		return -EINVAL;
	if (!buf || !buf->buf_ptr || buf->buf_ptr > ULONG_MAX ||
	    buf->buf_len < header || buf->buf_len > TEE_MAX_ARG_SIZE ||
	    buf->buf_ptr > ULONG_MAX - buf->buf_len)
		return -EINVAL;
	data = (void *)(unsigned long)buf->buf_ptr;
	memcpy(&arg, data, header);
	count = open ? arg.session.num_params : arg.invoke.num_params;
	/* Subtract before multiplying: the header and count cannot wrap. */
	if (count != (buf->buf_len - header) / sizeof(*wire) ||
	    (buf->buf_len - header) % sizeof(*wire))
		return -EINVAL;
	wire = (void *)((char *)data + header);
	if (count) {
		params = kcalloc(count, sizeof(*params), GFP_KERNEL);
		if (!params)
			return -ENOMEM;
		ret = tee_kernel_get_params(ctx, params, count, wire);
		if (ret)
			goto out;
	}
	if (open)
		ret = ops->open_session(ctx, &arg.session, params);
	else
		ret = ops->invoke_func(ctx, &arg.invoke, params);
	if (!ret) {
		if (open) {
			struct tee_ioctl_open_session_arg *dst = data;

			dst->session = arg.session.session;
			dst->ret = arg.session.ret;
			dst->ret_origin = arg.session.ret_origin;
		} else {
			struct tee_ioctl_invoke_arg *dst = data;

			dst->ret = arg.invoke.ret;
			dst->ret_origin = arg.invoke.ret_origin;
		}
		tee_kernel_output_params(wire, count, params);
	}
out:
	if (params)
		tee_kernel_put_params(params, count);
	return ret;
}

long tee_kernel_ioctl(struct file *filp, unsigned int cmd, void *arg)
{
	struct tee_context *ctx;
	const struct tee_driver_ops *ops;

	if (!filp || !filp->private_data)
		return -EBADF;
	if (!arg)
		return -EINVAL;
	ctx = filp->private_data;
	ops = ctx->teedev->desc->ops;
	switch (cmd) {
	case TEE_IOC_OPEN_SESSION:
		return tee_kernel_call(ctx, arg, true);
	case TEE_IOC_INVOKE:
		return tee_kernel_call(ctx, arg, false);
	case TEE_IOC_CANCEL: {
		const struct tee_ioctl_cancel_arg *cancel = arg;

		return ops->cancel_req ? ops->cancel_req(ctx, cancel->cancel_id,
				cancel->session) : -EINVAL;
	}
	case TEE_IOC_CLOSE_SESSION: {
		const struct tee_ioctl_close_session_arg *close = arg;

		return ops->close_session ? ops->close_session(ctx,
				close->session) : -EINVAL;
	}
	case TEE_IOC_SET_HOSTNAME: {
		const struct tee_ioctl_set_hostname_arg *host = arg;

		memcpy(ctx->hostname, host->hostname, TEE_MAX_HOSTNAME_SIZE - 1);
		ctx->hostname[TEE_MAX_HOSTNAME_SIZE - 1] = 0;
		return 0;
	}
	default:
		/* Original400 has no SHM_REGISTER_FD handler either. */
		return -EINVAL;
	}
}
