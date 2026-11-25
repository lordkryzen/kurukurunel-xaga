/* SPDX-License-Identifier: GPL-2.0 */
#ifndef TEE_KERNEL_IOCTL_H
#define TEE_KERNEL_IOCTL_H
struct file;
long tee_kernel_ioctl(struct file *filp, unsigned int cmd, void *arg);
#endif
