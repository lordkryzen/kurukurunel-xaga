/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 MediaTek Inc.
 */

#ifndef __WIDEVINE_DRIVER_H__
#define __WIDEVINE_DRIVER_H__

#include <linux/printk.h>

extern int wv_dbg_level;

#define WV_LOG(level, fmt, args...)				\
	do {							\
		if ((wv_dbg_level & level) == level)		\
			pr_info("[drm_kernel] level=%d %s(), %d: " fmt "\n",\
			level, __func__, __LINE__, ##args);	\
	} while (0)

struct WV_FD_TO_SEC_HANDLE {
	int share_fd;
	uint64_t sec_handle;
};

#define WV_DEVNAME "drm_wv"
#define WV_IOC_MAGIC 'W'

#define WV_IOC_GET_SEC_HANDLE  _IOR(WV_IOC_MAGIC, 1, struct WV_FD_TO_SEC_HANDLE)
/* xaga 5.10 userspace uses an 8-byte argument and a distinct ioctl number. */
struct WV_FD_TO_SEC_HANDLE32 {
 int share_fd;
 uint32_t sec_handle;
};
#define WV_IOC_GET_SEC_HANDLE32 _IOR(WV_IOC_MAGIC, 1, struct WV_FD_TO_SEC_HANDLE32)
#define WV_IOC_MACNR 2

#endif /* __WIDEVINE_DRIVER_H__ */
