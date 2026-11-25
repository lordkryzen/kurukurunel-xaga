// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#include "gbe_sysfs.h"
#include "fpsgo_sysfs.h"

#define GBE_SYSFS_DIR_NAME "gbe"

static struct kobject *gbe_kobj;
// --------------------------------------------------
void gbe_sysfs_create_file(struct kobj_attribute *attribute)
{
	fpsgo_sysfs_create_file(gbe_kobj, attribute);
}
// --------------------------------------------------
void gbe_sysfs_remove_file(struct kobj_attribute *kobj_attr)
{
	if (kobj_attr == NULL || gbe_kobj == NULL)
		return;

	sysfs_remove_file(gbe_kobj, &(kobj_attr->attr));
}
// --------------------------------------------------
int gbe_sysfs_init(void)
{
	gbe_kobj = kobject_create_and_add(GBE_SYSFS_DIR_NAME,
			kernel_kobj);
	if (!gbe_kobj) {
		return -ENOMEM;
	}
	return 0;
}
// --------------------------------------------------
void gbe_sysfs_exit(void)
{
	kobject_put(gbe_kobj);
	gbe_kobj = NULL;
}
