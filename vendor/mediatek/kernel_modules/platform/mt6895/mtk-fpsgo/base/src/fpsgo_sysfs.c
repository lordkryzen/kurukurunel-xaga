// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 MediaTek Inc. */
#include "fpsgo_sysfs.h"
#include "fpsgo_base.h"

#define FPSGO_PENDING_FILES 256
static struct kobject *fpsgo_kobj;
static struct {
	struct kobject *parent;
	struct kobj_attribute *attribute;
} pending_files[FPSGO_PENDING_FILES];
static unsigned int pending_count;
static int sysfs_fault;
static bool sysfs_published;

int fpsgo_sysfs_create_dir(struct kobject *parent,
		const char *name, struct kobject **result)
{
	if (!name || !result) {
		sysfs_fault = -EINVAL;
		return sysfs_fault;
	}
	*result = NULL;
	parent = parent ? parent : fpsgo_kobj;
	if (!parent) {
		sysfs_fault = -EINVAL;
		return sysfs_fault;
	}
	*result = kobject_create_and_add(name, parent);
	if (!*result) {
		sysfs_fault = -ENOMEM;
		return sysfs_fault;
	}
	return 0;
}

void fpsgo_sysfs_remove_dir(struct kobject **object)
{
	if (object && *object) {
		kobject_put(*object);
		*object = NULL;
	}
}

void fpsgo_sysfs_create_file(struct kobject *parent,
		struct kobj_attribute *attribute)
{
	int ret;

	parent = parent ? parent : fpsgo_kobj;
	if (!parent || !attribute) {
		sysfs_fault = -EINVAL;
		return;
	}
	if (!sysfs_published) {
		if (pending_count == FPSGO_PENDING_FILES) {
			sysfs_fault = -ENOSPC;
			return;
		}
		pending_files[pending_count].parent = parent;
		pending_files[pending_count++].attribute = attribute;
		return;
	}
	ret = sysfs_create_file(parent, &attribute->attr);
	if (ret)
		sysfs_fault = ret;
}

/* All constructors queue controls; none can run against partially allocated
 * policy/workqueue state. Publish only after the complete module is ready. */
int fpsgo_sysfs_publish(void)
{
	unsigned int i;
	int ret;

	if (sysfs_fault)
		return sysfs_fault;
	if (sysfs_published)
		return 0;
	for (i = 0; i < pending_count; i++) {
		if (!pending_files[i].attribute)
			continue;
		ret = sysfs_create_file(pending_files[i].parent,
				&pending_files[i].attribute->attr);
		if (ret)
			goto rollback;
	}
	sysfs_published = true;
	return 0;
rollback:
	while (i--) {
		if (pending_files[i].attribute)
			sysfs_remove_file(pending_files[i].parent,
					&pending_files[i].attribute->attr);
	}
	sysfs_fault = ret;
	return ret;
}

void fpsgo_sysfs_remove_file(struct kobject *parent,
		struct kobj_attribute *attribute)
{
	unsigned int i;

	if (!attribute)
		return;
	parent = parent ? parent : fpsgo_kobj;
	if (!parent)
		return;
	if (!sysfs_published) {
		for (i = 0; i < pending_count; i++)
			if (pending_files[i].parent == parent &&
			    pending_files[i].attribute == attribute)
				pending_files[i].attribute = NULL;
		return;
	}
	sysfs_remove_file(parent, &attribute->attr);
}

int fpsgo_sysfs_init(void)
{
	if (fpsgo_kobj)
		return -EBUSY;
	fpsgo_kobj = kobject_create_and_add("fpsgo", kernel_kobj);
	return fpsgo_kobj ? 0 : -ENOMEM;
}

void fpsgo_sysfs_exit(void)
{
	if (fpsgo_kobj) {
		kobject_put(fpsgo_kobj);
		fpsgo_kobj = NULL;
	}
}
