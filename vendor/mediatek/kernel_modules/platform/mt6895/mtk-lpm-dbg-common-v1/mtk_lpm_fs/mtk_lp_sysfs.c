// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "mtk_lp_sysfs.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

DEFINE_MUTEX(mtk_lp_sysfs_locker);
static LIST_HEAD(mtk_lp_sysfs_parent);
static int publication_error;
void mtk_lp_sysfs_error_reset(void){WRITE_ONCE(publication_error,0);}
int mtk_lp_sysfs_error_get(void){return READ_ONCE(publication_error);}
EXPORT_SYMBOL_GPL(mtk_lp_sysfs_error_reset);
EXPORT_SYMBOL_GPL(mtk_lp_sysfs_error_get);


int __weak mtk_lp_sysfs_entry_create_plat(const char *name,
		int mode, struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *handle)
{
	return 0;
}
int __weak mtk_lp_sysfs_entry_node_add_plat(const char *name,
		int mode, const struct mtk_lp_sysfs_op *op,
		struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *handle)
{
	return 0;
}

int __weak mtk_lp_sysfs_entry_node_remove_plat(
		struct mtk_lp_sysfs_handle *node)
{
	return 0;
}

int __weak mtk_lp_sysfs_entry_group_create_plat(const char *name,
		int mode, struct mtk_lp_sysfs_group *_group,
		struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *handle)
{
	return 0;
}

static
int __mtk_lp_sysfs_handle_add(struct mtk_lp_sysfs_handle *parent,
				      struct mtk_lp_sysfs_handle *node)
{
	if (!node)
		return -EINVAL;

	INIT_LIST_HEAD(&node->dr);
	INIT_LIST_HEAD(&node->np);

	if (parent)
		list_add(&node->np, &parent->dr);
	else
		list_add(&node->np, &mtk_lp_sysfs_parent);
	return 0;
}

int mtk_lp_sysfs_entry_func_create(const char *name,
		int mode, struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *handle)
{
	int bRet;
	struct mtk_lp_sysfs_handle *p = NULL;

	mutex_lock(&mtk_lp_sysfs_locker);

	do {
		if (!handle) {
			p = kzalloc(sizeof(*p), GFP_KERNEL);
			if (!p) {
				bRet = -ENOMEM;
				break;
			}
			p->flag |= MTK_LP_SYSFS_FREEZABLE;
		} else
			p = handle;

		bRet = mtk_lp_sysfs_entry_create_plat(name,
					mode, parent, p);

		if (!bRet) {
			p->flag |= MTK_LP_SYSFS_TYPE_ENTRY;
			p->name = name;
			__mtk_lp_sysfs_handle_add(parent, p);
		} else if(p->flag & MTK_LP_SYSFS_FREEZABLE)
			kfree(p);
	} while (0);
	if(bRet)WRITE_ONCE(publication_error,bRet);
 mutex_unlock(&mtk_lp_sysfs_locker);
	return bRet;
}
EXPORT_SYMBOL(mtk_lp_sysfs_entry_func_create);

int mtk_lp_sysfs_entry_func_node_add(const char *name,
		int mode, const struct mtk_lp_sysfs_op *op,
		struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *node)
{
	int bRet;
	struct mtk_lp_sysfs_handle *p = NULL;

	mutex_lock(&mtk_lp_sysfs_locker);

	do {
		if (!node) {
			p = kzalloc(sizeof(*p), GFP_KERNEL);
			if (!p) {
				bRet = -ENOMEM;
				break;
			}
			p->flag |= MTK_LP_SYSFS_FREEZABLE;
		} else
			p = node;

		bRet = mtk_lp_sysfs_entry_node_add_plat(name,
					mode, op, parent, p);
		if (!bRet) {
			p->flag &= ~MTK_LP_SYSFS_TYPE_ENTRY;
			p->name = name;
			__mtk_lp_sysfs_handle_add(parent, p);
		} else if(p->flag & MTK_LP_SYSFS_FREEZABLE)
			kfree(p);
	} while (0);

	if(bRet)WRITE_ONCE(publication_error,bRet);
 mutex_unlock(&mtk_lp_sysfs_locker);
	return bRet;
}
EXPORT_SYMBOL(mtk_lp_sysfs_entry_func_node_add);

static
int __mtk_lp_sysfs_handle_remove(struct mtk_lp_sysfs_handle *node)
{
	int ret = 0;

	if (!node)
		return -EINVAL;

	ret = mtk_lp_sysfs_entry_node_remove_plat(node);

	if (!ret) {
		node->_current=NULL;
		list_del(&node->np);
		INIT_LIST_HEAD(&node->np);
		if (node->flag & MTK_LP_SYSFS_FREEZABLE)
			kfree(node);
	}
	return ret;
}

static int __mtk_lp_sysfs_entry_rm(struct mtk_lp_sysfs_handle *node)
{
 struct mtk_lp_sysfs_handle *cur,*next;int ret;
 list_for_each_entry_safe(cur,next,&node->dr,np){
  if(cur->flag&MTK_LP_SYSFS_TYPE_ENTRY){ret=__mtk_lp_sysfs_entry_rm(cur);if(ret)return ret;}
  ret=__mtk_lp_sysfs_handle_remove(cur);if(ret)return ret;
 }
 return 0;
}

int mtk_lp_sysfs_entry_func_node_remove(
		struct mtk_lp_sysfs_handle *node)
{
	int bRet = 0;

	if (!node)
		return -EINVAL;

	mutex_lock(&mtk_lp_sysfs_locker);
	if (node->flag & MTK_LP_SYSFS_TYPE_ENTRY)
		__mtk_lp_sysfs_entry_rm(node);
	__mtk_lp_sysfs_handle_remove(node);
	if(bRet)WRITE_ONCE(publication_error,bRet);
 mutex_unlock(&mtk_lp_sysfs_locker);
	return bRet;
}
EXPORT_SYMBOL(mtk_lp_sysfs_entry_func_node_remove);

int mtk_lp_sysfs_entry_func_group_create(const char *name,
		int mode, struct mtk_lp_sysfs_group *_group,
		struct mtk_lp_sysfs_handle *parent,
		struct mtk_lp_sysfs_handle *handle)
{
	int bRet;

	mutex_lock(&mtk_lp_sysfs_locker);
	bRet = mtk_lp_sysfs_entry_group_create_plat(name
			, mode, _group, parent, handle);
	if(bRet)WRITE_ONCE(publication_error,bRet);
 mutex_unlock(&mtk_lp_sysfs_locker);
	return bRet;
}
EXPORT_SYMBOL(mtk_lp_sysfs_entry_func_group_create);

void mtk_lp_sysfs_remove_all(void)
{
 struct mtk_lp_sysfs_handle *cur,*next;
 extern int mtk_lp_proc_root_put(void);
 mutex_lock(&mtk_lp_sysfs_locker);
 list_for_each_entry_safe(cur,next,&mtk_lp_sysfs_parent,np){
  if(cur->flag&MTK_LP_SYSFS_TYPE_ENTRY)__mtk_lp_sysfs_entry_rm(cur);
  __mtk_lp_sysfs_handle_remove(cur);
 }
 mtk_lp_proc_root_put();mutex_unlock(&mtk_lp_sysfs_locker);
}
EXPORT_SYMBOL_GPL(mtk_lp_sysfs_remove_all);
