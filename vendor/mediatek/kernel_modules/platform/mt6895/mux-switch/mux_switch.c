// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include "mux_switch.h"

#define PROC_TYPEC "mtk_typec"
#define PROC_MUX_SWITCH PROC_TYPEC "/mux_switch"
#define PROC_MUX "mux"
#define PROC_SWITCH "switch"

/* struct typec_mux_switch */
struct typec_mux_switch {
	struct device *dev;
	struct list_head list;
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
	int orientation;
	struct typec_mux_state state;
	struct proc_dir_entry *root;
};

/* struct mtk_typec_switch */
struct mtk_typec_switch {
	struct device *dev;
	struct typec_switch_dev *sw;
	typec_switch_set_fn_t set;
	struct module *owner;
	struct list_head list;
};

/* struct mtk_typec_mux */
struct mtk_typec_mux {
	struct device *dev;
	struct typec_mux_dev *mux;
	typec_mux_set_fn_t set;
	struct module *owner;
	struct list_head list;
};

static LIST_HEAD(mux_list);
static LIST_HEAD(switch_list);
static DEFINE_MUTEX(mux_lock);
static DEFINE_MUTEX(switch_lock);

/* Core consumer handles may survive unregister. Resolve an opaque handle
 * under this lock instead of dereferencing retired drvdata. Lock order is
 * frontend_lock, then switch_lock or mux_lock; callbacks cannot reenter.
 */
static LIST_HEAD(frontend_list);
static DEFINE_MUTEX(frontend_lock);

static struct typec_mux_switch *typec_frontend_find(const void *handle, bool mux)
{
	struct typec_mux_switch *frontend;

	list_for_each_entry(frontend, &frontend_list, list) {
		if ((mux && frontend->mux == handle) ||
		    (!mux && frontend->sw == handle))
			return frontend;
	}
	return NULL;
}

static void typec_frontend_unpublish(struct typec_mux_switch *frontend)
{
	mutex_lock(&frontend_lock);
	list_del(&frontend->list);
	mutex_unlock(&frontend_lock);
}

/* MUX */
struct typec_mux_dev *mtk_typec_mux_register(struct device *dev,
			const struct typec_mux_desc *desc)
{
	struct mtk_typec_mux *typec_mux;
	struct typec_mux_dev *mux;

	if (!dev || !dev->driver || !desc || !desc->set)
		return ERR_PTR(-EINVAL);

	mutex_lock(&mux_lock);
	list_for_each_entry(typec_mux, &mux_list, list) {
		if (typec_mux->dev == dev) {
			mux = ERR_PTR(-EEXIST);
			goto out;
		}
	}

	typec_mux = kzalloc(sizeof(*typec_mux), GFP_KERNEL);
	if (!typec_mux) {
		mux = ERR_PTR(-ENOMEM);
		goto out;
	}

	mux = typec_mux_register(dev, desc);
	if (IS_ERR(mux)) {
		kfree(typec_mux);
		goto out;
	}
	typec_mux->dev = dev;
	typec_mux->set = desc->set;
	typec_mux->owner = dev->driver->owner;
	typec_mux->mux = mux;
	list_add_tail(&typec_mux->list, &mux_list);
out:
	mutex_unlock(&mux_lock);
	return mux;
}
EXPORT_SYMBOL_GPL(mtk_typec_mux_register);

void mtk_typec_mux_unregister(struct typec_mux_dev *mux)
{
	struct mtk_typec_mux *typec_mux;

	if (IS_ERR_OR_NULL(mux))
		return;
	mutex_lock(&mux_lock);
	list_for_each_entry(typec_mux, &mux_list, list) {
		if (typec_mux->mux != mux)
			continue;
		list_del(&typec_mux->list);
		typec_mux_unregister(mux);
		kfree(typec_mux);
		break;
	}
	mutex_unlock(&mux_lock);
}
EXPORT_SYMBOL_GPL(mtk_typec_mux_unregister);

static int mtk_typec_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct typec_mux_switch *mux_sw;
	struct mtk_typec_mux *typec_mux;
	int ret = 0;

	if (!state)
		return -EINVAL;
	mutex_lock(&frontend_lock);
	mux_sw = typec_frontend_find(mux, true);
	if (!mux_sw) {
		ret = -ENODEV;
		goto out_frontend;
	}
	dev_info(mux_sw->dev, "%s %lu %lu\n", __func__,
		 mux_sw->state.mode, state->mode);

	mutex_lock(&mux_lock);

	list_for_each_entry(typec_mux, &mux_list, list) {
		if (!try_module_get(typec_mux->owner)) {
			ret = -ENODEV;
			break;
		}
		ret = typec_mux->set(typec_mux->mux, state);
		module_put(typec_mux->owner);
		if (ret)
			break;
	}

	if (!ret)
		mux_sw->state.mode = state->mode;

	mutex_unlock(&mux_lock);

out_frontend:
	mutex_unlock(&frontend_lock);
	return ret;
}

static int proc_mux_show(struct seq_file *s, void *unused)
{
	struct typec_mux_switch *mux_sw = s->private;

	seq_printf(s, "%lu\n", mux_sw->state.mode);
	return 0;
}

static int proc_mux_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_mux_show, pde_data(inode));
}

static ssize_t proc_mux_write(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct typec_mux_switch *mux_sw = s->private;
	struct typec_mux_state state = {};
	char buf[20];
	u32 tmp;
	int ret;

	memset(buf, 0x00, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (kstrtouint(buf, 0, &tmp))
		return -EINVAL;

	if (tmp > 2) {
		dev_info(mux_sw->dev, "%s %lu, INVALID: %u\n", __func__,
			 mux_sw->state.mode, tmp);
		return count;
	}

	state.mode = tmp;
	ret = mtk_typec_mux_set(mux_sw->mux, &state);

	return ret ? ret : count;
}

static const struct proc_ops proc_mux_fops = {
	.proc_open = proc_mux_open,
	.proc_write = proc_mux_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

/* SWITCH */
struct typec_switch_dev *mtk_typec_switch_register(struct device *dev,
			const struct typec_switch_desc *desc)
{
	struct mtk_typec_switch *typec_sw;
	struct typec_switch_dev *sw;

	if (!dev || !dev->driver || !desc || !desc->set)
		return ERR_PTR(-EINVAL);

	mutex_lock(&switch_lock);
	list_for_each_entry(typec_sw, &switch_list, list) {
		if (typec_sw->dev == dev) {
			sw = ERR_PTR(-EEXIST);
			goto out;
		}
	}

	typec_sw = kzalloc(sizeof(*typec_sw), GFP_KERNEL);
	if (!typec_sw) {
		sw = ERR_PTR(-ENOMEM);
		goto out;
	}

	sw = typec_switch_register(dev, desc);
	if (IS_ERR(sw)) {
		kfree(typec_sw);
		goto out;
	}
	typec_sw->dev = dev;
	typec_sw->set = desc->set;
	typec_sw->owner = dev->driver->owner;
	typec_sw->sw = sw;
	list_add_tail(&typec_sw->list, &switch_list);
out:
	mutex_unlock(&switch_lock);
	return sw;
}
EXPORT_SYMBOL_GPL(mtk_typec_switch_register);

void mtk_typec_switch_unregister(struct typec_switch_dev *sw)
{
	struct mtk_typec_switch *typec_sw;

	if (IS_ERR_OR_NULL(sw))
		return;
	mutex_lock(&switch_lock);
	list_for_each_entry(typec_sw, &switch_list, list) {
		if (typec_sw->sw != sw)
			continue;
		list_del(&typec_sw->list);
		typec_switch_unregister(sw);
		kfree(typec_sw);
		break;
	}
	mutex_unlock(&switch_lock);
}
EXPORT_SYMBOL_GPL(mtk_typec_switch_unregister);

static int mtk_typec_switch_set(struct typec_switch_dev *sw,
			      enum typec_orientation orientation)
{
	struct typec_mux_switch *mux_sw;
	struct device_node *np;
	struct mtk_typec_switch *typec_sw;
	int ret = 0;
	enum typec_orientation requested = orientation;

	if (orientation < TYPEC_ORIENTATION_NONE || orientation > TYPEC_ORIENTATION_REVERSE)
		return -EINVAL;
	mutex_lock(&frontend_lock);
	mux_sw = typec_frontend_find(sw, false);
	if (!mux_sw) {
		ret = -ENODEV;
		goto out_frontend;
	}
	np = mux_sw->dev->of_node;
	dev_info(mux_sw->dev, "%s %d %d\n", __func__,
		 mux_sw->orientation, orientation);

	mutex_lock(&switch_lock);
	if(of_property_read_bool(np, "mediatek,inverse-orientation")) {
		switch (orientation) {
		case TYPEC_ORIENTATION_NONE:
			break;
		case TYPEC_ORIENTATION_NORMAL:
			orientation = TYPEC_ORIENTATION_REVERSE;
			break;
		case TYPEC_ORIENTATION_REVERSE:
			orientation = TYPEC_ORIENTATION_NORMAL;
			break;
		default:
			break;
		}
		dev_info(mux_sw->dev, "%s inverse orientation %d\n", __func__, orientation);
	}

	list_for_each_entry(typec_sw, &switch_list, list) {
		if (!try_module_get(typec_sw->owner)) {
			ret = -ENODEV;
			break;
		}
		ret = typec_sw->set(typec_sw->sw, orientation);
		module_put(typec_sw->owner);
		if (ret)
			break;
	}

	if (!ret)
		mux_sw->orientation = requested;

	mutex_unlock(&switch_lock);

out_frontend:
	mutex_unlock(&frontend_lock);
	return ret;
}

static int proc_switch_show(struct seq_file *s, void *unused)
{
	struct typec_mux_switch *mux_sw = s->private;

	switch (mux_sw->orientation) {
	case TYPEC_ORIENTATION_NONE:
		seq_puts(s, "NONE\n");
		break;
	case TYPEC_ORIENTATION_NORMAL:
		seq_puts(s, "NORMAL\n");
		break;
	case TYPEC_ORIENTATION_REVERSE:
		seq_puts(s, "REVERSE\n");
		break;
	default:
		seq_puts(s, "INVALID\n");
	}

	seq_printf(s, "%d\n", mux_sw->orientation);
	return 0;
}

static int proc_switch_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_switch_show, pde_data(inode));
}

static ssize_t proc_switch_write(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct typec_mux_switch *mux_sw = s->private;
	char buf[20];
	u32 tmp;
	int ret;

	memset(buf, 0x00, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (kstrtouint(buf, 0, &tmp))
		return -EINVAL;

	if (tmp > 2) {
		dev_info(mux_sw->dev, "%s %d, INVALID: %d\n", __func__,
			 mux_sw->orientation, tmp);
		return count;
	}

	ret = mtk_typec_switch_set(mux_sw->sw, tmp);

	return ret ? ret : count;
}

static const struct proc_ops proc_switch_fops = {
	.proc_open = proc_switch_open,
	.proc_write = proc_switch_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int typec_mux_switch_procfs_init(struct typec_mux_switch *mux_sw)
{
	struct device *dev = mux_sw->dev;
	struct proc_dir_entry *root;
	struct proc_dir_entry *file;
	int ret = 0;

	proc_mkdir(PROC_TYPEC, NULL);

	root = proc_mkdir(PROC_MUX_SWITCH, NULL);
	if (!root) {
		dev_info(dev, "failed to creat %s dir\n", PROC_MUX_SWITCH);
		return -ENOMEM;
	}

	file = proc_create_data(PROC_SWITCH, 0640,
			root, &proc_switch_fops, mux_sw);
	if (!file) {
		dev_info(dev, "failed to creat proc file: %s\n", PROC_SWITCH);
		ret = -ENOMEM;
		goto err;
	}

	file = proc_create_data(PROC_MUX, 0640,
			root, &proc_mux_fops, mux_sw);
	if (!file) {
		dev_info(dev, "failed to creat proc file: %s\n", PROC_MUX);
		ret = -ENOMEM;
		goto err;
	}

	mux_sw->root = root;
	return 0;

err:
	proc_remove(root);
	return ret;
}

static int typec_mux_switch_procfs_exit(struct typec_mux_switch *mux_sw)
{
	proc_remove(mux_sw->root);
	return 0;
}

/* The graph identifies the TCPC consumer, not the MMIO selector supplier.
 * Require an explicit firmware link before exposing either Type-C handle.
 * A successful selector probe has published both callback registrations.
 */
static int typec_frontend_require_selector(struct device *dev)
{
	struct device_node *node;
	struct platform_device *selector;
	int ret;

	if (of_property_count_u32_elems(dev->of_node, "mediatek,usb-dp-selector") != 1)
		return -EINVAL;
	node = of_parse_phandle(dev->of_node, "mediatek,usb-dp-selector", 0);
	if (!node)
		return -EINVAL;
	if (!of_device_is_available(node) ||
	    !of_device_is_compatible(node, "mediatek,usb_dp_selector")) {
		of_node_put(node);
		return -EINVAL;
	}
	selector = of_find_device_by_node(node);
	of_node_put(node);
	if (!selector)
		return -EPROBE_DEFER;
	if (&selector->dev == dev) {
		ret = -EINVAL;
		goto put_selector;
	}
	/* Never block on another device lock while this consumer is probing:
	 * supplier removal may be waiting for our probe to finish.
	 */
	if (!device_trylock(&selector->dev)) {
		ret = -EPROBE_DEFER;
		goto put_selector;
	}
	if (!device_is_bound(&selector->dev)) {
		ret = -EPROBE_DEFER;
	} else if (!device_link_add(dev, &selector->dev,
				    DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
	} else {
		ret = 0;
	}
	device_unlock(&selector->dev);
put_selector:
	put_device(&selector->dev);
	return ret;
}

static int typec_mux_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct typec_mux_switch *mux_sw;
	struct typec_switch_desc sw_desc = {};
	struct typec_mux_desc mux_desc = {};
	int ret;

	ret = typec_frontend_require_selector(dev);
	if (ret)
		return ret;

	mux_sw = kzalloc(sizeof(*mux_sw), GFP_KERNEL);
	if (!mux_sw)
		return -ENOMEM;
	mux_sw->dev = dev;
	INIT_LIST_HEAD(&mux_sw->list);
	sw_desc.drvdata = mux_sw;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = mtk_typec_switch_set;
	/* Core registration publishes handles before returning. Block external
	 * setters until both handles and the frontend mapping are committed.
	 * Public core register functions do not invoke descriptor callbacks.
	 */
	mutex_lock(&frontend_lock);
	mux_sw->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(mux_sw->sw)) {
		ret = PTR_ERR(mux_sw->sw);
		mutex_unlock(&frontend_lock);
		goto free_frontend;
	}
	mux_desc.drvdata = mux_sw;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = mtk_typec_mux_set;
	mux_sw->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(mux_sw->mux)) {
		ret = PTR_ERR(mux_sw->mux);
		mutex_unlock(&frontend_lock);
		goto unregister_switch;
	}
	list_add_tail(&mux_sw->list, &frontend_list);
	mutex_unlock(&frontend_lock);
	ret = typec_mux_switch_procfs_init(mux_sw);
	if (ret) {
		typec_frontend_unpublish(mux_sw);
		typec_mux_unregister(mux_sw->mux);
		goto unregister_switch;
	}
	platform_set_drvdata(pdev, mux_sw);
	return 0;
unregister_switch:
	typec_switch_unregister(mux_sw->sw);
free_frontend:
	kfree(mux_sw);
	return ret;
}

static void typec_mux_switch_remove(struct platform_device *pdev)
{
	struct typec_mux_switch *mux_sw = dev_get_drvdata(&pdev->dev);

	if (!mux_sw)
		return;
	/* proc_remove drains active proc operations before private data retires. */
	typec_mux_switch_procfs_exit(mux_sw);
	typec_frontend_unpublish(mux_sw);
	typec_mux_unregister(mux_sw->mux);
	typec_switch_unregister(mux_sw->sw);
	platform_set_drvdata(pdev, NULL);
	kfree(mux_sw);
}

static const struct of_device_id typec_mux_switch_ids[] = {
	{.compatible = "mediatek,typec_mux_switch",},
	{.compatible = "mediatek,typec_switch",},
	{},
};

MODULE_DEVICE_TABLE(of, typec_mux_switch_ids);

static struct platform_driver typec_mux_switch_driver = {
	.probe = typec_mux_switch_probe,
	.remove = typec_mux_switch_remove,
	.driver = {
		.name = "mtk-typec-mux-switch",
		.of_match_table = typec_mux_switch_ids,
	},
};

module_platform_driver(typec_mux_switch_driver);

MODULE_DESCRIPTION("Mediatek Type-C mux switch driver");
MODULE_LICENSE("GPL");
