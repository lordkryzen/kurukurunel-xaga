// SPDX-License-Identifier: GPL-2.0
/* Xaga TEE performance interface, using owned Linux frequency QoS votes. */
#include <linux/cdev.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define TEEPERF_IO_HIGH_FREQ _IO('T', 0)
#define TEEPERF_CPU_COUNT 5

struct teeperf_vote {
	struct cpufreq_policy *policy;
	struct freq_qos_request request;
	unsigned int floor;
};

/* Static control state outlives open descriptors; each open records its epoch. */
static DEFINE_MUTEX(teeperf_lock);
static struct platform_device *teeperf_owner;
static unsigned long teeperf_epoch;
static bool teeperf_ready;
static dev_t teeperf_devt;
static struct class *teeperf_class;
static struct cdev *teeperf_cdev;
static struct proc_dir_entry *teeperf_proc;
static struct teeperf_vote teeperf_votes[TEEPERF_CPU_COUNT];
static unsigned int teeperf_vote_count;
static unsigned long teeperf_target_mask;

static void teeperf_drop_votes(void)
{
	while (teeperf_vote_count) {
		struct teeperf_vote *vote = &teeperf_votes[--teeperf_vote_count];

		freq_qos_remove_request(&vote->request);
		cpufreq_cpu_put(vote->policy);
		vote->policy = NULL;
	}
	teeperf_target_mask = 0;
}

static unsigned long teeperf_online_targets(void)
{
	unsigned long mask = 0;
	unsigned int slot;

	for (slot = 0; slot < TEEPERF_CPU_COUNT; slot++) {
		unsigned int cpu = slot == 4 ? 6 : slot;

		if (cpu_online(cpu))
			mask |= BIT(cpu);
	}
	return mask;
}

/* Preserve original xaga raw table selection, including its CPU6 TEE vote. */
static int teeperf_policy_floor(struct cpufreq_policy *policy, unsigned int cpu,
			       unsigned int *floor)
{
	struct cpufreq_frequency_table *entry;
	unsigned int desired = cpu == 6 ? 23 : 0;
	unsigned int last = 0, index;
	bool found = false;
	int ret = -EINVAL;

	down_read(&policy->rwsem);
	if (!policy->freq_table ||
	    policy->freq_table_sorted != CPUFREQ_TABLE_SORTED_DESCENDING)
		goto out;
	cpufreq_for_each_valid_entry_idx(entry, policy->freq_table, index) {
		last = entry->frequency;
		found = true;
		if (index >= desired)
			break;
	}
	if (found) {
		*floor = last;
		ret = 0;
	}
out:
	up_read(&policy->rwsem);
	return ret;
}

/* Caller holds teeperf_lock. Build all policy references before changing votes. */
static int teeperf_prepare_votes(void)
{
	unsigned int slot, i;
	int ret;

	if (teeperf_vote_count)
		return 0;
	for (slot = 0; slot < TEEPERF_CPU_COUNT; slot++) {
		unsigned int cpu = slot == 4 ? 6 : slot;
		unsigned int floor;
		struct cpufreq_policy *policy;
		struct teeperf_vote *vote;

		if (!cpu_online(cpu))
			continue;
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			ret = -EPROBE_DEFER;
			goto fail;
		}
		ret = teeperf_policy_floor(policy, cpu, &floor);
		if (ret) {
			cpufreq_cpu_put(policy);
			goto fail;
		}
		for (i = 0; i < teeperf_vote_count; i++)
			if (teeperf_votes[i].policy == policy)
				break;
		if (i < teeperf_vote_count) {
			teeperf_votes[i].floor = max(teeperf_votes[i].floor, floor);
			cpufreq_cpu_put(policy);
			continue;
		}
		vote = &teeperf_votes[teeperf_vote_count];
		vote->policy = policy;
		vote->floor = floor;
		ret = freq_qos_add_request(&policy->constraints, &vote->request,
					   FREQ_QOS_MIN, FREQ_QOS_MIN_DEFAULT_VALUE);
		if (ret < 0) {
			cpufreq_cpu_put(policy);
			vote->policy = NULL;
			goto fail;
		}
		teeperf_vote_count++;
	}
	teeperf_target_mask = teeperf_online_targets();
	return 0;
fail:
	teeperf_drop_votes();
	return ret;
}

static int teeperf_set_boost(bool enable)
{
	unsigned int i;
	int ret;

	if (!teeperf_ready)
		return -ENODEV;
	if (!enable) {
		teeperf_drop_votes();
		return 0;
	}
	if (teeperf_target_mask != teeperf_online_targets())
		teeperf_drop_votes();
	ret = teeperf_prepare_votes();
	if (ret)
		return ret;
	for (i = 0; i < teeperf_vote_count; i++) {
		struct teeperf_vote *vote = &teeperf_votes[i];

		ret = freq_qos_update_request(&vote->request, enable ?
			vote->floor : FREQ_QOS_MIN_DEFAULT_VALUE);
		if (ret < 0) {
			/* Drop all our votes on failure; never leave a partial boost. */
			teeperf_drop_votes();
			return ret;
		}
	}
	return 0;
}

static int teeperf_open(struct inode *inode, struct file *file)
{
	unsigned long *epoch = kmalloc(sizeof(*epoch), GFP_KERNEL);
	int ret = 0;

	if (!epoch)
		return -ENOMEM;
	mutex_lock(&teeperf_lock);
	if (!teeperf_ready) {
		ret = -ENODEV;
	} else {
		*epoch = teeperf_epoch;
		file->private_data = epoch;
	}
	mutex_unlock(&teeperf_lock);
	if (ret)
		kfree(epoch);
	return ret;
}

static int teeperf_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static long teeperf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 enable;
	int ret;

	if (cmd != TEEPERF_IO_HIGH_FREQ)
		return -ENOIOCTLCMD;
	/* Preserve the existing _IO command with its actual u32 pointer payload. */
	if (copy_from_user(&enable, (void __user *)arg, sizeof(enable)))
		return -EFAULT;
	cpus_read_lock();
	mutex_lock(&teeperf_lock);
	if (!file->private_data || *(unsigned long *)file->private_data != teeperf_epoch)
		ret = -ENODEV;
	else
		ret = teeperf_set_boost(!!enable);
	mutex_unlock(&teeperf_lock);
	cpus_read_unlock();
	return ret;
}

static ssize_t teeperf_write(struct file *file, const char __user *buffer,
			    size_t count, loff_t *position)
{
	char input[32], *value_text;
	long value;
	int ret;

	if (!count || count >= sizeof(input))
		return -EINVAL;
	if (copy_from_user(input, buffer, count))
		return -EFAULT;
	input[count] = '\0';
	value_text = strchr(input, ' ');
	if (!value_text)
		return -EINVAL;
	*value_text++ = '\0';
	if (strcmp(input, "teeperf_ut"))
		return -EINVAL;
	ret = kstrtol(skip_spaces(value_text), 10, &value);
	if (ret)
		return ret;
	cpus_read_lock();
	mutex_lock(&teeperf_lock);
	ret = teeperf_set_boost(!!value);
	mutex_unlock(&teeperf_lock);
	cpus_read_unlock();
	return ret ? ret : count;
}

static const struct file_operations teeperf_fops = {
	.owner = THIS_MODULE,
	.open = teeperf_open,
	.release = teeperf_release,
	.unlocked_ioctl = teeperf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_ptr_ioctl,
#endif
};
static const struct proc_ops teeperf_proc_ops = { .proc_write = teeperf_write };

static int teeperf_probe(struct platform_device *pdev)
{
	struct device *device;
	u32 type, map, index;
	int ret;

	if (!pdev->dev.of_node ||
	    of_property_read_u32(pdev->dev.of_node, "cpu-type", &type) || type != 2 ||
	    of_property_read_u32(pdev->dev.of_node, "cpu-map", &map) || map != 1)
		return -EINVAL;
	/* Original xaga selects little CPUs 0..3 at table index 0. */
	if (of_find_property(pdev->dev.of_node, "svp-cpu-index", NULL) &&
	    (of_property_read_u32(pdev->dev.of_node, "svp-cpu-index", &index) || index))
		return -EOPNOTSUPP;
	mutex_lock(&teeperf_lock);
	if (teeperf_owner) {
		ret = -EBUSY;
		goto unlock;
	}
	teeperf_owner = pdev;
	ret = alloc_chrdev_region(&teeperf_devt, 0, 1, "teeperf");
	if (ret)
		goto clear_owner;
	teeperf_class = class_create("teeperf");
	if (IS_ERR(teeperf_class)) {
		ret = PTR_ERR(teeperf_class);
		goto free_region;
	}
	teeperf_cdev = cdev_alloc();
	if (!teeperf_cdev) {
		ret = -ENOMEM;
		goto free_class;
	}
	teeperf_cdev->ops = &teeperf_fops;
	teeperf_cdev->owner = THIS_MODULE;
	ret = cdev_add(teeperf_cdev, teeperf_devt, 1);
	if (ret) {
		kobject_put(&teeperf_cdev->kobj);
		goto free_class;
	}
	device = device_create(teeperf_class, &pdev->dev, teeperf_devt, NULL, "teeperf");
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto free_cdev;
	}
	teeperf_proc = proc_create("teeperf_dbg", 0660, NULL, &teeperf_proc_ops);
	if (!teeperf_proc) {
		ret = -ENOMEM;
		goto free_device;
	}
	teeperf_epoch++;
	teeperf_ready = true;
	mutex_unlock(&teeperf_lock);
	return 0;
free_device:
	device_destroy(teeperf_class, teeperf_devt);
free_cdev:
	cdev_del(teeperf_cdev);
free_class:
	class_destroy(teeperf_class);
free_region:
	unregister_chrdev_region(teeperf_devt, 1);
clear_owner:
	teeperf_owner = NULL;
unlock:
	mutex_unlock(&teeperf_lock);
	return ret;
}

static void teeperf_remove(struct platform_device *pdev)
{
	mutex_lock(&teeperf_lock);
	teeperf_ready = false;
	teeperf_drop_votes();
	mutex_unlock(&teeperf_lock);
	/* proc_remove drains active proc callbacks; never hold their mutex here. */
	proc_remove(teeperf_proc);
	device_destroy(teeperf_class, teeperf_devt);
	cdev_del(teeperf_cdev);
	class_destroy(teeperf_class);
	unregister_chrdev_region(teeperf_devt, 1);
	mutex_lock(&teeperf_lock);
	teeperf_owner = NULL;
	mutex_unlock(&teeperf_lock);
}

static const struct of_device_id teeperf_of_match[] = {
	{ .compatible = "mediatek,teeperf" }, { }
};
MODULE_DEVICE_TABLE(of, teeperf_of_match);
static struct platform_driver teeperf_driver = {
	.probe = teeperf_probe,
	.remove = teeperf_remove,
	.driver = { .name = "teeperf", .of_match_table = teeperf_of_match }
};
module_platform_driver(teeperf_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Xaga TEE performance frequency QoS interface");
