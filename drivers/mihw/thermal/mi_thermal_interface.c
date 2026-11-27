// SPDX-License-Identifier: GPL-2.0-only
/* Xiaomi thermal userspace interface, derived from Xiaomi's xaga driver. */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#include "mi_disp_notifier.h"

#ifndef MI_THERMAL_DEVICE_NAME
#define MI_THERMAL_DEVICE_NAME "thermal_message"
#endif

struct thermal_sensor_alias {
	const char *name;
	const char *source_name;
	struct thermal_zone_device *source;
	struct thermal_zone_device *zone;
};

static struct thermal_sensor_alias sensor_aliases[] = {
	{ .name = "mtktsAP", .source_name = "ap_ntc" },
	{ .name = "mtktsbtsmdpa", .source_name = "ltepa_ntc" },
	{ .name = "mtktsbtsnrpa", .source_name = "nrpa_ntc" },
};

static int sensor_alias_get_temp(struct thermal_zone_device *zone, int *temp)
{
	struct thermal_sensor_alias *alias = thermal_zone_device_priv(zone);

	return thermal_zone_get_temp(alias->source, temp);
}

static const struct thermal_zone_device_ops sensor_alias_ops = {
	.get_temp = sensor_alias_get_temp,
};

static void sensor_aliases_destroy(void)
{
	int i;

	for (i = ARRAY_SIZE(sensor_aliases) - 1; i >= 0; i--) {
		struct thermal_sensor_alias *alias = &sensor_aliases[i];

		if (alias->zone) {
			thermal_zone_device_unregister(alias->zone);
			alias->zone = NULL;
		}
		if (alias->source) {
			put_device(thermal_zone_device(alias->source));
			alias->source = NULL;
		}
	}
}

static int sensor_aliases_init(void)
{
	static const struct thermal_zone_params params = { .no_hwmon = true };
	int i, ret;

	/* Xaga's native thermal policy uses the shipped NTC sensor names. */
	for (i = 0; i < ARRAY_SIZE(sensor_aliases); i++) {
		struct thermal_sensor_alias *alias = &sensor_aliases[i];
		struct thermal_zone_device *source;

		source = thermal_zone_get_zone_by_name(alias->name);
		if (!IS_ERR(source))
			continue;
		if (PTR_ERR(source) != -ENODEV) {
			ret = PTR_ERR(source);
			goto fail;
		}
		source = thermal_zone_get_zone_by_name(alias->source_name);
		if (IS_ERR(source)) {
			ret = PTR_ERR(source) == -ENODEV ? -EPROBE_DEFER : PTR_ERR(source);
			goto fail;
		}
		get_device(thermal_zone_device(source));
		alias->source = source;
		alias->zone = thermal_zone_device_register_with_trips(alias->name,
				NULL, 0, alias, &sensor_alias_ops, &params, 0, 0);
		if (IS_ERR(alias->zone)) {
			ret = PTR_ERR(alias->zone);
			alias->zone = NULL;
			goto fail;
		}
	}
	return 0;
fail:
	sensor_aliases_destroy();
	return ret;
}

static struct device *message_dev;
static char board_sensor[128];
static DEFINE_MUTEX(text_lock);
static char boost[128];
static char board_sensor_temp[128];
static atomic_t screen_state = ATOMIC_INIT(0);
static atomic_t usb_online = ATOMIC_INIT(-1);

#define THERMAL_VALUE(name, initial) \
static atomic_t name##_value = ATOMIC_INIT(initial); \
static ssize_t name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	return sysfs_emit(buf, "%d\n", atomic_read(&name##_value)); \
} \
static ssize_t name##_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count) \
{ \
	int value, ret = kstrtoint(buf, 10, &value); \
	if (memchr(buf, '\0', count)) \
		return -EINVAL; \
	if (ret) \
		return ret; \
	atomic_set(&name##_value, value); \
	return count; \
} \
static struct device_attribute dev_attr_##name = \
	__ATTR(name, 0664, name##_show, name##_store)

THERMAL_VALUE(temp_state, 0);
THERMAL_VALUE(sconfig, -1);
THERMAL_VALUE(balance_mode, 0);
THERMAL_VALUE(modem_limit, 0);
THERMAL_VALUE(poor_modem_limit, 0);
THERMAL_VALUE(wifi_limit, 0);
THERMAL_VALUE(market_download_limit, 0);
THERMAL_VALUE(flash_state, 0);
THERMAL_VALUE(thermal_max_brightness, 0);
THERMAL_VALUE(atc_enable, -1);
THERMAL_VALUE(charger_temp, -1);

static ssize_t text_show(char *buf, const char *text)
{
	ssize_t ret;

	mutex_lock(&text_lock);
	ret = sysfs_emit(buf, "%s", text);
	mutex_unlock(&text_lock);
	return ret;
}

static ssize_t text_store(char *text, const char *buf, size_t count)
{
	if (count >= sizeof(boost))
		return -E2BIG;
	if (memchr(buf, '\0', count))
		return -EINVAL;
	mutex_lock(&text_lock);
	memcpy(text, buf, count);
	text[count] = '\0';
	mutex_unlock(&text_lock);
	return count;
}

#define THERMAL_TEXT(name, mode) \
static ssize_t name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	return text_show(buf, name); \
} \
static ssize_t name##_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count) \
{ \
	return text_store(name, buf, count); \
} \
static struct device_attribute dev_attr_##name = \
	__ATTR(name, mode, name##_show, name##_store)

THERMAL_TEXT(boost, 0644);
THERMAL_TEXT(board_sensor_temp, 0664);

static ssize_t board_sensor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s", board_sensor);
}
static DEVICE_ATTR_RO(board_sensor);

static ssize_t screen_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&screen_state));
}
static DEVICE_ATTR_RO(screen_state);

static ssize_t usb_online_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&usb_online));
}
static DEVICE_ATTR_RO(usb_online);

struct thermal_cpu_limit {
	struct list_head node;
	struct cpufreq_policy *policy;
	struct freq_qos_request request;
};
static LIST_HEAD(cpu_limits_list);
static DEFINE_MUTEX(cpu_limits_lock);

static void cpu_limits_destroy(void)
{
	struct thermal_cpu_limit *limit, *next;

	list_for_each_entry_safe(limit, next, &cpu_limits_list, node) {
		freq_qos_remove_request(&limit->request);
		cpufreq_cpu_put(limit->policy);
		list_del(&limit->node);
		kfree(limit);
	}
}

static int cpu_limits_init(void)
{
	struct thermal_cpu_limit *limit;
	cpumask_t seen;
	unsigned int cpu;
	int ret;

	cpumask_clear(&seen);
	cpus_read_lock();
	for_each_possible_cpu(cpu) {
		if (cpumask_test_cpu(cpu, &seen))
			continue;
		limit = kzalloc(sizeof(*limit), GFP_KERNEL);
		if (!limit) {
			ret = -ENOMEM;
			goto fail;
		}
		limit->policy = cpufreq_cpu_get(cpu);
		if (!limit->policy) {
			kfree(limit);
			ret = -EPROBE_DEFER;
			goto fail;
		}
		if (!limit->policy->freq_table) {
			cpufreq_cpu_put(limit->policy);
			kfree(limit);
			ret = -ENODEV;
			goto fail;
		}
		ret = freq_qos_add_request(&limit->policy->constraints,
				&limit->request, FREQ_QOS_MAX,
				FREQ_QOS_MAX_DEFAULT_VALUE);
		if (ret < 0) {
			cpufreq_cpu_put(limit->policy);
			kfree(limit);
			goto fail;
		}
		cpumask_or(&seen, &seen, limit->policy->related_cpus);
		list_add_tail(&limit->node, &cpu_limits_list);
	}
	cpus_read_unlock();
	return 0;
fail:
	cpus_read_unlock();
	cpu_limits_destroy();
	return ret;
}

static ssize_t cpu_limits_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t cpu_limits_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct thermal_cpu_limit *limit;
	struct cpufreq_frequency_table *entry;
	unsigned int cpu, requested, frequency, minimum;
	char input[64], *cursor, *token;
	int ret = -ENODEV;

	if (count >= sizeof(input) || count < 6 || memchr(buf, '\0', count))
		return -EINVAL;
	memcpy(input, buf, count);
	input[count] = '\0';
	cursor = strim(input);
	if (strncmp(cursor, "cpu", 3))
		return -EINVAL;
	cursor += 3;
	token = strsep(&cursor, " \t");
	if (!cursor || kstrtouint(token, 10, &cpu) ||
	    kstrtouint(skip_spaces(cursor), 10, &requested) ||
	    cpu >= nr_cpu_ids || !cpu_possible(cpu))
		return -EINVAL;

	mutex_lock(&cpu_limits_lock);
	list_for_each_entry(limit, &cpu_limits_list, node) {
		if (!cpumask_test_cpu(cpu, limit->policy->related_cpus))
			continue;
		minimum = UINT_MAX;
		frequency = 0;
		cpufreq_for_each_valid_entry(entry, limit->policy->freq_table) {
			minimum = min(minimum, entry->frequency);
			if (entry->frequency <= requested)
				frequency = max(frequency, entry->frequency);
		}
		if (minimum == UINT_MAX) {
			ret = -ENODEV;
			break;
		}
		/* A request below the table still selects its lowest valid OPP. */
		if (!frequency)
			frequency = minimum;
		ret = freq_qos_update_request(&limit->request, frequency);
		break;
	}
	mutex_unlock(&cpu_limits_lock);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(cpu_limits, 0664, cpu_limits_show, cpu_limits_store);

static struct attribute *thermal_message_attrs[] = {
	&dev_attr_temp_state.attr,
	&dev_attr_cpu_limits.attr,
	&dev_attr_sconfig.attr,
	&dev_attr_screen_state.attr,
	&dev_attr_boost.attr,
	&dev_attr_board_sensor.attr,
	&dev_attr_board_sensor_temp.attr,
	&dev_attr_balance_mode.attr,
	&dev_attr_modem_limit.attr,
	&dev_attr_poor_modem_limit.attr,
	&dev_attr_wifi_limit.attr,
	&dev_attr_thermal_max_brightness.attr,
	&dev_attr_market_download_limit.attr,
	&dev_attr_flash_state.attr,
	&dev_attr_atc_enable.attr,
	&dev_attr_charger_temp.attr,
	&dev_attr_usb_online.attr,
	NULL,
};
ATTRIBUTE_GROUPS(thermal_message);

static int screen_event(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct mi_disp_notifier *notification = data;
	int state, old;

	if (event != MI_DISP_DPMS_EVENT || !notification ||
	    notification->disp_id != MI_DISPLAY_PRIMARY || !notification->data)
		return NOTIFY_DONE;
	state = *(int *)notification->data;
	if (state != MI_DISP_DPMS_ON && state != MI_DISP_DPMS_POWERDOWN &&
	    state != MI_DISP_DPMS_LP1 && state != MI_DISP_DPMS_LP2)
		return NOTIFY_DONE;
	state = state == MI_DISP_DPMS_ON;
	old = atomic_xchg(&screen_state, state);
	if (old != state)
		sysfs_notify(&message_dev->kobj, NULL, "screen_state");
	return NOTIFY_OK;
}
static struct notifier_block screen_notifier = {
	.notifier_call = screen_event,
};

static void update_usb_online(struct work_struct *work)
{
	struct power_supply *usb;
	union power_supply_propval value;
	int ret, old;

	usb = power_supply_get_by_name("usb");
	if (!usb)
		return;
	ret = power_supply_get_property(usb, POWER_SUPPLY_PROP_ONLINE, &value);
	power_supply_put(usb);
	if (ret)
		return;
	old = atomic_xchg(&usb_online, !!value.intval);
	if (old != !!value.intval)
		sysfs_notify(&message_dev->kobj, NULL, "usb_online");
}
static DECLARE_WORK(usb_work, update_usb_online);

static int usb_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && psy &&
	    !strcmp(psy->desc->name, "usb"))
		schedule_work(&usb_work);
	return NOTIFY_OK;
}
static struct notifier_block usb_notifier = {
	.notifier_call = usb_event,
};

static int __init mi_thermal_interface_init(void)
{
	struct thermal_zone_device *zone;
	struct device_node *node;
	struct device *zone_dev;
	const char *sensor;
	int ret;

	node = of_find_node_by_name(NULL, "thermal-message");
	if (!node)
		return -ENODEV;
	ret = of_property_read_string(node, "board-sensor", &sensor);
	if (!ret)
		ret = strscpy(board_sensor, sensor, sizeof(board_sensor));
	of_node_put(node);
	if (ret < 0)
		return ret;

	/* Borrow the public device's class, not private kernfs/class internals. */
	zone = thermal_zone_get_zone_by_name("soc_max");
	if (IS_ERR(zone))
		return PTR_ERR(zone);
	zone_dev = get_device(thermal_zone_device(zone));
	if (!zone_dev)
		return -ENODEV;
	ret = sensor_aliases_init();
	if (ret)
		goto put_zone;
	ret = cpu_limits_init();
	if (ret)
		goto destroy_aliases;
	message_dev = device_create_with_groups(zone_dev->class, NULL, 0, NULL,
			thermal_message_groups, MI_THERMAL_DEVICE_NAME);
	if (IS_ERR(message_dev)) {
		ret = PTR_ERR(message_dev);
		goto destroy_cpu;
	}
	ret = mi_disp_register_client(&screen_notifier);
	if (ret)
		goto destroy_device;
	ret = power_supply_reg_notifier(&usb_notifier);
	if (ret)
		goto unregister_screen;
	update_usb_online(NULL);
	put_device(zone_dev);
	return 0;

unregister_screen:
	mi_disp_unregister_client(&screen_notifier);
destroy_device:
	device_unregister(message_dev);
destroy_cpu:
	cpu_limits_destroy();
destroy_aliases:
	sensor_aliases_destroy();
put_zone:
	put_device(zone_dev);
	return ret;
}

static void __exit mi_thermal_interface_exit(void)
{
	power_supply_unreg_notifier(&usb_notifier);
	mi_disp_unregister_client(&screen_notifier);
	cancel_work_sync(&usb_work);
	device_unregister(message_dev);
	cpu_limits_destroy();
	sensor_aliases_destroy();
}

module_init(mi_thermal_interface_init);
module_exit(mi_thermal_interface_exit);
MODULE_DESCRIPTION("Xiaomi thermal control interface");
MODULE_LICENSE("GPL");
