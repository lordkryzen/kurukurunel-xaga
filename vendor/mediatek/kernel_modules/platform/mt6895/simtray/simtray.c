// SPDX-License-Identifier: GPL-2.0
/*
 * SIM card and SD card tray insertion and extraction monitoring support
 *
 * Copyright (C) 2021-2022 XiaoMi, Inc.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>

struct simtray_data {
	struct device *dev;
	int status_gpio;
};

static ssize_t simtray_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct simtray_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value_cansleep(data->status_gpio));
}
static DEVICE_ATTR(status, 0444, simtray_status_show, NULL);

static int simtray_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simtray_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->status_gpio = of_get_named_gpio(dev->of_node, "status-gpio", 0);
	if (data->status_gpio < 0)
		return dev_err_probe(dev, data->status_gpio, "Failed to get tray GPIO\n");

	/* The modem owns the hotplug mux. Requesting this pin as a GPIO
	 * switches MD_INT1 to GPIO mode and prevents modem hotplug events.
	 * Match the native 5.10 monitor: resolve and read the existing signal,
	 * without requesting, changing direction or reconfiguring the mux.
	 */

	data->dev = dev;
	platform_set_drvdata(pdev, data);
	return 0;
}

static struct attribute *simtray_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(simtray);

static const struct of_device_id simtray_of_match[] = {
	{ .compatible = "xiaomi,simtray-status", },
	{},
};

MODULE_DEVICE_TABLE(of, simtray_of_match);

static struct platform_driver simtray_status_driver = {
	.driver = {
		.name = "simtray-status",
		.dev_groups = simtray_groups,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(simtray_of_match),
	},
	.probe = simtray_probe,
};

module_platform_driver(simtray_status_driver);
MODULE_AUTHOR("Tao Jun<taojun@xiaomi.com>");
MODULE_DESCRIPTION("Xiaomi SIM tray status");
MODULE_LICENSE("GPL");


