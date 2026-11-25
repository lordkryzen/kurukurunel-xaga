// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "adsp_mbox.h"
#include "adsp_platform.h"
#include "adsp_platform_driver.h"
#include "adsp_core.h"
#include "adsp_clk_audio.h"
#include "adsp_driver_v2.h"
#include "mtk-afe-external.h"

const struct adspsys_description mt6983_adspsys_desc = {
	.platform_name = "mt6983",
	.version = 2,
	.semaphore_ways = 3,
	.semaphore_ctrl = 2,
	.semaphore_retry = 5000,
	.axibus_idle_val = 0x0,
};

const struct adspsys_description mt6879_adspsys_desc = {
	.platform_name = "mt6879",
	.version = 2,
	.semaphore_ways = 3,
	.semaphore_ctrl = 2,
	.semaphore_retry = 5000,
	.axibus_idle_val = 0x0,
};

const struct adspsys_description mt6895_adspsys_desc = {
	.platform_name = "mt6895",
	.version = 2,
	.semaphore_ways = 3,
	.semaphore_ctrl = 2,
	.semaphore_retry = 5000,
	.axibus_idle_val = 0x0,
};

const struct adsp_core_description mt6983_adsp_c0_desc = {
	.id = 0,
	.name = "adsp_0",
	.sharedmems = {
		[ADSP_SHAREDMEM_BOOTUP_MARK] = {0x0004, 0x0004},
		[ADSP_SHAREDMEM_SYS_STATUS] = {0x0008, 0x0004},
		[ADSP_SHAREDMEM_MPUINFO] = {0x0028, 0x0020},
		[ADSP_SHAREDMEM_WAKELOCK] = {0x002C, 0x0004},
		[ADSP_SHAREDMEM_IPCBUF] = {0x0300, 0x0200},
		[ADSP_SHAREDMEM_C2C_0_BUF] = {0x2508, 0x2208}, //common begin
		[ADSP_SHAREDMEM_C2C_BUFINFO] = {0x2510, 0x0008},
		[ADSP_SHAREDMEM_TIMESYNC] = {0x2530, 0x0020},
		[ADSP_SHAREDMEM_DVFSSYNC] = {0x253C, 0x000C},
		[ADSP_SHAREDMEM_SLEEPSYNC] = {0x2540, 0x0004},
		[ADSP_SHAREDMEM_BUS_MON_DUMP] = {0x25FC, 0x00BC},
		[ADSP_SHAREDMEM_INFRA_BUS_DUMP] = {0x269C, 0x00A0},
		[ADSP_SHAREDMEM_LATMON_DUMP] = {0x26B8, 0x001C},
	},
	.ops = {
		.initialize = adsp_core0_init,
		.after_bootup = adsp_after_bootup,
	}
};

const struct adsp_core_description mt6983_adsp_c1_desc = {
	.id = 1,
	.name = "adsp_1",
	.sharedmems = {
		[ADSP_SHAREDMEM_BOOTUP_MARK] = {0x0004, 0x0004},
		[ADSP_SHAREDMEM_SYS_STATUS] = {0x0008, 0x0004},
		[ADSP_SHAREDMEM_MPUINFO] = {0x0028, 0x0020},
		[ADSP_SHAREDMEM_WAKELOCK] = {0x002C, 0x0004},
		[ADSP_SHAREDMEM_IPCBUF] = {0x0300, 0x0200},
		[ADSP_SHAREDMEM_C2C_1_BUF] = {0x2508, 0x2208}, //common begin
	},
	.ops = {
		.initialize = adsp_core1_init,
		.after_bootup = adsp_after_bootup,
	}
};

const struct adsp_core_description mt6879_adsp_c0_desc = {
	.id = 0,
	.name = "adsp_0",
	.sharedmems = {
		[ADSP_SHAREDMEM_BOOTUP_MARK] = {0x0004, 0x0004},
		[ADSP_SHAREDMEM_SYS_STATUS] = {0x0008, 0x0004},
		[ADSP_SHAREDMEM_MPUINFO] = {0x0028, 0x0020},
		[ADSP_SHAREDMEM_WAKELOCK] = {0x002C, 0x0004},
		[ADSP_SHAREDMEM_IPCBUF] = {0x0300, 0x0200},
		[ADSP_SHAREDMEM_TIMESYNC] = {0x2530, 0x0020},
		[ADSP_SHAREDMEM_DVFSSYNC] = {0x253C, 0x000C},
		[ADSP_SHAREDMEM_SLEEPSYNC] = {0x2540, 0x0004},
		[ADSP_SHAREDMEM_BUS_MON_DUMP] = {0x25FC, 0x00BC},
		[ADSP_SHAREDMEM_INFRA_BUS_DUMP] = {0x269C, 0x00A0},
		[ADSP_SHAREDMEM_LATMON_DUMP] = {0x26B8, 0x001C},
	},
	.ops = {
		.initialize = adsp_core0_init,
		.after_bootup = adsp_after_bootup,
	}
};

const struct adsp_core_description mt6895_adsp_c0_desc = {
	.id = 0,
	.name = "adsp_0",
	.sharedmems = {
		[ADSP_SHAREDMEM_BOOTUP_MARK] = {0x0004, 0x0004},
		[ADSP_SHAREDMEM_SYS_STATUS] = {0x0008, 0x0004},
		[ADSP_SHAREDMEM_MPUINFO] = {0x0028, 0x0020},
		[ADSP_SHAREDMEM_WAKELOCK] = {0x002C, 0x0004},
		[ADSP_SHAREDMEM_IPCBUF] = {0x0300, 0x0200},
		[ADSP_SHAREDMEM_C2C_0_BUF] = {0x2508, 0x2208}, //common begin
		[ADSP_SHAREDMEM_C2C_BUFINFO] = {0x2510, 0x0008},
		[ADSP_SHAREDMEM_TIMESYNC] = {0x2530, 0x0020},
		[ADSP_SHAREDMEM_DVFSSYNC] = {0x253C, 0x000C},
		[ADSP_SHAREDMEM_SLEEPSYNC] = {0x2540, 0x0004},
		[ADSP_SHAREDMEM_BUS_MON_DUMP] = {0x25FC, 0x00BC},
		[ADSP_SHAREDMEM_INFRA_BUS_DUMP] = {0x269C, 0x00A0},
		[ADSP_SHAREDMEM_LATMON_DUMP] = {0x26B8, 0x001C},
	},
	.ops = {
		.initialize = adsp_core0_init,
		.after_bootup = adsp_after_bootup,
	}
};

const struct adsp_core_description mt6895_adsp_c1_desc = {
	.id = 1,
	.name = "adsp_1",
	.sharedmems = {
		[ADSP_SHAREDMEM_BOOTUP_MARK] = {0x0004, 0x0004},
		[ADSP_SHAREDMEM_SYS_STATUS] = {0x0008, 0x0004},
		[ADSP_SHAREDMEM_MPUINFO] = {0x0028, 0x0020},
		[ADSP_SHAREDMEM_WAKELOCK] = {0x002C, 0x0004},
		[ADSP_SHAREDMEM_IPCBUF] = {0x0300, 0x0200},
		[ADSP_SHAREDMEM_C2C_1_BUF] = {0x2508, 0x2208}, //common begin
	},
	.ops = {
		.initialize = adsp_core1_init,
		.after_bootup = adsp_after_bootup,
	}
};

static const struct of_device_id adspsys_of_ids[] = {
	{ .compatible = "mediatek,mt6983-adspsys", .data = &mt6983_adspsys_desc},
	{ .compatible = "mediatek,mt6879-adspsys", .data = &mt6879_adspsys_desc},
	{ .compatible = "mediatek,mt6895-adspsys", .data = &mt6895_adspsys_desc},
	{}
};

static const struct of_device_id adsp_core_of_ids[] = {
	{ .compatible = "mediatek,mt6983-adsp_core_0", .data = &mt6983_adsp_c0_desc},
	{ .compatible = "mediatek,mt6983-adsp_core_1", .data = &mt6983_adsp_c1_desc},
	{ .compatible = "mediatek,mt6879-adsp_core_0", .data = &mt6879_adsp_c0_desc},
	{ .compatible = "mediatek,mt6895-adsp_core_0", .data = &mt6895_adsp_c0_desc},
	{ .compatible = "mediatek,mt6895-adsp_core_1", .data = &mt6895_adsp_c1_desc},
	{}
};

/* Diagnostic ownership boundary: no module exit or manual unbind while
 * native globals/callbacks refer to the devm-owned platform resources.
 * Firmware boot is one explicit operation after all three providers bind.
 */
static struct platform_device *adsp_bound[3];
static bool adsp_registered;
static int registration_result = -EAGAIN;
module_param(registration_result, int, 0444);
static DEFINE_MUTEX(adsp_boot_lock);
static atomic_t adsp_boot_attempted = ATOMIC_INIT(0);
static int boot_result = -EAGAIN;
module_param(boot_result, int, 0444);

static int adspsys_drv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct adspsys_priv *adspsys;

	/* create private data */
	adspsys = devm_kzalloc(dev, sizeof(*adspsys), GFP_KERNEL);
	if (!adspsys)
		return -ENOMEM;

	match = of_match_node(adspsys_of_ids, dev->of_node);
	if (!match)
		return -ENODEV;

	adspsys->desc = (struct adspsys_description *)match->data;
	adspsys->dev = dev;

	/* get resource from platform_device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	adspsys->cfg = devm_ioremap_resource(dev, res);
	if (IS_ERR(adspsys->cfg))
		return PTR_ERR(adspsys->cfg);
	adspsys->cfg_size = resource_size(res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	adspsys->cfg2 = devm_ioremap_resource(dev, res);
	if (IS_ERR(adspsys->cfg2))
		return PTR_ERR(adspsys->cfg2);
	adspsys->cfg2_size = resource_size(res);

	ret = of_property_read_u32(dev->of_node, "core_num", &adspsys->num_cores);
	if (ret || adspsys->num_cores != 2)
		return ret ? ret : -EINVAL;

	ret = adsp_clk_probe(pdev, &adspsys->clk_ops);
	if (ret) {
		pr_warn("%s(), clk probe fail, %d\n", __func__, ret);
		goto ERROR;
	}

	ret = adsp_mem_device_probe(pdev);
	if (ret) {
		pr_info("%s(), memory probe fail, %d\n", __func__, ret);
		goto ERROR;
	}

	ret = adsp_mbox_probe(pdev);
	if (ret) {
		pr_warn("%s(), mbox probe fail, %d\n", __func__, ret);
		goto ERROR;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		pr_warn("%s(), pm_runtime_enable fail, %d\n", __func__, ret);

	/* register as syscore_device, not to be turned off when suspend */
	dev_pm_syscore_device(&pdev->dev, true);

	/* register syscore if adsp on infra */
	//register_syscore_ops(&adsp_syscore_ops);

	register_adspsys(adspsys);
	platform_set_drvdata(pdev, adspsys);
	WRITE_ONCE(adsp_bound[0], pdev);

	pr_info("%s, success\n", __func__);
ERROR:
	return ret;
}

static void adspsys_drv_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}

static int adsp_core_drv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct adsp_priv *pdata;
	const struct adsp_core_description *desc;
	const struct of_device_id *match;
	struct of_phandle_args spec;
	u64 system_info[2];

	pr_info("%s: adsp core init\n", __func__);

	/* create private data */
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	match = of_match_node(adsp_core_of_ids, dev->of_node);
	if (!match) {
		pr_info("%s: cannot find match node\n", __func__);
		return -ENODEV;
	}

	desc = (struct adsp_core_description *)match->data;

	pdata->id = desc->id;
	pdata->name = desc->name;
	pdata->ops = &desc->ops;
	pdata->mapping_table = desc->sharedmems;

	pdata->dev = dev;
	init_completion(&pdata->done);

	/* get resource from platform_device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	pdata->itcm = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->itcm))
		return PTR_ERR(pdata->itcm);
	pdata->itcm_size = resource_size(res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	pdata->dtcm = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->dtcm))
		return PTR_ERR(pdata->dtcm);
	pdata->dtcm_size = resource_size(res);

	pdata->irq[ADSP_IRQ_IPC_ID].seq = platform_get_irq(pdev, 0);
	if (pdata->irq[ADSP_IRQ_IPC_ID].seq < 0)
		return pdata->irq[ADSP_IRQ_IPC_ID].seq;
	pdata->irq[ADSP_IRQ_IPC_ID].clear_irq = adsp_mt_clr_sysirq;
	pdata->irq[ADSP_IRQ_WDT_ID].seq = platform_get_irq(pdev, 1);
	if (pdata->irq[ADSP_IRQ_WDT_ID].seq < 0)
		return pdata->irq[ADSP_IRQ_WDT_ID].seq;
	pdata->irq[ADSP_IRQ_WDT_ID].clear_irq = adsp_mt_disable_wdt;
	pdata->irq[ADSP_IRQ_AUDIO_ID].seq = platform_get_irq(pdev, 2);
	if (pdata->irq[ADSP_IRQ_AUDIO_ID].seq < 0)
		return pdata->irq[ADSP_IRQ_AUDIO_ID].seq;
	pdata->irq[ADSP_IRQ_AUDIO_ID].clear_irq = adsp_mt_clr_auidoirq;

	ret = of_property_read_u64_array(dev->of_node, "system", system_info, 2);
	if (ret)
		return ret;
	pdata->sysram_phys = (phys_addr_t)system_info[0];
	pdata->sysram_size = (size_t)system_info[1];

	if (pdata->sysram_phys == 0 || pdata->sysram_size == 0) {
		pr_info("%s: get property fail, sysram_phys: 0x%llx, sysram_size: %zu\n",
			__func__, pdata->sysram_phys, pdata->sysram_size);
		return -ENODEV;
	}
	pdata->sysram = devm_ioremap_wc(dev, pdata->sysram_phys, pdata->sysram_size);
	if (!pdata->sysram)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "feature_control_bits",
			     &pdata->feature_set);

	/* mailbox channel parsing */
	if (of_parse_phandle_with_args(dev->of_node, "mboxes",
				       "#mbox-cells", 0, &spec)) {
		pr_info("%s: can't parse \"mboxes\" property\n", __func__);
		return -ENODEV;
	}
	if (spec.args_count != 1) {
		of_node_put(spec.np);
		return -EINVAL;
	}
	pdata->send_mbox = get_adsp_mbox_pin_send(spec.args[0]);
	of_node_put(spec.np);
	if (IS_ERR(pdata->send_mbox))
		return PTR_ERR(pdata->send_mbox);
	if (!pdata->send_mbox)
		return -ENODEV;

	if (of_parse_phandle_with_args(dev->of_node, "mboxes",
				       "#mbox-cells", 1, &spec)) {
		pr_info("%s: can't parse \"mboxes\" property\n", __func__);
		return -ENODEV;
	}
	if (spec.args_count != 1) {
		of_node_put(spec.np);
		return -EINVAL;
	}
	pdata->recv_mbox = get_adsp_mbox_pin_recv(spec.args[0]);
	of_node_put(spec.np);
	if (IS_ERR(pdata->recv_mbox))
		return PTR_ERR(pdata->recv_mbox);
	if (!pdata->recv_mbox)
		return -ENODEV;

	/* add to adsp_core list */
	if (pdata->id > 1)
		return -EINVAL;
	register_adsp_core(pdata);
	platform_set_drvdata(pdev, pdata);
	WRITE_ONCE(adsp_bound[pdata->id + 1], pdev);

	pr_info("%s, id:%d success\n", __func__, pdata->id);
	return ret;
}

static void adsp_core_drv_remove(struct platform_device *pdev)
{
}

static struct platform_driver adspsys_driver = {
	.probe = adspsys_drv_probe,
	.remove = adspsys_drv_remove,
	.driver = {
		.name = "adspsys",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = adspsys_of_ids,
#endif
	},
};

static struct platform_driver adsp_core0_driver = {
	.probe = adsp_core_drv_probe,
	.remove = adsp_core_drv_remove,
	.driver = {
		.name = "adsp_core0",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = adsp_core_of_ids,
#endif
	},
};

static struct platform_driver adsp_core1_driver = {
	.probe = adsp_core_drv_probe,
	.remove = adsp_core_drv_remove,
	.driver = {
		.name = "adsp_core1",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = adsp_core_of_ids,
#endif
	},
};

static struct platform_driver * const drivers[] = {
	&adspsys_driver,
	&adsp_core0_driver,
	&adsp_core1_driver,
};

int notify_adsp_semaphore_event(struct notifier_block *nb,
				unsigned long event, void *v)
{
	int status = NOTIFY_DONE;

	if (event == NOTIFIER_ADSP_3WAY_SEMAPHORE_GET) {
		status = (get_adsp_semaphore(SEMA_AUDIOREG) == ADSP_OK) ?
			 NOTIFY_STOP : NOTIFY_BAD;
	} else if (event == NOTIFIER_ADSP_3WAY_SEMAPHORE_RELEASE) {
		release_adsp_semaphore(SEMA_AUDIOREG);
		status = NOTIFY_STOP;
	}

	return status;
}

static struct notifier_block adsp_semaphore_init_notifier = {
	.notifier_call = notify_adsp_semaphore_event,
};

/*
 * driver initialization entry point
 */
static int adsp_boot_once_set(const char *value, const struct kernel_param *parameter)
{
	unsigned int request;
	int ret;

	ret = kstrtouint(value, 0, &request);
	if (ret || request != 1)
		return ret ? ret : -EINVAL;
	mutex_lock(&adsp_boot_lock);
	if (!READ_ONCE(adsp_registered) || !READ_ONCE(adsp_bound[0]) ||
	    !READ_ONCE(adsp_bound[1]) || !READ_ONCE(adsp_bound[2])) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	if (atomic_cmpxchg(&adsp_boot_attempted, 0, 1)) {
		ret = -EALREADY;
		goto out;
	}
	WRITE_ONCE(boot_result, -EINPROGRESS);
	pr_info("adsp-startup: one native firmware boot attempt\n");
	ret = adsp_system_bootup();
	if (!ret)
		ret = register_3way_semaphore_notifier(&adsp_semaphore_init_notifier);
	WRITE_ONCE(boot_result, ret);
	pr_info("adsp-startup: native firmware boot result=%d\n", ret);
out:
	mutex_unlock(&adsp_boot_lock);
	return ret;
}

static const struct kernel_param_ops adsp_boot_once_ops = {
	.set = adsp_boot_once_set,
};
module_param_cb(boot_once, &adsp_boot_once_ops, NULL, 0200);

static int __init platform_adsp_init(void)
{
	int ret;
	size_t i;

	/* Keep successful owners pinned if a later registration fails: native
	 * published globals have no unregister contract. The separate result
	 * reports registration failure and keeps firmware boot gated off.
	 */
	for (i = 0; i < ARRAY_SIZE(drivers); i++) {
		ret = platform_driver_register(drivers[i]);
		if (ret) {
			WRITE_ONCE(registration_result, ret);
			pr_err("adsp-startup: driver %zu registration failed: %d; firmware gated\n", i, ret);
			return 0;
		}
	}
	WRITE_ONCE(registration_result, 0);
	WRITE_ONCE(adsp_registered, true);
	pr_info("adsp-startup: platform drivers registered; firmware NOT started\n");
	return 0;
}

module_init(platform_adsp_init);
/* Permanent diagnostic owner: no active remove/unload until quiescence is qualified. */

MODULE_AUTHOR("Chien-Wei Hsu <Chien-Wei.Hsu@mediatek.com>");
MODULE_DESCRIPTION("MTK AUDIO DSP PLATFORM Device Driver");
MODULE_LICENSE("GPL v2");

