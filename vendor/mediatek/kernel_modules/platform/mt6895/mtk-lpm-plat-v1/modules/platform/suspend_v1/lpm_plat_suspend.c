// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/cpuidle.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/cpu.h>
#include <linux/syscore_ops.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/irq.h>
#include <asm/cpuidle.h>
#include <asm/suspend.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/spinlock.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <uapi/linux/sched/types.h>

#include <lpm.h>
#include <lpm_module.h>
#include <lpm_call.h>
#include <lpm_type.h>
#include <lpm_call_type.h>
#include <lpm_dbg_common_v1.h>

#include "lpm_plat.h"
#include "lpm_plat_comm.h"
#include "lpm_plat_suspend.h"

unsigned int lpm_suspend_status;
struct cpumask s2idle_cpumask;
static struct pm_qos_request lpm_qos_request;
static bool model_owned, model_qos_owned, model_pm_owned, spm_irq_owned, spm_wake_owned;
static int spm_virq=-1;
static const char *model_name;
static unsigned long suspend_prompts, suspend_reflects;
module_param(suspend_prompts,ulong,0444);
module_param(suspend_reflects,ulong,0444);

long long before_md_sleep_time;
long long after_md_sleep_time;

#define S2IDLE_STATE_NAME "s2idle"

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
#define MD_SLEEP_INFO_SMEM_OFFEST (4)
u32 __iomem *share_mem;
static void __iomem *md_sleep_base;
struct md_sleep_status before_md_sleep_status;
struct md_sleep_status after_md_sleep_status;
#endif

static int get_md_sleep_time_addr(void)
{
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
 struct device_node *node;u64 address;int ret;
 node=of_find_compatible_node(NULL,NULL,"mediatek,mddriver");
 if(!node)return 0;
 ret=of_property_read_u64(node,"md_low_power_addr",&address);of_node_put(node);
 if(ret)return 0; /* Optional native modem sleep telemetry. */
 if(!address || address>PHYS_ADDR_MAX-0x200)return -EINVAL;
 md_sleep_base=ioremap_wc(address,0x200);if(!md_sleep_base)return -ENOMEM;
 share_mem=md_sleep_base+MD_SLEEP_INFO_SMEM_OFFEST*sizeof(u32);
#endif
 return 0;
}

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
static void get_md_sleep_time(struct md_sleep_status *md_data)
{
	if (!md_data)
		return;

	/* dump subsystem sleep info */
	if (share_mem ==  NULL) {
		pr_info("MD shared memory is NULL");
	} else {
		memset(md_data, 0, sizeof(struct md_sleep_status));
		memcpy_fromio(md_data, share_mem, sizeof(struct md_sleep_status));
		return;
	}
}
#endif
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
static void log_md_sleep_info(void)
{

#define LOG_BUF_SIZE	256
	char log_buf[LOG_BUF_SIZE] = { 0 };
	int log_size = 0;

	if (after_md_sleep_status.sleep_time >= before_md_sleep_status.sleep_time) {
		pr_info("[name:spm&][SPM] md_slp_duration = %llu (32k)\n",
			after_md_sleep_status.sleep_time - before_md_sleep_status.sleep_time);

		log_size += scnprintf(log_buf + log_size,
		LOG_BUF_SIZE - log_size, "[name:spm&][SPM] ");
		log_size += scnprintf(log_buf + log_size,
		LOG_BUF_SIZE - log_size, "MD/2G/3G/4G/5G_FR1 = ");
		log_size += scnprintf(log_buf + log_size,
		LOG_BUF_SIZE - log_size, "%llu.%03llu/%llu.%03llu/%llu.%03llu/%llu.%03llu/%llu.%03llu seconds",
			(after_md_sleep_status.md_sleep_time -
				before_md_sleep_status.md_sleep_time) / 1000000,
			(after_md_sleep_status.md_sleep_time -
				before_md_sleep_status.md_sleep_time) % 1000000 / 1000,
			(after_md_sleep_status.gsm_sleep_time -
				before_md_sleep_status.gsm_sleep_time) / 1000000,
			(after_md_sleep_status.gsm_sleep_time -
				before_md_sleep_status.gsm_sleep_time) % 1000000 / 1000,
			(after_md_sleep_status.wcdma_sleep_time -
				before_md_sleep_status.wcdma_sleep_time) / 1000000,
			(after_md_sleep_status.wcdma_sleep_time -
				before_md_sleep_status.wcdma_sleep_time) % 1000000 / 1000,
			(after_md_sleep_status.lte_sleep_time -
				before_md_sleep_status.lte_sleep_time) / 1000000,
			(after_md_sleep_status.lte_sleep_time -
				before_md_sleep_status.lte_sleep_time) % 1000000 / 1000,
			(after_md_sleep_status.nr_sleep_time -
				before_md_sleep_status.nr_sleep_time) / 1000000,
			(after_md_sleep_status.nr_sleep_time -
				before_md_sleep_status.nr_sleep_time) % 1000000 / 1000);

		WARN_ON(strlen(log_buf) >= LOG_BUF_SIZE);
		pr_info("[name:spm&][SPM] %s", log_buf);
	}
}
#endif

static inline int lpm_suspend_common_enter(unsigned int *susp_status)
{
	unsigned int status = PLAT_VCORE_LP_MODE
				| PLAT_PMIC_VCORE_SRCLKEN0
				| PLAT_SUSPEND;

	/* maybe need to stop sspm/mcupm mcdi task here */
	if (susp_status)
		*susp_status = status;

	return 0;
}


static inline int lpm_suspend_common_resume(unsigned int susp_status)
{
	/* Implement suspend common flow here */
	return 0;
}

static int __lpm_suspend_prompt(int type, int cpu,
				   const struct lpm_issuer *issuer)
{
	int ret = 0;
	unsigned int spm_res = 0;

	suspend_prompts++;
	lpm_suspend_status = 0;

	ret = lpm_suspend_common_enter(&lpm_suspend_status);

	if (ret)
		goto PLAT_LEAVE_SUSPEND;

	/* Legacy SSPM flow, spm sw resource request flow */
	lpm_do_mcusys_prepare_pdn(lpm_suspend_status, &spm_res);

PLAT_LEAVE_SUSPEND:
	return ret;
}

static void __lpm_suspend_reflect(int type, int cpu,
					const struct lpm_issuer *issuer)
{
	struct lpm_issuer _suspend_issuer;
	suspend_reflects++;

	lpm_suspend_common_resume(lpm_suspend_status);
	lpm_do_mcusys_prepare_on();
	if (issuer) {
		/* make sure suspend always print log */
		_suspend_issuer.log_type = LOG_SUCCEESS;
		_suspend_issuer.log = issuer->log;
		_suspend_issuer.log(LPM_ISSUER_SUSPEND, "suspend", (void *)&_suspend_issuer);
	}
}
int lpm_suspend_system_prompt(int cpu,
					const struct lpm_issuer *issuer)
{
	return __lpm_suspend_prompt(LPM_SUSPEND_S2IDLE,
				       cpu, issuer);
}

void lpm_suspend_system_reflect(int cpu,
					const struct lpm_issuer *issuer)
{
	return __lpm_suspend_reflect(LPM_SUSPEND_S2IDLE,
					cpu, issuer);
}

int lpm_suspend_s2idle_prompt(int cpu,
					const struct lpm_issuer *issuer)
{
	int ret = 0;

	cpumask_set_cpu(cpu, &s2idle_cpumask);
	if (cpumask_weight(&s2idle_cpumask) == num_online_cpus()) {

		pr_info("[name:spm&][%s:%d] - suspend enter\n",
			__func__, __LINE__);

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
		/* Record md sleep status*/
		get_md_sleep_time(&before_md_sleep_status);
#endif
		ret = __lpm_suspend_prompt(LPM_SUSPEND_S2IDLE,
					      cpu, issuer);
	}
	return ret;
}

void lpm_suspend_s2idle_reflect(int cpu,
					const struct lpm_issuer *issuer)
{
	if (cpumask_weight(&s2idle_cpumask) == num_online_cpus()) {

		__lpm_suspend_reflect(LPM_SUSPEND_S2IDLE,
					 cpu, issuer);
	pr_info("[name:spm&][%s:%d] - resume\n",
			__func__, __LINE__);

	if ((after_md_sleep_time >= 0) && (after_md_sleep_time >= before_md_sleep_time))
		pr_info("[name:spm&][SPM] md_slp_duration = %lld",
			after_md_sleep_time - before_md_sleep_time);
	else
		pr_info("[name:spm&][SPM] md share memory is NULL");
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	/* show md sleep status */
	get_md_sleep_time(&after_md_sleep_status);
	log_md_sleep_info();
#endif

	pm_system_wakeup();

	}
	cpumask_clear_cpu(cpu, &s2idle_cpumask);
}

#define LPM_SUSPEND_OP_INIT(_prompt, _enter, _resume, _reflect) ({\
	lpm_model_suspend.op.prompt = _prompt;\
	lpm_model_suspend.op.prepare_enter = _enter;\
	lpm_model_suspend.op.prepare_resume = _resume;\
	lpm_model_suspend.op.reflect = _reflect; })



struct lpm_model lpm_model_suspend = {
	.flag = LPM_REQ_NONE,
	.op = {
		.prompt = lpm_suspend_system_prompt,
		.reflect = lpm_suspend_system_reflect,
	}
};

#if IS_ENABLED(CONFIG_PM)
static int lpm_spm_suspend_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	struct timespec64 ts;
	struct rtc_time tm;

	ktime_get_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec, &tm);

	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block lpm_spm_suspend_pm_notifier_func = {
	.notifier_call = lpm_spm_suspend_pm_event,
	.priority = 0,
};

#define MTK_LPM_SLEEP_COMPATIBLE_STRING "mediatek,sleep"
static long firmware_pending_result;
module_param(firmware_pending_result,long,0444);
static int spm_irq_number = -1;
module_param(spm_irq_number,int,0444);
static irqreturn_t spm_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}




static int lpm_init_spm_irq(void)
{
 struct device_node *node;struct irq_data *data;int ret;long fw;
 node=of_find_compatible_node(NULL,NULL,MTK_LPM_SLEEP_COMPATIBLE_STRING);
 if(!node)return -ENODEV;
 spm_virq=of_irq_get(node,0);of_node_put(node);
 if(spm_virq<=0)return spm_virq?:-EINVAL;
 data=irq_get_irq_data(spm_virq);if(!data)return -EINVAL;
 spm_irq_number=irqd_to_hwirq(data);
 ret=request_irq(spm_virq,spm_irq_handler,0,"spm-irq",&spm_virq);
 if(ret)return ret;spm_irq_owned=true;
 ret=enable_irq_wake(spm_virq);if(ret)return ret;spm_wake_owned=true;
 fw=lpm_smc_spm(MT_SPM_SMC_UID_SET_PENDING_IRQ_INIT,MT_LPM_SMC_ACT_SET,spm_irq_number,0);
 firmware_pending_result=fw;
 if(fw)return fw<0?fw:-EIO;
 pr_info("[name:spm&][SPM] installed pending irq %d (virq %d)\n",spm_irq_number,spm_virq);
 return 0;
}

#endif

static int lpm_s2idle_barrier(void)
{
	int i;
	struct cpuidle_driver *drv = cpuidle_get_driver();
	unsigned int s2idle_block_value;

	if (!drv || drv->state_count<2 || drv->state_count>CPUIDLE_STATE_MAX)
		return -EINVAL;

	i = drv->state_count - 1;
	if (strcmp(drv->states[i].name, S2IDLE_STATE_NAME))
		return -1;

	/* request PM QoS between S2ilde and mcusys off */
	s2idle_block_value = (drv->states[i].exit_latency +
			      drv->states[i-1].exit_latency)/2;

	cpu_latency_qos_add_request(&lpm_qos_request, s2idle_block_value);
	model_qos_owned=true;

	return 0;
}

void lpm_model_suspend_exit(void)
{
 if(model_owned){lpm_suspend_unregister(model_name,&lpm_model_suspend);model_owned=false;}
#if IS_ENABLED(CONFIG_PM)
 if(model_pm_owned){unregister_pm_notifier(&lpm_spm_suspend_pm_notifier_func);model_pm_owned=false;}
 if(spm_wake_owned){disable_irq_wake(spm_virq);spm_wake_owned=false;}
 if(spm_irq_owned){free_irq(spm_virq,&spm_virq);spm_irq_owned=false;}
#endif
 if(model_qos_owned){cpu_latency_qos_remove_request(&lpm_qos_request);model_qos_owned=false;}
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
 share_mem=NULL;if(md_sleep_base){iounmap(md_sleep_base);md_sleep_base=NULL;}
#endif
 cpumask_clear(&s2idle_cpumask);
}
int lpm_model_suspend_init(void)
{
 int ret;int type=lpm_suspend_type_get();
 cpumask_clear(&s2idle_cpumask);
 if(type==LPM_SUSPEND_S2IDLE){
  model_name="s2idle";
  LPM_SUSPEND_OP_INIT(lpm_suspend_s2idle_prompt,NULL,NULL,lpm_suspend_s2idle_reflect);
  cpus_read_lock();cpuidle_pause_and_lock();
  ret=lpm_s2idle_barrier();
  cpuidle_resume_and_unlock();cpus_read_unlock();if(ret)goto fail;
 } else {
  model_name="suspend";
  LPM_SUSPEND_OP_INIT(lpm_suspend_system_prompt,NULL,NULL,lpm_suspend_system_reflect);
 }
 ret=get_md_sleep_time_addr();if(ret)goto fail;
#if IS_ENABLED(CONFIG_PM)
 ret=register_pm_notifier(&lpm_spm_suspend_pm_notifier_func);if(ret)goto fail;model_pm_owned=true;
 ret=lpm_init_spm_irq();if(ret)goto fail;
#endif
 ret=lpm_suspend_registry(model_name,&lpm_model_suspend);if(ret)goto fail;
 model_owned=true;return 0;
fail:lpm_model_suspend_exit();return ret;
}
