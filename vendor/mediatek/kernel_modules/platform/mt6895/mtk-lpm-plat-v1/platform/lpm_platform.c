// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <lpm.h>
#include <lpm_module.h>
#include <lpm_plat_common.h>
#include <lpm_trace.h>
#include <lpm_irqremain.h>
struct native_irq {u32 hwirq,cat,src;};
struct native_policy {u32 id,value;};
static unsigned int native_config_stage;
module_param(native_config_stage,uint,0444);
static int fw_result(unsigned long v){return (long)v<0 ? (int)(long)v : 0;}
static int read_policies(struct device_node *np,const char *key,struct native_policy *rows,unsigned int count)
{
 unsigned int i,j;int ret;struct device_node *child;
 if(of_count_phandle_with_args(np,key,NULL)!=count)return -EINVAL;
 for(i=0;i<count;i++){
  child=of_parse_phandle(np,key,i);if(!child)return -EINVAL;
  ret=of_property_read_u32(child,"id",&rows[i].id);
  if(!ret)ret=of_property_read_u32(child,"value",&rows[i].value);
  of_node_put(child);if(ret)return ret;
  if(rows[i].id>=count || rows[i].value>1)return -EINVAL;
  for(j=0;j<i;j++)if(rows[j].id==rows[i].id)return -EINVAL;
 }
 return 0;
}
int lpm_platform_trace_get(int type,struct LPM_PLAT_TRACE *trace){return lpm_trace_instance_get(type,trace);}
EXPORT_SYMBOL(lpm_platform_trace_get);
int __init lpm_platform_init(void)
{
 struct device_node *np,*child,*target;struct native_irq irqs[16];struct native_policy resources[5],constraints[4];
 unsigned int i;u32 value[4];int irq,ret,irq_count;struct irq_data *data;
 np=of_find_compatible_node(NULL,NULL,MTK_LPM_DTS_COMPATIBLE);if(!np)return -ENODEV;
 ret=read_policies(np,"resource-ctrl",resources,5);if(ret)goto out;
 ret=read_policies(np,"constraints",constraints,4);if(ret)goto out;
 irq_count=of_count_phandle_with_args(np,"irq-remain",NULL);if(irq_count<1 || irq_count>16){ret=-EINVAL;goto out;}
 for(i=0;i<irq_count;i++){
  child=of_parse_phandle(np,"irq-remain",i);if(!child){ret=-EINVAL;goto out;}
  ret=of_property_read_u32_array(child,"value",value,4);
  target=of_parse_phandle(child,"target",0);of_node_put(child);
  if(ret || !target){of_node_put(target);ret=-EINVAL;goto out;}
  irq=of_irq_get(target,value[1]);of_node_put(target);if(irq<0){ret=irq;goto out;}
  data=irq_get_irq_data(irq);if(!data){ret=-EINVAL;goto out;}
  if(value[0]>1 || value[2]>31){ret=-EINVAL;goto out;}
  /* Native requested type is already encoded by the interrupt provider. */
  irqs[i]=(struct native_irq){irqd_to_hwirq(data),value[2],value[3]};
 }
 /* Map and validate trace SRAM before installing firmware tables. */
 ret=lpm_irqremain_parsing(np);if(ret)goto out;
 ret=lpm_trace_parsing(np);if(ret)goto out;
 native_config_stage=1;
 #define FW_CHECK(call) do{ret=fw_result(call);if(ret)goto out;}while(0)
 FW_CHECK(lpm_smc_cpu_pm(IRQ_REMAIN_LIST_ALLOC,irq_count,0,0));
 FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_ALLOC,MT_LPM_SMC_ACT_SET,irq_count,0));
 for(i=irq_count;i-- > 0;){
  FW_CHECK(lpm_smc_cpu_pm(IRQ_REMAIN_IRQ_ADD,irqs[i].hwirq,irqs[i].cat,irqs[i].src));
  FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_IRQ,MT_LPM_SMC_ACT_SET,irqs[i].hwirq,0));
  FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_WAKEUP_CAT,MT_LPM_SMC_ACT_SET,irqs[i].cat,0));
  FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_WAKEUP_SRC,MT_LPM_SMC_ACT_SET,irqs[i].src,0));
  FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_CTRL,MT_LPM_SMC_ACT_PUSH,0,0));
 }
 FW_CHECK(lpm_smc_cpu_pm(IRQ_REMAIN_IRQ_SUBMIT,0,0,0));
 FW_CHECK(lpm_smc_cpu_pm_lp(IRQS_REMAIN_CTRL,MT_LPM_SMC_ACT_SUBMIT,0,0));
 native_config_stage=2;
 for(i=0;i<5;i++)FW_CHECK(lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_DOE_RESOURCE_CTRL,MT_LPM_SMC_ACT_SET,resources[i].id,resources[i].value));
 for(i=0;i<4;i++)FW_CHECK(lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_DOE_RC,constraints[i].value?MT_LPM_SMC_ACT_SET:MT_LPM_SMC_ACT_CLR,constraints[i].id,0));
 native_config_stage=3;
 #undef FW_CHECK
out:of_node_put(np);if(ret)pr_err("native LPM platform configuration failed stage%u rc%d\n",native_config_stage,ret);return ret;
}
void lpm_platform_deinit(void){lpm_trace_release();lpm_irqremain_list_release();native_config_stage=0;}
