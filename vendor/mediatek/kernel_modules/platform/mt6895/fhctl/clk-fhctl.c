// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Yu-Chang Wang <Yu-Chang.Wang@mediatek.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <dt-bindings/clock/mt6895-clk.h>
extern int register_fh_rate_result(int (*handler)(const char *, unsigned long, int));
#include "clk-fhctl.h"
#include "clk-fhctl-util.h"
#include "clk-mtk.h"

/* Explicit probe/transport-only parameters retain diagnostic modes. */
static bool probe_only;
static bool transport_only;
module_param(transport_only, bool, 0444);
static bool transport_published;
static bool native_transport_ready;
module_param(native_transport_ready, bool, 0444);
static int native_fault;
module_param(native_fault, int, 0444);
module_param(probe_only, bool, 0444);
static bool native_probe_ready;
module_param(native_probe_ready, bool, 0444);
static unsigned int native_pll_count;
module_param(native_pll_count, uint, 0444);
static DEFINE_MUTEX(fh_probe_lock);
static struct device *fh_owner;
static bool _inited;
static bool rate_control_ready;
module_param(rate_control_ready, bool, 0444);
static bool diagnostic_reapply_active;
static bool diagnostic_reapply_passed;
module_param(diagnostic_reapply_passed, bool, 0444);
static struct pll_dts *_array;
static void set_dts_array(struct pll_dts *array) {_array = array; }
static struct pll_dts *get_dts_array(void) {return _array; }

/* -ENOENT means no firmware operation started and native fallback is safe. */
static int fh_rate_result(const char *pll_name, unsigned long dds, int postdiv)
{
 struct pll_dts *array;
 int i, ret;

 if (!pll_name)
  return -EINVAL;
 if (!smp_load_acquire(&_inited))
  return -EAGAIN;
 array = READ_ONCE(_array);
 if (!array || array->num_pll <= 0)
  return -ENODEV;
 for (i = 0; i < array->num_pll; i++) {
  struct pll_dts *pll = &array[i];
  struct fh_hdlr *hdlr;
  if (!pll->pll_name || strcmp(pll_name, pll->pll_name))
   continue;
  if (!(pll->perms & PERM_DRV_HOP))
   return -ENOENT;
  ret = READ_ONCE(native_fault);
  if (ret)
   return ret;
  if (dds > U32_MAX || !dds)
   return -ERANGE;
  if (postdiv <= 0 || postdiv > 128 || (postdiv & (postdiv - 1)))
   return -EINVAL;
  hdlr = pll->hdlr;
  if (!hdlr || !hdlr->ops || !hdlr->ops->hopping || !hdlr->data || !pll->domain)
   return -ENODEV;
  ret = hdlr->ops->hopping(hdlr->data, pll->domain, pll->fh_id, (u32)dds, postdiv);
  return ret <= 0 ? ret : -EPROTO;
 }
 return -ENOENT;
}

/* Match the native xaga contract before any handler can index its tables. */
static bool fh_descriptor_valid(const char *domain, const char *method,
                                const char *name, u32 id, u32 pll_id)
{
 static const char * const top[] = {
  "armpll_ll", "armpll_bl", "armpll_b", "ccipll", NULL, NULL,
  "mpll", "mmpll", "mainpll", "msdcpll", "adsppll", "imgpll", "tvdpll"
 };
 if (!domain || !method || !name || pll_id != 999)
  return false;
 if (!strcmp(domain, "top"))
  return !strcmp(method, FHCTL_MCUPM) && id < ARRAY_SIZE(top) &&
         top[id] && !strcmp(name, top[id]);
 if (strcmp(method, FHCTL_GPUEB) || id)
  return false;
 return (!strcmp(domain, "gpu0") && !strcmp(name, "mfgpll")) ||
        (!strcmp(domain, "gpu2") && !strcmp(name, "mfgpll2")) ||
        (!strcmp(domain, "gpu3") && !strcmp(name, "mfgscpll"));
}

static void fh_iounmap(void *base)
{
 iounmap(base);
}

static void __iomem *fh_map_resource(struct device *dev, unsigned int index,
                                    bool top, bool apmixed)
{
 struct resource resource;
 void __iomem *base;
 resource_size_t minimum = top ? (apmixed ? 0x3b8 : 0x140) :
                                (apmixed ? 0x10 : 0x28);
 int ret = of_address_to_resource(dev->of_node, index, &resource);
 if (ret)
  return ERR_PTR(ret);
 if (resource_size(&resource) < minimum)
  return ERR_PTR(-EINVAL);
 base = ioremap(resource.start, resource_size(&resource));
 if (!base)
  return ERR_PTR(-ENOMEM);
 ret = devm_add_action_or_reset(dev, fh_iounmap, base);
 return ret ? ERR_PTR(ret) : base;
}

static struct pll_dts *parse_dt(struct platform_device *pdev)
{
 struct device *dev = &pdev->dev;
 struct device_node *root = dev->of_node, *map, *of_pll;
 unsigned int count = 0, maps = 0, index = 0, reg_index = 0, i;
 struct pll_dts *array;
 int ret;

 if (!of_device_is_compatible(root, "mediatek,mt6895-fhctl"))
  return ERR_PTR(-ENODEV);
 for_each_available_child_of_node(root, map) {
  maps++;
  for_each_available_child_of_node(map, of_pll)
   count++;
 }
 if (count != 14 || maps != 4)
  return ERR_PTR(-EINVAL);
 array = devm_kcalloc(dev, count, sizeof(*array), GFP_KERNEL);
 if (!array)
  return ERR_PTR(-ENOMEM);
 for_each_available_child_of_node(root, map) {
  const char *domain, *method;
  void __iomem *fh_base, *ap_base;
  ret = of_property_read_string(map, "domain", &domain);
  if (ret)
   goto put_map;
  ret = of_property_read_string(map, "method", &method);
  if (ret)
   goto put_map;
  fh_base = fh_map_resource(dev, reg_index++, !strcmp(domain, "top"), false);
  if (IS_ERR(fh_base)) {
   ret = PTR_ERR(fh_base);
   goto put_map;
  }
  ap_base = fh_map_resource(dev, reg_index++, !strcmp(domain, "top"), true);
  if (IS_ERR(ap_base)) {
   ret = PTR_ERR(ap_base);
   goto put_map;
  }
  for_each_available_child_of_node(map, of_pll) {
   u32 id, pll_id, perms = U32_MAX, ssc_rate = 0;
   struct pll_dts *pll;
   if (index == count) {
    ret = -EOVERFLOW;
    goto put_pll;
   }
   ret = of_property_read_u32(of_pll, "fh-id", &id);
   if (ret)
    goto put_pll;
   ret = of_property_read_u32(of_pll, "pll-id", &pll_id);
   if (ret)
    goto put_pll;
   if (of_find_property(of_pll, "perms", NULL)) {
    ret = of_property_read_u32(of_pll, "perms", &perms);
    if (ret)
     goto put_pll;
   }
   if (of_find_property(of_pll, "ssc-rate", NULL)) {
    ret = of_property_read_u32(of_pll, "ssc-rate", &ssc_rate);
    if (ret)
     goto put_pll;
   }
   if (!fh_descriptor_valid(domain, method, of_pll->name, id, pll_id) ||
       ssc_rate > 8) {
    ret = -EINVAL;
    goto put_pll;
   }
   for (i = 0; i < index; i++) {
    if (!strcmp(array[i].pll_name, of_pll->name) ||
        (!strcmp(array[i].domain, domain) && array[i].fh_id == (int)id)) {
     ret = -EINVAL;
     goto put_pll;
    }
   }
   pll = &array[index++];
   pll->num_pll = count;
   pll->comp = "mediatek,mt6895-fhctl";
   pll->pll_name = devm_kstrdup(dev, of_pll->name, GFP_KERNEL);
   pll->domain = devm_kstrdup(dev, domain, GFP_KERNEL);
   pll->method = devm_kstrdup(dev, method, GFP_KERNEL);
   if (!pll->pll_name || !pll->domain || !pll->method) {
    ret = -ENOMEM;
    goto put_pll;
   }
   pll->fh_id = id;
   pll->pll_id = pll_id;
   pll->perms = perms;
   pll->ssc_rate = ssc_rate;
   pll->fhctl_base = fh_base;
   pll->apmixed_base = ap_base;
  }
 }
 if (index != count)
  return ERR_PTR(-EINVAL);
 return array;
put_pll:
 of_node_put(of_pll);
put_map:
 of_node_put(map);
 return ERR_PTR(ret);
}

bool fhctl_clock_ops_allowed(void)
{
 return !probe_only && READ_ONCE(native_transport_ready) &&
        !READ_ONCE(native_fault) &&
        (!transport_only || READ_ONCE(diagnostic_reapply_active));
}

/* Native global IPI registrations cannot be fully unwound. Keep the bound
 * owner, devres, code and callback storage until reboot after this boundary. */
void fhctl_pin_transport(void)
{
 if (!transport_published) {
  __module_get(THIS_MODULE);
  transport_published = true;
 }
}

static unsigned int mcupm_query_reply, gpueb_query_reply;
static int mcupm_query_result = -ENODATA, gpueb_query_result = -ENODATA;
module_param(mcupm_query_reply, uint, 0444);
module_param(gpueb_query_reply, uint, 0444);
module_param(mcupm_query_result, int, 0444);
module_param(gpueb_query_result, int, 0444);
static int fh_run_query(const char *val, const struct kernel_param *kp)
{
 bool trigger;
 int ret = kstrtobool(val, &trigger);
 if (ret || !trigger)
  return ret ? ret : -EINVAL;
 mutex_lock(&fh_probe_lock);
 if (!native_transport_ready || native_fault) {
  mutex_unlock(&fh_probe_lock);
  return -EAGAIN;
 }
 mcupm_query_result = fhctl_mcupm_query(&mcupm_query_reply);
 gpueb_query_result = fhctl_gpueb_query(&gpueb_query_reply);
 ret = mcupm_query_result ? mcupm_query_result : gpueb_query_result;
 pr_info("FHCTL native read-only TR_ID query: MCUPM=%d/%u GPUEB=%d/%u\n",
          mcupm_query_result, mcupm_query_reply, gpueb_query_result, gpueb_query_reply);
 mutex_unlock(&fh_probe_lock);
 return ret;
}
static const struct kernel_param_ops fh_query_ops = { .set = fh_run_query };
module_param_cb(diagnostic_query, &fh_query_ops, NULL, 0200);

void fhctl_record_fault(int error)
{
 if (error)
  cmpxchg(&native_fault, 0, error < 0 ? error : -EIO);
}

/* This test retains the current DDS and native keep-divider sentinel9999.
 * Hold CCF rate exclusivity on mainpll before touching its firmware path. */
static int fh_reapply_current(const char *val, const struct kernel_param *kp)
{
 struct of_phandle_args args = {};
 struct clk *clk;
 struct pll_dts *pll = NULL;
 bool trigger;
 unsigned int i;
 int ret = kstrtobool(val, &trigger);
 unsigned long before, after;
 if (ret || !trigger)
  return ret ? ret : -EINVAL;
 mutex_lock(&fh_probe_lock);
 if (!native_transport_ready || native_fault || rate_control_ready) {
  ret = -EAGAIN;
  goto unlock;
 }
 for (i = 0; i < native_pll_count; i++)
  if (!strcmp(_array[i].pll_name, "mainpll"))
   pll = &_array[i];
 if (!pll || !pll->hdlr || !pll->hdlr->ops->reapply_current) {
  ret = -ENODEV;
  goto unlock;
 }
 args.np = of_find_compatible_node(NULL, NULL, "mediatek,mt6895-apmixedsys");
 if (!args.np) {
  ret = -ENODEV;
  goto unlock;
 }
 args.args_count = 1;
 args.args[0] = CLK_APMIXED_MAINPLL;
 clk = of_clk_get_from_provider(&args);
 of_node_put(args.np);
 if (IS_ERR(clk)) {
  ret = PTR_ERR(clk);
  goto unlock;
 }
 ret = clk_rate_exclusive_get(clk);
 if (!ret) {
  before = clk_get_rate(clk);
  WRITE_ONCE(diagnostic_reapply_active, true);
  ret = pll->hdlr->ops->reapply_current(pll->hdlr->data);
  WRITE_ONCE(diagnostic_reapply_active, false);
  after = clk_get_rate(clk);
  if (!ret && (!before || before != after)) {
   ret = -EIO;
   fhctl_record_fault(ret);
  }
  if (!ret)
   WRITE_ONCE(diagnostic_reapply_passed, true);
  pr_info("FHCTL current-mainpll reapply ret=%d before=%lu after=%lu\n", ret, before, after);
  clk_rate_exclusive_put(clk);
 }
 clk_put(clk);
unlock:
 mutex_unlock(&fh_probe_lock);
 return ret;
}
static const struct kernel_param_ops fh_reapply_ops = { .set = fh_reapply_current };
module_param_cb(diagnostic_reapply, &fh_reapply_ops, NULL, 0200);

static int fh_enable_rate_control(const char *val, const struct kernel_param *kp)
{
 bool trigger;
 int ret = kstrtobool(val, &trigger);
 if (ret || !trigger)
  return ret ? ret : -EPERM;
 mutex_lock(&fh_probe_lock);
 if (rate_control_ready) {
  ret = 0;
  goto out;
 }
 if (!native_transport_ready || native_fault || !diagnostic_reapply_passed) {
  ret = -EAGAIN;
  goto out;
 }
 /* Transport already pinned all callback-owned code and resources. */
 transport_only = false;
 smp_store_release(&_inited, true);
 ret = register_fh_rate_result(fh_rate_result);
 if (ret) {
  smp_store_release(&_inited, false);
  transport_only = true;
 } else {
  WRITE_ONCE(rate_control_ready, true);
 }
out:
 mutex_unlock(&fh_probe_lock);
 return ret;
}
static const struct kernel_param_ops fh_enable_ops = { .set = fh_enable_rate_control };
module_param_cb(enable_rate_control, &fh_enable_ops, NULL, 0200);

static int fh_plt_drv_probe(struct platform_device *pdev)
{
 struct pll_dts *array;
 int i, ret;

 mutex_lock(&fh_probe_lock);
 if (fh_owner) {
  mutex_unlock(&fh_probe_lock);
  return -EBUSY;
 }
 WRITE_ONCE(native_fault, 0); /* Only prepublication retries reach here. */
 array = parse_dt(pdev);
 if (IS_ERR(array)) {
  mutex_unlock(&fh_probe_lock);
  return PTR_ERR(array);
 }
 set_dts_array(array);
 fh_owner = &pdev->dev;
 native_pll_count = array->num_pll;
 WRITE_ONCE(native_probe_ready, true);
 if (probe_only) {
  dev_info(&pdev->dev, "native probe-only: %u PLLs; no IPI/SSC/rate hook or register writes\n", native_pll_count);
  mutex_unlock(&fh_probe_lock);
  return 0;
 }
 ret = fhctl_mcupm_init(array);
 if (!ret)
  ret = fhctl_gpueb_init(array);
 if (!ret) {
  for (i = 0; i < native_pll_count; i++) {
   if (!array[i].hdlr || !array[i].hdlr->data || !array[i].hdlr->ops) {
    ret = -ENODEV;
    break;
   }
  }
 }
 if (ret) {
  WRITE_ONCE(native_fault, ret);
  if (transport_published) {
   /* Return success only to retain global callback-owned resources.
    * Readiness stays false, fault is exposed; no rate hook is published. */
   dev_err(&pdev->dev, "transport fault %d, quarantined until reboot\n", ret);
   mutex_unlock(&fh_probe_lock);
   return 0;
  }
  set_dts_array(NULL);
  fh_owner = NULL;
  native_pll_count = 0;
  WRITE_ONCE(native_probe_ready, false);
  mutex_unlock(&fh_probe_lock);
  return ret;
 }
 WRITE_ONCE(native_transport_ready, true);
 if (!transport_only) {
  /* Native board tables/transport state are complete before publication.
   * Production startup attaches the qualified callback, not a test cycle. */
  smp_store_release(&_inited, true);
  ret = register_fh_rate_result(fh_rate_result);
  if (ret) {
   smp_store_release(&_inited, false);
   fhctl_record_fault(ret);
   dev_err(&pdev->dev, "rate hook registration failed %d, state retained\n", ret);
   mutex_unlock(&fh_probe_lock);
   return 0;
  }
  WRITE_ONCE(rate_control_ready, true);
 }
 dev_info(&pdev->dev, "native FHCTL: %u handlers, rate control=%u, SSC disabled\n", native_pll_count, rate_control_ready);
 mutex_unlock(&fh_probe_lock);
 return 0;
}

static void fh_plt_drv_remove(struct platform_device *pdev)
{
 mutex_lock(&fh_probe_lock);
 if (fh_owner == &pdev->dev && !transport_published) {
  WRITE_ONCE(native_probe_ready, false);
  native_pll_count = 0;
  set_dts_array(NULL);
  fh_owner = NULL;
 }
 mutex_unlock(&fh_probe_lock);
}

static void fh_plt_drv_shutdown(struct platform_device *pdev)
{
	struct pll_dts *array = get_dts_array();
	int num_pll;
	int i;

	if (!smp_load_acquire(&_inited) || !array)
		return;
	num_pll = array->num_pll;

	for (i = 0; i < num_pll; i++, array++) {
		struct fh_hdlr *hdlr = array->hdlr;

		if (array->ssc_rate)
			hdlr->ops->ssc_disable(hdlr->data,
					array->domain,
					array->fh_id);
	}
}

static const struct of_device_id fh_of_match[] = {
	{ .compatible = "mediatek,mt6853-fhctl"},
	{ .compatible = "mediatek,mt6855-fhctl"},
	{ .compatible = "mediatek,mt6877-fhctl"},
	{ .compatible = "mediatek,mt6873-fhctl"},
	{ .compatible = "mediatek,mt6879-fhctl"},
	{ .compatible = "mediatek,mt6885-fhctl"},
	{ .compatible = "mediatek,mt6895-fhctl"},
	{ .compatible = "mediatek,mt6983-fhctl"},
	{}
};

static struct platform_driver fhctl_driver = {
	.probe = fh_plt_drv_probe,
	.remove = fh_plt_drv_remove,
	.shutdown = fh_plt_drv_shutdown,
	.driver = {
		.name = "fhctl",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
		.of_match_table = fh_of_match,
	},
};
module_platform_driver(fhctl_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek FHCTL Driver");
MODULE_AUTHOR("Kuan-Hsin Lee <kuan-hsin.lee@mediatek.com>");
