static void fmt_native_put_larb(void *p) {put_device(p);}

static int vdec_fmt_probe(struct platform_device *pdev)
{
 static const char *events[GCE_EVENT_MAX] = {
  "rdma0-sw-rst-done-eng", "rdma0-tile-done", "wdma0-sw-rst-done-eng", "wdma0-tile-done",
  "rdma1-sw-rst-done-eng", "rdma1-tile-done", "wdma1-sw-rst-done-eng", "wdma1-tile-done"
 };
 struct device *dev = &pdev->dev;
 struct mtk_vdec_fmt *fmt;
 struct device_node *node;
 struct platform_device *larb;
 struct resource *res;
 unsigned int i;
 u32 wa = 0;
 int ret;
 if (fmt_mtkdev) return -EBUSY;
 fmt = devm_kzalloc(dev, sizeof(*fmt), GFP_KERNEL);
 if (!fmt) return -ENOMEM;
 fmt->dev = dev;
 ret = of_property_read_string(dev->of_node, "mediatek,fmtname", &fmt->fmtname);
 if (ret) return ret;
 if (strcmp(fmt->fmtname, VDEC_FMT_DEVNAME)) return -EINVAL;
 ret = of_property_read_u32(dev->of_node, "mediatek,fmt-gce-th-num", &fmt->gce_th_num);
 if (ret || !fmt->gce_th_num || fmt->gce_th_num > FMT_CORE_NUM) return -EINVAL;
 fmt->dtsInfo.pipeNum = fmt->gce_th_num;
 ret = of_property_read_u32(dev->of_node, "mediatek,fmt-rdma-swwa", &wa);
 if (ret && of_find_property(dev->of_node, "mediatek,fmt-rdma-swwa", NULL)) return ret;
 fmt->dtsInfo.RDMA_needWA = !!wa;
 for (i = 0; i < FMT_MAP_HW_REG_NUM; i++) {
  res = platform_get_resource(pdev, IORESOURCE_MEM, i);
  if (!res) break;
  fmt->map_base[i].base = res->start;
  fmt->map_base[i].len = resource_size(res);
  fmt->map_base[i].va = (unsigned long)devm_ioremap(dev, res->start, resource_size(res));
  if (!fmt->map_base[i].va) return -ENOMEM;
 }
 if (i < fmt->gce_th_num * 2) return -EINVAL;
 fmt->dtsInfo.RDMA_baseAddr = fmt->map_base[0].base;
 fmt->dtsInfo.WROT_baseAddr = fmt->map_base[1].base;
 for (i = 0; i < GCE_EVENT_MAX; i++) {
  ret = of_property_read_u16(dev->of_node, events[i], &fmt->gce_codec_eid[i]);
  if (ret) return ret;
 }
 ret = of_property_read_u32(dev->of_node, "gce-gpr", &fmt->gce_gpr[0]);
 if (ret) return ret;
 for (i = 1; i < fmt->gce_th_num; i++) {
  ret = of_property_read_u32_index(dev->of_node, "gce-gpr", i, &fmt->gce_gpr[i]);
  if (ret) fmt->gce_gpr[i] = fmt->gce_gpr[0]; /* Native DT has one shared scratch register. */
 }
 for (i = 0; i < FMT_PORT_NUM; i++) {
  ret = of_property_read_u32_index(dev->of_node, "m4u-ports", i, &wa);
  if (ret) return ret;
  fmt->fmt_m4u_ports[i] = wa;
 }
 ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
 if (ret) return ret;
 node = of_parse_phandle(dev->of_node, "mediatek,larbs", 0);
 if (!node) return -EINVAL;
 larb = of_find_device_by_node(node); of_node_put(node);
 if (!larb) return -EPROBE_DEFER;
 ret = devm_add_action_or_reset(dev, fmt_native_put_larb, &larb->dev);
 if (ret) return ret;
 if (!device_is_bound(&larb->dev)) return -EPROBE_DEFER;
 fmt->fmtLarb = &larb->dev;
 fmt->clk_VDEC = devm_clk_get(dev, "MT_CG_VDEC");
 if (IS_ERR(fmt->clk_VDEC)) return PTR_ERR(fmt->clk_VDEC);
 fmt->clk_MINI_MDP = devm_clk_get(dev, "MT_CG_MINI_MDP");
 if (IS_ERR(fmt->clk_MINI_MDP)) return PTR_ERR(fmt->clk_MINI_MDP);
 ret = fmt_native_prepare_qos(fmt);
 if (ret) return ret;
 fmt->clt_base = cmdq_register_device(dev);
 if (!fmt->clt_base) {ret = -ENOMEM; goto qos;}
 for (i = 0; i < fmt->gce_th_num; i++) {
  fmt->gce_cmds[i] = devm_kzalloc(dev, sizeof(struct gce_cmds), GFP_KERNEL);
  fmt->mux_gce_th[i] = devm_kzalloc(dev, sizeof(struct mutex), GFP_KERNEL);
  fmt->mux_cmdq_pkt[i] = devm_kzalloc(dev, sizeof(struct mutex), GFP_KERNEL);
  if (!fmt->gce_cmds[i] || !fmt->mux_gce_th[i] || !fmt->mux_cmdq_pkt[i]) {ret = -ENOMEM; goto clients;}
  mutex_init(fmt->mux_gce_th[i]); mutex_init(fmt->mux_cmdq_pkt[i]);
  sema_init(&fmt->fmt_sem[i], 1);
  fmt->clt_fmt[i] = cmdq_mbox_create(dev, i);
  if (IS_ERR_OR_NULL(fmt->clt_fmt[i])) {
   ret = fmt->clt_fmt[i] ? PTR_ERR(fmt->clt_fmt[i]) : -ENODEV;
   fmt->clt_fmt[i] = NULL; goto clients;
  }
 }
 mutex_init(&fmt->mux_fmt); mutex_init(&fmt->mux_task); mutex_init(&fmt->mux_active_time);
 fmt->cmdq_cb_workqueue = alloc_ordered_workqueue(VDEC_FMT_DEVNAME, WQ_MEM_RECLAIM | WQ_FREEZABLE);
 if (!fmt->cmdq_cb_workqueue) {ret = -ENOMEM; goto clients;}
 INIT_WORK(&fmt->cmdq_cb_work, fmt_active_resource_time_check);
 fmt->native_larb_link = device_link_add(dev, fmt->fmtLarb, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
 if (!fmt->native_larb_link) {ret = -ENOMEM; goto work;}
 pm_runtime_enable(dev);
 platform_set_drvdata(pdev, fmt);
 fmt->pm_notifier.notifier_call = vdec_fmt_suspend_notifier;
 /* Callbacks are registered only after all their state exists. */
 WRITE_ONCE(fmt_mtkdev, fmt);
 ret = register_pm_notifier(&fmt->pm_notifier);
 if (ret) goto global;
 for (i = 0; i < FMT_PORT_NUM; i++) {
  ret = mtk_iommu_register_fault_callback(fmt->fmt_m4u_ports[i],
   (mtk_iommu_fault_callback_t)fmt_translation_fault_callback, fmt, false);
  if (ret) goto callbacks;
  fmt->native_fault_count++;
 }
 ret = alloc_chrdev_region(&fmt->fmt_devno, 0, 1, fmt->fmtname);
 if (ret) goto callbacks;
 fmt->fmt_cdev = cdev_alloc();
 if (!fmt->fmt_cdev) {ret = -ENOMEM; goto devno;}
 fmt->fmt_cdev->owner = THIS_MODULE; fmt->fmt_cdev->ops = &vdec_fmt_fops;
 ret = cdev_add(fmt->fmt_cdev, fmt->fmt_devno, 1);
 if (ret) {kobject_put(&fmt->fmt_cdev->kobj); goto devno;}
 fmt->fmt_class = class_create(fmt->fmtname);
 if (IS_ERR(fmt->fmt_class)) {ret = PTR_ERR(fmt->fmt_class); goto cdev;}
 fmt->fmt_device = device_create(fmt->fmt_class, NULL, fmt->fmt_devno, NULL, "%s", fmt->fmtname);
 if (IS_ERR(fmt->fmt_device)) {ret = PTR_ERR(fmt->fmt_device); goto class;}
 /* Qualified callback owner remains resident; runtime unbind is disabled. */
 __module_get(THIS_MODULE);
 WRITE_ONCE(fmt->native_ready, true);
 cmdq_get_usage_cb(fmt->clt_fmt[0]->chan, cmdq_usage_cb_func);
 fmt_debug(0, "initialization completed pipes=%u rdma_wa=%u", fmt->gce_th_num, fmt->dtsInfo.RDMA_needWA);
 return 0;
class: class_destroy(fmt->fmt_class);
cdev: cdev_del(fmt->fmt_cdev);
devno: unregister_chrdev_region(fmt->fmt_devno, 1);
callbacks:
 while (fmt->native_fault_count) mtk_iommu_unregister_fault_callback(fmt->fmt_m4u_ports[--fmt->native_fault_count], false);
 unregister_pm_notifier(&fmt->pm_notifier);
global:
 WRITE_ONCE(fmt_mtkdev, NULL); platform_set_drvdata(pdev, NULL); pm_runtime_disable(dev);
 device_link_del(fmt->native_larb_link);
work: destroy_workqueue(fmt->cmdq_cb_workqueue);
clients:
 for (i = 0; i < FMT_CORE_NUM; i++) if (fmt->clt_fmt[i]) cmdq_mbox_destroy(fmt->clt_fmt[i]);
qos: fmt_native_unprepare_qos(fmt);
 return ret;
}
