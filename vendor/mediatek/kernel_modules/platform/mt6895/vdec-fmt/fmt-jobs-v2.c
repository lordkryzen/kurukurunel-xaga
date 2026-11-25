static int fmt_native_finish_task(unsigned int taskid, struct file *owner, bool wait)
{
 struct mtk_vdec_fmt *fmt = fmt_mtkdev;
 struct gce_cmdq_task *task;
 unsigned int id, i;
 int ret = 0;
 if (taskid >= FMT_INST_MAX) return -EINVAL;
 task = &fmt->gce_task[taskid];
 if (!task->used || !task->pkt_ptr || task->owner != owner) return -EINVAL;
 id = task->identifier;
 if (id >= fmt->gce_th_num) return -EINVAL;
 if (wait) ret = cmdq_pkt_wait_complete(task->pkt_ptr);
 /* wait_complete synchronizes completion/error callback before destruction. */
 cmdq_pkt_destroy(task->pkt_ptr);
 fmt_clear_gce_task(taskid);
 task->owner = NULL;
 atomic_dec(&fmt->gce_job_cnt[id]);
 if (!atomic_read(&fmt->gce_job_cnt[0]) && !atomic_read(&fmt->gce_job_cnt[1])) {
  for (i = 0; i < fmt->gce_th_num; i++) fmt_end_dvfs_emi_bw(fmt, i);
  fmt_clock_off(fmt);
 }
 if (!atomic_read(&fmt->gce_job_cnt[id])) fmt_unlock(id);
 return ret;
}

static int fmt_native_submit(struct gce_cmdq_obj *buff, struct file *owner)
{
 struct mtk_vdec_fmt *fmt = fmt_mtkdev;
 struct cmdq_pkt *pkt = NULL;
 struct dmabuf_info in = {0}, out = {0};
 struct dmabufmap map[FMT_FD_RESERVE];
 struct gce_cmds *cmds;
 unsigned int id = buff->identifier, i;
 int ret, taskid;
 bool acquired = false, powered = false;
 if (id >= fmt->gce_th_num || buff->secure ||
     !buff->pmqos_param.pixel_size) return -EOPNOTSUPP;
 if (atomic_read(&fmt->fmt_error)) return -EIO;
 if (READ_ONCE(fmt->is_entering_suspend)) return -EBUSY;
 if (!fmt->clt_fmt[id]) return -ENODEV;
 if (!access_ok(buff->taskid, sizeof(u32))) return -EFAULT;
 cmds = fmt->gce_cmds[id];
 if (copy_from_user(cmds, u64_to_user_ptr(buff->cmds_user_ptr), sizeof(*cmds))) return -EFAULT;
 if (!cmds->cmd_cnt || cmds->cmd_cnt > FMT_CMDQ_CMD_MAX) return -EINVAL;
 for (i = 0; i < FMT_FD_RESERVE; i++) {map[i].fd = -1; map[i].iova = 0;}
 ret = fmt_lock(id, false);
 if (ret) return -EBUSY;
 acquired = !atomic_read(&fmt->gce_job_cnt[id]);
 pkt = cmdq_pkt_create(fmt->clt_fmt[id]);
 if (IS_ERR_OR_NULL(pkt)) {ret = pkt ? PTR_ERR(pkt) : -ENOMEM; pkt = NULL; goto fail;}
 for (i = 0; i < cmds->cmd_cnt; i++) {
  ret = fmt_set_gce_cmd(pkt, cmds->cmd[i], cmds->addr[i], cmds->data[i],
   cmds->mask[i], fmt->gce_gpr[id], id, &in, &out, map, false);
  if (ret) goto fail;
 }
 if (!atomic_read(&fmt->gce_job_cnt[0]) && !atomic_read(&fmt->gce_job_cnt[1])) {
  ret = fmt_clock_on(fmt);
  if (ret) goto fail;
  powered = true;
 }
 taskid = fmt_set_gce_task(pkt, id, in, out);
 if (taskid < 0) {ret = -ENOSPC; goto fail;}
 fmt->gce_task[taskid].owner = owner;
 memcpy(&fmt->gce_task[taskid].cmdq_buff, buff, sizeof(*buff));
 atomic_inc(&fmt->gce_job_cnt[id]);
 fmt_start_dvfs_emi_bw(fmt, buff->pmqos_param, id);
 pkt->aee_cb = fmt_gce_timeout_aee;
 ret = cmdq_pkt_flush_async(pkt, fmt_gce_flush_callback, pkt);
 if (ret) {fmt_native_finish_task(taskid, owner, false); return ret;}
 if (copy_to_user(buff->taskid, &taskid, sizeof(taskid))) {
  fmt_native_finish_task(taskid, owner, true); return -EFAULT;
 }
 return 0;
fail:
 if (powered) fmt_clock_off(fmt);
 if (pkt) cmdq_pkt_destroy(pkt);
 fmt_dmabuf_free_iova(in.dbuf, in.attach, in.sgt);
 fmt_dmabuf_free_iova(out.dbuf, out.attach, out.sgt);
 fmt_dmabuf_put(in.dbuf); fmt_dmabuf_put(out.dbuf);
 if (acquired) fmt_unlock(id);
 return ret;
}

static int fmt_gce_cmd_flush(unsigned long arg, struct file *owner)
{
 struct gce_cmdq_obj buff;
 if (copy_from_user(&buff, (void __user *)arg, sizeof(buff))) return -EFAULT;
 return fmt_native_submit(&buff, owner);
}

static int fmt_gce_wait_callback(unsigned long arg, struct file *owner)
{
 unsigned int id;
 if (copy_from_user(&id, (void __user *)arg, sizeof(id))) return -EFAULT;
 return fmt_native_finish_task(id, owner, true);
}
