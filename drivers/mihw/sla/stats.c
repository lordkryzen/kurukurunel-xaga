/*
    Copyright (c) 2018, The Linux Foundation. All rights reserved.

    This program is free software; you can redistribute it and/or modify it under the terms
    of the GNU General Public License version 2 and only version 2 as published by the Free
    Software Foundation.

    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.
*/
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <linux/version.h>
#include <linux/skbuff.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/hash.h>
#include <linux/rculist.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <linux/ktime.h>

#include <linux/capability.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <net/ip.h>
#include <net/netfilter/nf_socket.h>
#include <net/sock.h>
#include "sla_stats.h"

static DEFINE_SPINLOCK(stats_spin_lock);
static DEFINE_MUTEX(config_mutex);
static char *proc_name = "sla";
module_param(proc_name, charp, 0444);
static int test_ifindex;
module_param(test_ifindex, int, 0444);
static unsigned int test_uid;
module_param(test_uid, uint, 0444);
static bool exiting;

/* about fs config */
#define USE_PROCFS

/* about stats */
#define STATS_BUF_SIZE (sizeof(struct sla_interface_stats)) //16*(1+ENTRY_TOTAL) = (20*1024)

#define MMAP_FILE_SUFFIX "_mmap"
#define TOTAL_STATS_FILE_SUFFIX "_stats"
#define VOIP_STATS_FILE "sla_voip_state"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
/*
 * Commit 359745d78351 ("proc: remove PDE_DATA() completely")
 * Replaced PDE_DATA() with pde_data()
 */
#define pde_data(inode) PDE_DATA(inode)
#endif

static struct sla_config config_data;
static struct sla_voip_state voip_state;
static u64 pkt_alloc_num = 0;
static u64 pkt_free_num = 0;


LIST_HEAD(sla_iface_list) ;


/* about port hash list */
#define PORT_HASH_BITS 8
#define PORT_HASH_SIZE (1<<PORT_HASH_BITS)
#define MAX_SIZE_DEFAULT    (4*1024)
#define LIMIT_TIME_DEFAULT  (5)

#define STATS_LOCK_IRQ spin_lock_irqsave
#define STATS_UNLOCK_IRQ spin_unlock_irqrestore

struct uplink_pkt
{
    struct hlist_node   hlist;
    u16 local_port;
};

struct hlist_head   pkt_list[PORT_HASH_SIZE];


/* about fs */

static void sla_interface_release(struct kref *ref)
{
    struct sla_interface *iface = container_of(ref, struct sla_interface, refs);

    vfree(iface->mmap_struct);
    kfree(iface);
}

static int traffic_show(struct seq_file *seq, void *unused)
{
    struct sla_interface *iface = seq->private;
    unsigned long flags;
    u64 value;

    spin_lock_irqsave(&stats_spin_lock, flags);
    value = iface->total_trfc;
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    seq_printf(seq, "%llu\n", value);
    return 0;
}

static int traffic_open(struct inode *inode, struct file *file)
{
    struct sla_interface *iface = pde_data(inode);
    int ret;

    kref_get(&iface->refs);
    ret = single_open(file, traffic_show, iface);
    if (ret)
        kref_put(&iface->refs, sla_interface_release);
    return ret;
}

static ssize_t traffic_write(struct file *file, const char __user *buffer,
                            size_t count, loff_t *position)
{
    struct seq_file *seq = file->private_data;
    struct sla_interface *iface = seq->private;
    unsigned long flags;
    u64 value;
    int ret = kstrtou64_from_user(buffer, count, 0, &value);

    if (ret)
        return ret;
    spin_lock_irqsave(&stats_spin_lock, flags);
    iface->total_trfc = value;
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    return count;
}

static int traffic_release(struct inode *inode, struct file *file)
{
    struct seq_file *seq = file->private_data;
    struct sla_interface *iface = seq->private;
    int ret = single_release(inode, file);

    kref_put(&iface->refs, sla_interface_release);
    return ret;
}

static const struct proc_ops fops_u64 = {
    .proc_open = traffic_open,
    .proc_release = traffic_release,
    .proc_read = seq_read,
    .proc_write = traffic_write,
    .proc_lseek = seq_lseek,
};

/* dir */
static struct proc_dir_entry *sla_dir;
/* config file */
static struct proc_dir_entry *config_file; //Used for SLA config: enable, prots, reset_traffic and so on

static struct sla_interface * sla_find_interface(const char *interface_name);
/* sla voip file */
static struct proc_dir_entry *sla_voip_file;

/* about lock */


static inline void stats_lock_init(void)
{
    spin_lock_init(&stats_spin_lock);
}

static inline void stats_lock(void)
{
    spin_lock(&stats_spin_lock);
}

static inline void stats_unlock(void)
{
    spin_unlock(&stats_spin_lock);
}

static spinlock_t pkt_spin_lock;

static inline void pkt_lock_init(void)
{
    spin_lock_init(&pkt_spin_lock);
}

static spinlock_t list_spin_lock;

static inline void list_lock_init(void)
{
    spin_lock_init(&list_spin_lock);
}

static inline struct hlist_head *pkt_hlist_head(u16 port)
{
    return &pkt_list[hash_32(port, PORT_HASH_BITS)];
}

static struct uplink_pkt *pkt_find(u16 port)
{
    struct uplink_pkt *pkt;

    hlist_for_each_entry(pkt, pkt_hlist_head(port), hlist)
    {
        if (pkt->local_port == port)
            return pkt;
    }

    return NULL;
}

static void pkt_add(u16 port)
{
    struct uplink_pkt *pkt;

    if (pkt_find(port))
        return;
    pkt = kzalloc(sizeof(*pkt), GFP_ATOMIC);
    if (!pkt)
        return;
    pkt->local_port = port;
    pkt_alloc_num++;
    hlist_add_head(&pkt->hlist, pkt_hlist_head(port));
}

static void pkt_release(struct uplink_pkt *pkt)
{
    pkt_free_num++;
    hlist_del(&pkt->hlist);
    kfree(pkt);
}

static void pkt_hlist_init(void)
{
    unsigned int h;

    for (h = 0; h < PORT_HASH_SIZE; h++)
        INIT_HLIST_HEAD(&pkt_list[h]);
}

static void pkt_hlist_deinit(void)
{
    struct uplink_pkt *pkt;
    struct hlist_node *next;
    unsigned int h;

    for (h = 0; h < PORT_HASH_SIZE; h++)
        hlist_for_each_entry_safe(pkt, next, &pkt_list[h], hlist)
            pkt_release(pkt);
}

static void pkt_hlist_print(void)
{
    struct uplink_pkt *pkt;
    unsigned int h;

    for (h = 0; h < PORT_HASH_SIZE; h++)
        hlist_for_each_entry(pkt, &pkt_list[h], hlist)
            pr_info("sla: port %u\n", pkt->local_port);
}

static bool sla_pkt_valid(struct sk_buff *skb, u8 link_type)
{
    // int i;
    u16 local_port;
    u16 ext_port;
    u16 len;
    struct tcphdr *tcph;
    struct uplink_pkt *pkt;

    if(IPPROTO_TCP != ip_hdr(skb)->protocol)
        return false;

    if(LT_UPLINK == link_type)
    {
        local_port = ntohs(tcp_hdr(skb)->source);
        ext_port = ntohs(tcp_hdr(skb)->dest);
    }
    else
    {
        local_port = ntohs(tcp_hdr(skb)->dest);
        ext_port = ntohs(tcp_hdr(skb)->source);
    }

    // for(i=0; (i<PORTS_MAX) && (0!=config_data.ports[i]); i++)
    // {
    //     if(ext_port == config_data.ports[i])
    //         break;
    // }

    // if((i == PORTS_MAX) || ext_port != config_data.ports[i])
    // {
    //     return false;
    // }

    len = ntohs(ip_hdr(skb)->tot_len) - (ip_hdr(skb)->ihl*4 + tcp_hdrlen(skb));
    if(0 == len)
    {
        tcph = tcp_hdr(skb);
        if(tcph->fin || tcph->rst)
        {
            pkt = pkt_find(local_port);
            if(pkt)
            {
                pkt_release(pkt);
            }
        }
        return false;
    }

    return true;
}

static bool sla_pkt_consolidate(struct sla_interface_stats *stats, u32 index)
{
    u32 prev;
    struct sla_stats_entry* entry = &stats->entries[index];

    if(0 == index)
        prev = ENTRY_TOTAL - 1;
    else
        prev = index - 1;

    if(stats->entries[prev].discard)
        return false;

    if(stats->entries[prev].port != entry->port)
        return false;

    if (READ_ONCE(stats->entries[prev].timestamp) > entry->timestamp ||
        entry->timestamp - READ_ONCE(stats->entries[prev].timestamp) > config_data.limit_time)
        return false;

    if (READ_ONCE(stats->entries[prev].size) > config_data.max_size ||
        entry->size > config_data.max_size - READ_ONCE(stats->entries[prev].size))
        return false;

    stats->entries[prev].size += entry->size;

    return true;
}

static inline bool sla_voip_pack_filter(struct sk_buff *skb)
{
    if (ip_hdr(skb)->protocol == IPPROTO_UDP && !IS_ERR_OR_NULL(skb->sk)) {
        uid_t voip_uid = __kuid_val(skb->sk->sk_uid);
        if (config_data.voip_uid == voip_uid && voip_state.address == 0) {
            __be16 port = udp_hdr(skb)->dest;
            // Exclude port 53 of udp
            if (port != 13568) {
                voip_state.address = ip_hdr(skb)->daddr;
                voip_state.port = port;
            }
        }
            return true;
    }
    return false;
}

static bool sla_prepare_packet(struct sk_buff *skb)
{
    struct iphdr *ip;
    unsigned int offset = skb_network_offset(skb), ip_length, header_length;

    if (skb_network_offset(skb) < 0 ||
        !pskb_may_pull(skb, offset + sizeof(*ip)))
        return false;
    ip = ip_hdr(skb);
    if (ip->version != 4 || ip->ihl < 5 || ip_is_fragment(ip))
        return false;
    ip_length = ip->ihl * 4;
    if (ntohs(ip->tot_len) < ip_length ||
        ntohs(ip->tot_len) > skb->len - offset)
        return false;
    if (ip->protocol == IPPROTO_TCP)
        header_length = sizeof(struct tcphdr);
    else if (ip->protocol == IPPROTO_UDP)
        header_length = sizeof(struct udphdr);
    else
        return false;
    if (ntohs(ip->tot_len) < ip_length + header_length ||
        !pskb_may_pull(skb, offset + ip_length + header_length))
        return false;
    skb_set_transport_header(skb, offset + ip_length);
    ip = ip_hdr(skb);
    if (ip->protocol == IPPROTO_TCP) {
        header_length = tcp_hdr(skb)->doff * 4;
        if (header_length < sizeof(struct tcphdr) ||
            ntohs(ip->tot_len) < ip_length + header_length ||
            !pskb_may_pull(skb, offset + ip_length + header_length))
            return false;
    }
    return true;
}

static bool sla_test_owner(struct sk_buff *skb, const struct nf_hook_state *state)
{
    struct sock *sk;
    bool matched;

    if (!test_uid)
        return true;
    if (state->hook == NF_INET_PRE_ROUTING) {
        sk = nf_sk_lookup_slow_v4(state->net, skb, state->in);
    } else {
        sk = skb_to_full_sk(skb);
        if (sk)
            sock_hold(sk);
    }
    if (!sk)
        return false;
    matched = sk_fullsock(sk) && __kuid_val(sock_i_uid(sk)) == test_uid;
    sock_gen_put(sk);
    return matched;
}

static unsigned int sla_ipv4_in(void *priv,
                                struct sk_buff *skb,
                                const struct nf_hook_state *state)
{
    struct sla_interface_stats *p;
    struct sla_stats_entry *entry;
    struct timespec64 tm;
    struct uplink_pkt *pkt;
    bool consolidated = false;
    struct sla_interface* iface = NULL;
    unsigned long flags;

    if (!READ_ONCE(config_data.enable) || !state->in || (test_ifindex && state->in->ifindex != test_ifindex) ||
        (!sla_prepare_packet(skb) || !sla_test_owner(skb, state)))
        return NF_ACCEPT;
    STATS_LOCK_IRQ(&stats_spin_lock,flags);
    if(config_data.enable && config_data.rate_on)
    {
        if(!sla_pkt_valid(skb, LT_DOWNLINK))
            goto unlock;

        //printk("#### SLA sla_ipv4_in ");
        //printk("#### %s devname=[%s] mac len=%d skb->len=%d ifaceid = ((%d)) \n", __func__, state->in->name, skb->mac_len, skb->len, state->in->ifindex);

        iface =  sla_find_interface(state->in->name);

        if(iface == NULL)
        {
            goto unlock;
        }
        p = iface->mmap_struct;

        iface->total_trfc += skb->len;

        ktime_get_real_ts64(&tm);
        entry = &p->entries[iface->write_index];
        entry->link_type = LT_DOWNLINK;
        entry->timestamp = tm.tv_sec*1000 + tm.tv_nsec/1000000;
        entry->size = skb->len + 14; //14 is mac len
        entry->port = ntohs(tcp_hdr(skb)->dest);

        pkt = pkt_find(entry->port);
        if(pkt)
        {
            pkt_release(pkt);

            entry->discard = 1;
        }
        else
        {
            entry->discard = 0;
            consolidated = sla_pkt_consolidate(p, iface->write_index);
        }

        if(!consolidated)
        {
            iface->write_index = (iface->write_index + 1) % ENTRY_TOTAL;
            p->magic_num = STATS_MAGIC;
            p->total = ENTRY_TOTAL;
            smp_wmb();
            WRITE_ONCE(p->cur_idx, iface->write_index);
        }
        else
            memset(entry, 0, sizeof(*entry));

    }
unlock:
    STATS_UNLOCK_IRQ(&stats_spin_lock,flags);
    return NF_ACCEPT;
}

static unsigned int sla_ipv4_out(void *priv,
                                 struct sk_buff *skb,
                                 const struct nf_hook_state *state)
{
    struct sla_interface_stats *p;
    struct sla_stats_entry *entry;
    struct timespec64 tm;
    struct sla_interface* iface = NULL;
    unsigned long flags;

    if (!READ_ONCE(config_data.enable) || !state->out || (test_ifindex && state->out->ifindex != test_ifindex) ||
        (!sla_prepare_packet(skb) || !sla_test_owner(skb, state)))
        return NF_ACCEPT;
    STATS_LOCK_IRQ(&stats_spin_lock,flags);
    if (config_data.enable && config_data.voip_enable && config_data.voip_uid != 0)
    {
        if (sla_voip_pack_filter(skb))
            goto unlock;
    }

    if(config_data.enable && config_data.rate_on)
    {
        if(!sla_pkt_valid(skb, LT_UPLINK))
            goto unlock;

        //printk("#### SLA sla_ipv4_out ");
        //printk("#### %s devname=[%s] mac-len=%d skb->len=%d ifaceid = ((%d)) \n", __func__, state->out->name, skb->mac_len, skb->len, state->out->ifindex);
        iface =  sla_find_interface(state->out->name);

        if(iface == NULL)
        {
            goto unlock;
        }
        p = iface->mmap_struct;

        ktime_get_real_ts64(&tm);
        pkt_add(ntohs(tcp_hdr(skb)->source));

        entry = &p->entries[iface->write_index];
        entry->link_type = LT_UPLINK;

        entry->timestamp = tm.tv_sec*1000 + tm.tv_nsec/1000000;
        entry->size = skb->len + 14; //14 is mac len
        entry->port = ntohs(tcp_hdr(skb)->source);
        entry->discard = 1;

        iface->write_index = (iface->write_index + 1) % ENTRY_TOTAL;
        p->magic_num = STATS_MAGIC;
        p->total = ENTRY_TOTAL;
        smp_wmb();
        WRITE_ONCE(p->cur_idx, iface->write_index);

    }
unlock:
    STATS_UNLOCK_IRQ(&stats_spin_lock,flags);
    return NF_ACCEPT;
}

static struct nf_hook_ops sla_ipv4_ops[] __read_mostly =
{
    {
        .hook       = sla_ipv4_in,
        .pf         = NFPROTO_IPV4,
        .hooknum    = NF_INET_PRE_ROUTING,
        .priority   = NF_IP_PRI_CONNTRACK+2,
    },
    {
        .hook       = sla_ipv4_out,
        .pf         = NFPROTO_IPV4,
        .hooknum    = NF_INET_POST_ROUTING,
        .priority   = NF_IP_PRI_LAST,
    },
};

static ssize_t stats_read(struct file *file, char __user *buffer,
                          size_t count, loff_t *position)
{
    struct sla_interface *iface = file->private_data;

    return simple_read_from_buffer(buffer, count, position,
                                   iface->mmap_struct, STATS_BUF_SIZE);
}

static void stats_vma_open(struct vm_area_struct *vma)
{
    struct sla_interface *iface = vma->vm_private_data;

    __module_get(THIS_MODULE);
    kref_get(&iface->refs);
}

static void stats_vma_close(struct vm_area_struct *vma)
{
    struct sla_interface *iface = vma->vm_private_data;

    kref_put(&iface->refs, sla_interface_release);
    module_put(THIS_MODULE);
}

static const struct vm_operations_struct stats_vm_ops = {
    .open = stats_vma_open,
    .close = stats_vma_close,
};

static int stats_map(struct file *file, struct vm_area_struct *vma)
{
    struct sla_interface *iface = file->private_data;
    unsigned long length = vma->vm_end - vma->vm_start;
    int ret;

    if (!iface || vma->vm_pgoff || !length ||
        length > PAGE_ALIGN(STATS_BUF_SIZE))
        return -EINVAL;
    if (!try_module_get(THIS_MODULE))
        return -ENODEV;
    vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
    ret = remap_vmalloc_range(vma, iface->mmap_struct, 0);
    if (ret) {
        module_put(THIS_MODULE);
        return ret;
    }
    kref_get(&iface->refs);
    vma->vm_private_data = iface;
    vma->vm_ops = &stats_vm_ops;
    return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
    struct sla_interface *iface = pde_data(inode);

    kref_get(&iface->refs);
    file->private_data = iface;
    return 0;
}

static int stats_release(struct inode *inode, struct file *file)
{
    struct sla_interface *iface = file->private_data;

    kref_put(&iface->refs, sla_interface_release);
    return 0;
}

static const struct proc_ops stats_fops = {
    .proc_open = stats_open,
    .proc_read = stats_read,
    .proc_lseek = default_llseek,
    .proc_mmap = stats_map,
    .proc_release = stats_release,
};

/*sla voip state file operations start*/
static int sla_voip_file_show(struct seq_file *seq, void *unused)
{
    struct sla_voip_state value;
    unsigned long flags;

    spin_lock_irqsave(&stats_spin_lock, flags);
    value = voip_state;
    memset(&voip_state, 0, sizeof(voip_state));
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    seq_printf(seq, "%u,%d", value.address, value.port);
    return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(sla_voip_file);

/*sla voip state file operations end*/

static void sla_stats_data_init(struct sla_interface_stats *p)
{
    memset(p, 0, STATS_BUF_SIZE);

    p->magic_num = STATS_MAGIC;
    p->total = ENTRY_TOTAL;
    p->cur_idx = 0;
    p->reserve = 0;
}

/**
 *
 * @param interface_name
 *
 * @return struct sla_interface*
 *
 * This function itterates over sla_iface_list to find the
 * interface structure that matches interface_name.
 *
 * The caller needs to acquire stats_lock() before calling the
 * function and while using the returned sla_interface ptr.
 *
 * The caller also needs to release the lock by calling
 * stats_unlock() after using ( reading or writing to)
 * sla_interface ptr.
 */
static struct sla_interface * sla_find_interface(const char *interface_name)
{
    struct sla_interface * iface = NULL;

    list_for_each_entry (iface , &sla_iface_list, interface_list_hook)
    {
         if(strcmp(interface_name,iface->interface_name ) == 0)
         {
             return iface;
         }
    }

    return NULL;
}

static int sla_interface_start(const char *name)
{
    struct sla_interface *iface;
    unsigned long flags;
    char mmap_name[IF_NAME_LEN + sizeof(MMAP_FILE_SUFFIX)];
    char traffic_name[IF_NAME_LEN + sizeof(TOTAL_STATS_FILE_SUFFIX)];
    int ret = -ENOMEM;

    if (!name || !dev_valid_name(name))
        return -EINVAL;
    spin_lock_irqsave(&stats_spin_lock, flags);
    iface = sla_find_interface(name);
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    if (iface)
        return -EEXIST;
    iface = kzalloc(sizeof(*iface), GFP_KERNEL);
    if (!iface)
        return -ENOMEM;
    kref_init(&iface->refs);
    if (strscpy(iface->interface_name, name, sizeof(iface->interface_name)) < 0) {
        ret = -ENAMETOOLONG;
        goto put;
    }
    iface->mmap_struct = vmalloc_user(PAGE_ALIGN(STATS_BUF_SIZE));
    if (!iface->mmap_struct)
        goto put;
    sla_stats_data_init(iface->mmap_struct);
    snprintf(mmap_name, sizeof(mmap_name), "%s%s", name, MMAP_FILE_SUFFIX);
    snprintf(traffic_name, sizeof(traffic_name), "%s%s", name, TOTAL_STATS_FILE_SUFFIX);
    iface->mmap_file = proc_create_data(mmap_name, 0644, sla_dir, &stats_fops, iface);
    if (!iface->mmap_file)
        goto put;
    iface->total_stats_file = proc_create_data(traffic_name, 0644, sla_dir, &fops_u64, iface);
    if (!iface->total_stats_file)
        goto remove;
    spin_lock_irqsave(&stats_spin_lock, flags);
    list_add_tail(&iface->interface_list_hook, &sla_iface_list);
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    return 0;
remove:
    proc_remove(iface->mmap_file);
put:
    kref_put(&iface->refs, sla_interface_release);
    return ret;
}

static void sla_stats_disable(void)
{
    LIST_HEAD(detached);
    struct sla_interface *iface, *next;
    unsigned long flags;

    spin_lock_irqsave(&stats_spin_lock, flags);
    config_data.enable = 0;
    config_data.rate_on = 0;
    list_splice_init(&sla_iface_list, &detached);
    pkt_hlist_deinit();
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    list_for_each_entry_safe(iface, next, &detached, interface_list_hook) {
        proc_remove(iface->mmap_file);
        proc_remove(iface->total_stats_file);
        list_del(&iface->interface_list_hook);
        kref_put(&iface->refs, sla_interface_release);
    }
}

static void sla_stats_enable(void)
{
    config_data.enable = 1;
    config_data.rate_on = 1;
}

static int config_file_show(struct seq_file *s, void *data)
{
    struct sla_config snapshot;
    struct sla_config *config = &snapshot;
    unsigned long flags;

    spin_lock_irqsave(&stats_spin_lock, flags);
    snapshot = config_data;
    spin_unlock_irqrestore(&stats_spin_lock, flags);
    int i;

    seq_printf(s, CMD_ENABLE"%d\n", config->enable);
    seq_printf(s, CMD_RATE_ON"%d\n", config->rate_on);

    seq_puts(s, CMD_PORTS);
    if(0 != config->ports[0])
    {
        seq_printf(s, "%d", config->ports[0]);

        for(i=1; (i<PORTS_MAX) && (0!=config->ports[i]); i++)
            seq_printf(s, ",%d", config->ports[i]);
    }

    seq_printf(s, "\n");
    seq_printf(s, CMD_MAX_SIZE"%d\n", config->max_size);
    seq_printf(s, CMD_LIMIT_TIME"%d\n", config->limit_time);
    seq_printf(s, CMD_VOIP_ENABLE"%d\n", config->voip_enable);
    seq_printf(s, CMD_VOIP_UID"%u\n", config->voip_uid);

    return 0;
}

static int config_file_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, config_file_show, pde_data(inode));
}

static ssize_t config_file_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *position)
{
    char *command, *value, *cursor, *token;
    struct sla_interface *iface;
    unsigned long flags;
    unsigned int number, i;
    u16 ports[PORTS_MAX] = {};
    int ret = 0;

    if (!count)
        return 0;
    if (count >= CONFIG_CMD_MAX)
        return -E2BIG;
    command = memdup_user_nul(buffer, count);
    if (IS_ERR(command))
        return PTR_ERR(command);
    strim(command);
    mutex_lock(&config_mutex);
    if (exiting) {
        ret = -ENODEV;
        goto out;
    }
    if (!strncmp(command, CMD_IFACE_ADD, strlen(CMD_IFACE_ADD))) {
        ret = sla_interface_start(command + strlen(CMD_IFACE_ADD));
        goto out;
    }
    if (!strcmp(command, "enable=0")) {
        sla_stats_disable();
        goto out;
    }
    if (!strncmp(command, CMD_PORTS, strlen(CMD_PORTS))) {
        cursor = command + strlen(CMD_PORTS);
        for (i = 0; (token = strsep(&cursor, ",")); i++) {
            if (i >= PORTS_MAX || kstrtouint(token, 10, &number) ||
                !number || number > U16_MAX) {
                ret = -EINVAL;
                goto out;
            }
            ports[i] = number;
        }
        spin_lock_irqsave(&stats_spin_lock, flags);
        memcpy(config_data.ports, ports, sizeof(ports));
        spin_unlock_irqrestore(&stats_spin_lock, flags);
        goto out;
    }
    value = strchr(command, '=');
    if (value) {
        *value++ = 0;
        if (kstrtouint(value, 10, &number)) {
            ret = -EINVAL;
            goto out;
        }
    } else {
        number = 0;
    }
    spin_lock_irqsave(&stats_spin_lock, flags);
    if (!strcmp(command, "enable") && number == 1 && value) {
        sla_stats_enable();
    } else if (!strcmp(command, "rate_on") && number <= 1 && value) {
        config_data.rate_on = number;
    } else if (!strcmp(command, "reset_traffic") && number == 1 && value) {
        list_for_each_entry(iface, &sla_iface_list, interface_list_hook)
            iface->total_trfc = 0;
    } else if (!strcmp(command, "hash_clear") && !value) {
        pkt_hlist_deinit();
    } else if (!strcmp(command, "hash_print") && !value) {
        pkt_hlist_print();
    } else if (!strcmp(command, "max_size") && value &&
               number >= MAX_SIZE_DEFAULT && number <= U16_MAX) {
        config_data.max_size = number;
    } else if (!strcmp(command, "limit_time") && value &&
               number >= LIMIT_TIME_DEFAULT && number <= U16_MAX) {
        config_data.limit_time = number;
    } else if (!strcmp(command, "voip_enable") && value && number <= 1) {
        config_data.voip_enable = number;
        if (number)
            memset(&voip_state, 0, sizeof(voip_state));
    } else if (!strcmp(command, "voip_uid") && value &&
               uid_valid(make_kuid(&init_user_ns, number))) {
        config_data.voip_uid = number;
    } else {
        ret = -EINVAL;
    }
    spin_unlock_irqrestore(&stats_spin_lock, flags);
out:
    mutex_unlock(&config_mutex);
    kfree(command);
    return ret ? ret : count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
static const struct proc_ops config_fops =
{
    .proc_open = config_file_open,
    .proc_read = seq_read,
    .proc_write = config_file_write,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations config_fops =
{
    .owner = THIS_MODULE,
    .open = config_file_open,
    .read = seq_read,
    .write = config_file_write,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

static int __init sla_static_collector_init(void)
{
    int ret;

    static_assert(sizeof(struct sla_stats_entry) == 16);
    static_assert(STATS_BUF_SIZE == 20 * 1024);
    if (!proc_name || !*proc_name || strchr(proc_name, '/') || test_ifindex < 0)
        return -EINVAL;
    stats_lock_init();
    pkt_lock_init();
    list_lock_init();
    pkt_hlist_init();
    config_data.ports[0] = 80;
    config_data.ports[1] = 443;
    config_data.max_size = MAX_SIZE_DEFAULT;
    config_data.limit_time = LIMIT_TIME_DEFAULT;
    sla_dir = proc_mkdir(proc_name, NULL);
    if (!sla_dir)
        return -ENOMEM;
    config_file = proc_create_data("config", 0644, sla_dir, &config_fops, &config_data);
    sla_voip_file = proc_create_data(VOIP_STATS_FILE, 0644, sla_dir,
                                     &sla_voip_file_proc_ops, NULL);
    if (!config_file || !sla_voip_file) {
        ret = -ENOMEM;
        goto remove_proc;
    }
    ret = nf_register_net_hooks(&init_net, sla_ipv4_ops, ARRAY_SIZE(sla_ipv4_ops));
    if (!ret)
        return 0;
remove_proc:
    proc_remove(sla_dir);
    return ret;
}

static void __exit sla_static_collector_deinit(void)
{
    mutex_lock(&config_mutex);
    exiting = true;
    mutex_unlock(&config_mutex);
    nf_unregister_net_hooks(&init_net, sla_ipv4_ops, ARRAY_SIZE(sla_ipv4_ops));
    sla_stats_disable();
    proc_remove(sla_dir);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Xiaomi SLA packet timing and traffic collector");
module_init(sla_static_collector_init);
module_exit(sla_static_collector_deinit);
