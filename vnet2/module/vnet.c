// vnet.c - minimal-satisfying virtual NIC + /dev/vnet0 + mmap rings
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/log2.h>
#include <linux/io.h>
#include <linux/ip.h>

#define DRV_NAME "vnet"
#define DRV_VERSION "0.2-napi"

/* ---- shared ring layout (userspace sees the same structs) ---- */
#define VNET_RING_SZ  256
#define VNET_SLOT_SZ  2048

enum vnet_desc_state {
    VNET_DESC_FREE  = 0,
    VNET_DESC_READY = 1,
};

struct vnet_desc {
    __u32 state;
    __u32 len;
    __u32 off;   // offset from start of data region
    __u32 rsvd;
};

struct vnet_ring {
    __u32 prod;      // producer index
    __u32 cons;      // consumer index
    __u32 size;      // VNET_RING_SZ
    __u32 slot_sz;   // VNET_SLOT_SZ
    struct vnet_desc desc[VNET_RING_SZ];
    /* followed by data region: size * slot_sz bytes */
};

struct vnet_shared {
    struct vnet_ring rx; /* user->kernel */
    struct vnet_ring tx; /* kernel->user */
    /* followed by rx data region then tx data region (suggested) */
};

/* ioctl to “kick” RX (userspace says: I produced RX descs) */
#define VNET_IOC_MAGIC 'V'
#define VNET_IOC_KICK_RX _IO(VNET_IOC_MAGIC, 1)

/* ---- private driver state ---- */
struct vnet_priv {
    struct net_device *dev;
    struct napi_struct napi;

    /* char device */
    dev_t devno;
    struct cdev cdev;
    struct class *cls;
    struct device *cdev_dev;

    /* shared memory backing for mmap */
    void *shmem;
    size_t shmem_len;

    /* waitqueue so userspace can poll for TX availability */
    wait_queue_head_t tx_wq;

    /* basic locking for ring ops (simple + safe for first pass) */
    spinlock_t tx_lock;
    spinlock_t rx_lock;

    /* for “interrupt simulation” */
    bool napi_scheduled;

    /* FIXME */
    u8 rx_tmp[VNET_SLOT_SZ];
};

static const char *ip_proto_to_string(u8 protocol)
{
    switch (protocol) {
    case IPPROTO_TCP:
        return "TCP";
    case IPPROTO_UDP:
        return "UDP";
    case IPPROTO_ICMP:
        return "ICMP";
    case IPPROTO_IGMP:
        return "IGMP";
    case IPPROTO_IPV6:
        return "IPv6";
    case IPPROTO_ICMPV6:
        return "ICMPv6";
    case IPPROTO_SCTP:
        return "SCTP";
    case IPPROTO_GRE:
        return "GRE";
    case IPPROTO_ESP:
        return "ESP";
    case IPPROTO_AH:
        return "AH";
    case IPPROTO_IPIP:
        return "IPIP";
    case IPPROTO_DCCP:
        return "DCCP";
    case IPPROTO_RAW:
        return "RAW";
    default:
        return "Unknown";
    }
}

void pr_skb_iphdr(struct net_device *dev, struct sk_buff *skb, const char *msg)
{
    struct iphdr *iph;

    iph = ip_hdr(skb);

    if (iph) {
        netdev_info(dev, "ip (%s): src:%pI4 dst:%pI4 len:%u proto:%s ttl:%u ver:%u ihl:%u\n",
                    msg, &iph->saddr, &iph->daddr, ntohs(iph->tot_len),
                    ip_proto_to_string(iph->protocol), iph->ttl, iph->version, iph->ihl);
    } else {
        netdev_info(dev, "ip (%s): no hdr\n", msg);
    }
}

static size_t vnet_calc_shmem_len(void)
{
    size_t hdr = ALIGN(sizeof(struct vnet_shared), PAGE_SIZE);
    size_t rx_data = VNET_RING_SZ * VNET_SLOT_SZ;
    size_t tx_data = VNET_RING_SZ * VNET_SLOT_SZ;
    return hdr + rx_data + tx_data;
}

static inline u8 *vnet_rx_data_base(struct vnet_priv *priv)
{
    return (u8 *)priv->shmem + ALIGN(sizeof(struct vnet_shared), PAGE_SIZE);
}

static inline u8 *vnet_tx_data_base(struct vnet_priv *priv)
{
    size_t hdr = ALIGN(sizeof(struct vnet_shared), PAGE_SIZE);
    size_t rx_data = VNET_RING_SZ * VNET_SLOT_SZ;
    return (u8 *)priv->shmem + hdr + rx_data;
}

static void vnet_ring_init(struct vnet_ring *r)
{
    int i;
    r->prod = 0;
    r->cons = 0;
    r->size = VNET_RING_SZ;
    r->slot_sz = VNET_SLOT_SZ;
    for (i = 0; i < VNET_RING_SZ; i++) {
        r->desc[i].state = VNET_DESC_FREE;
        r->desc[i].len = 0;
        r->desc[i].off = i * VNET_SLOT_SZ; /* fixed slots */
    }
}

static int vnet_alloc_shmem(struct vnet_priv *priv)
{
    size_t len = vnet_calc_shmem_len();
    unsigned int order = get_order(len); /* rounds up to power-of-two pages */

    priv->shmem_len = (size_t)PAGE_SIZE << order;
    priv->shmem = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, order);
    if (!priv->shmem)
        return -ENOMEM;

    /* Initialize ring headers */
    {
        struct vnet_shared *sh = (struct vnet_shared *)priv->shmem;
        vnet_ring_init(&sh->rx);
        vnet_ring_init(&sh->tx);
    }

    pr_info(DRV_NAME ": shmem requested=%zu, allocated=%zu (order=%u)\n",
            len, priv->shmem_len, order);
    return 0;
}

static void vnet_free_shmem(struct vnet_priv *priv)
{
    if (!priv->shmem)
        return;

    /* free_pages needs the same order you allocated with */
    free_pages((unsigned long)priv->shmem, get_order(priv->shmem_len));
    priv->shmem = NULL;
    priv->shmem_len = 0;
}

/* ---- ring helpers (TODO) ---- */
static bool vnet_ring_has_ready(struct vnet_ring *r)
{
    return r->desc[r->cons].state == VNET_DESC_READY;
}

static int vnet_ring_push_bytes(struct vnet_ring *r, const void *src, u32 len,
                                u8 *data_base)
{
    u32 i = r->prod;
    struct vnet_desc *d = &r->desc[i];

    if (len > r->slot_sz)
        return -EMSGSIZE;

    if (d->state != VNET_DESC_FREE)
        return -ENOSPC;

    memcpy(data_base + d->off, src, len);
    /* publish */
    d->len = len;
    smp_wmb();
    d->state = VNET_DESC_READY;

    r->prod = (i + 1) % r->size;
    return 0;
}

static int vnet_ring_pop_bytes(struct vnet_ring *r, void *dst, u32 dst_cap, u32 *out_len,
                               u8 *data_base)
{
    u32 i = r->cons;
    struct vnet_desc *d = &r->desc[i];
    u32 len;

    printk("%s: enter\n", __func__);

    if (d->state != VNET_DESC_READY) {
        printk("%s:error: desc not ready\n", __func__);
        return -EAGAIN;
    }

    /* acquire */
    smp_rmb();
    len = d->len;

    if (len > dst_cap) {
        printk("%s:error: msgsize %u > dst_cap %u\n", __func__, len, dst_cap);
        return -EMSGSIZE;
    }

    memcpy(dst, data_base + d->off, len);

    d->len = 0;


    smp_wmb();

    d->state = VNET_DESC_FREE;

    r->cons = (i + 1) % r->size;
    *out_len = len;
    return 0;
}

/* ---- NAPI poll: consume RX ring and inject into stack ---- */
static int vnet_napi_poll(struct napi_struct *napi, int budget)
{
    struct vnet_priv *priv = container_of(napi, struct vnet_priv, napi);
    struct net_device *dev = priv->dev;

    int work = 0;

    netdev_info(dev, "vnet_napi_poll: budget=%d\n", budget);

    /* Locate shared rings + data regions */
    struct vnet_shared *sh = (struct vnet_shared *)priv->shmem;
    u8 *rx_data = vnet_rx_data_base(priv);

    while (work < budget) {
        u8 *tmp = priv->rx_tmp;
        u32 len = 0;
        int ret;

        spin_lock(&priv->rx_lock);
        ret = vnet_ring_pop_bytes(&sh->rx, tmp, VNET_SLOT_SZ, &len, rx_data);
        spin_unlock(&priv->rx_lock);
        if (ret) {
            netdev_info(dev, "vnet_napi_poll: no more RX frames (ret=%d)\n", ret);
            break;
        }
        pr_skb_iphdr(dev, (struct sk_buff *)tmp, "vnet_napi_poll RX");

        /* Build skb and inject */
        {
            struct sk_buff *skb = napi_alloc_skb(napi, len);
            if (!skb) {
                dev->stats.rx_dropped++;
                continue;
            }
            memcpy(skb_put(skb, len), tmp, len);
            skb->protocol = eth_type_trans(skb, dev);
            napi_gro_receive(napi, skb);

            dev->stats.rx_packets++;
            dev->stats.rx_bytes += len;
        }

        work++;
    }

    if (work < budget) {
        napi_complete_done(napi, work);
        priv->napi_scheduled = false;
        /* “interrupts re-enabled” conceptually */
    }

    return work;
}

/* ---- netdev ops ---- */
static int vnet_open(struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);
    napi_enable(&priv->napi);
    netif_start_queue(dev);
    return 0;
}

static int vnet_stop(struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);
    netif_stop_queue(dev);
    napi_disable(&priv->napi);
    return 0;
}

static netdev_tx_t vnet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);
    struct vnet_shared *sh = (struct vnet_shared *)priv->shmem;

    netdev_info(dev, "%s: enter\n", __func__);
    pr_skb_iphdr(dev, skb, __func__);

    u8 *tx_data = vnet_tx_data_base(priv);

    /* For now: drop if shmem not set up */
    if (!priv->shmem) {
        dev_kfree_skb(skb);
        dev->stats.tx_dropped++;
        return NETDEV_TX_OK;
    }

    /* Copy skb bytes into TX ring */
    spin_lock(&priv->tx_lock);
    if (vnet_ring_push_bytes(&sh->tx, skb->data, skb->len, tx_data) == 0) {
        dev->stats.tx_packets++;
        dev->stats.tx_bytes += skb->len;
        spin_unlock(&priv->tx_lock);

        /* wake userspace pollers that want TX */
        wake_up_interruptible(&priv->tx_wq);
    } else {
        spin_unlock(&priv->tx_lock);
        dev->stats.tx_dropped++;
    }

    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static const struct net_device_ops vnet_netdev_ops = {
    .ndo_open       = vnet_open,
    .ndo_stop       = vnet_stop,
    .ndo_start_xmit = vnet_start_xmit,
};

static void vnet_chrdev_destroy(struct vnet_priv *priv)
{
    if (priv->cdev_dev)
        device_destroy(priv->cls, priv->devno);
    if (priv->cls)
        class_destroy(priv->cls);

    cdev_del(&priv->cdev);
    unregister_chrdev_region(priv->devno, 1);

    priv->cdev_dev = NULL;
    priv->cls = NULL;
}


/* ---- char device ops ---- */
static int vnet_chr_open(struct inode *ino, struct file *f)
{
    struct vnet_priv *priv = container_of(ino->i_cdev, struct vnet_priv, cdev);
    f->private_data = priv;
    return 0;
}

static long vnet_chr_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct vnet_priv *priv = f->private_data;

    pr_info(DRV_NAME ": enter ioctl cmd=0x%x\n", cmd);

    switch (cmd) {
    case VNET_IOC_KICK_RX:
        /* schedule NAPI if not already scheduled */
        if (!priv->napi_scheduled) {
            pr_info(DRV_NAME ": ioctl KICK_RX: scheduling NAPI\n");
            priv->napi_scheduled = true;
            napi_schedule(&priv->napi);
        } else {
            pr_info(DRV_NAME ": ioctl KICK_RX: NAPI already scheduled\n");
        }
        return 0;
    default:
        return -ENOTTY;
    }
}

static __poll_t vnet_chr_poll(struct file *f, poll_table *pt)
{
    struct vnet_priv *priv = f->private_data;
    struct vnet_shared *sh = (struct vnet_shared *)priv->shmem;

    __poll_t mask = 0;
    poll_wait(f, &priv->tx_wq, pt);

    /* If there’s something in TX ring, userspace can read it from mmap */
    if (sh && vnet_ring_has_ready(&sh->tx))
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

static int vnet_chr_mmap(struct file *f, struct vm_area_struct *vma)
{
    struct vnet_priv *priv = f->private_data;
    unsigned long vsize = vma->vm_end - vma->vm_start;
    unsigned long paddr, pfn;
    unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

    if (!priv->shmem)
        return -ENODEV;

    /* We only support mapping from offset 0 for now */
    if (off != 0)
        return -EINVAL;

    if (vsize > priv->shmem_len)
        return -EINVAL;

    /* Make it non-cached to keep coherence simple at first */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    paddr = virt_to_phys(priv->shmem);
    pfn = paddr >> PAGE_SHIFT;

    if (remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot))
        return -EAGAIN;

    return 0;
}


static const struct file_operations vnet_fops = {
    .owner          = THIS_MODULE,
    .open           = vnet_chr_open,
    .unlocked_ioctl = vnet_chr_ioctl,
    .poll           = vnet_chr_poll,
    .mmap           = vnet_chr_mmap,
};

static int vnet_chrdev_create(struct vnet_priv *priv)
{
    int ret;

    ret = alloc_chrdev_region(&priv->devno, 0, 1, "vnet0");
    if (ret)
        return ret;

    cdev_init(&priv->cdev, &vnet_fops);
    priv->cdev.owner = THIS_MODULE;

    ret = cdev_add(&priv->cdev, priv->devno, 1);
    if (ret)
        goto err_unreg;

    priv->cls = class_create("vnet");
    if (IS_ERR(priv->cls)) {
        ret = PTR_ERR(priv->cls);
        priv->cls = NULL;
        goto err_cdev;
    }

    priv->cdev_dev = device_create(priv->cls, NULL, priv->devno, NULL, "vnet0");
    if (IS_ERR(priv->cdev_dev)) {
        ret = PTR_ERR(priv->cdev_dev);
        priv->cdev_dev = NULL;
        goto err_class;
    }

    return 0;

err_class:
    class_destroy(priv->cls);
    priv->cls = NULL;
err_cdev:
    cdev_del(&priv->cdev);
err_unreg:
    unregister_chrdev_region(priv->devno, 1);
    return ret;
}

/* ---- module init/exit ---- */
static struct net_device *g_dev;

static int __init vnet_init(void)
{
    int ret;
    struct net_device *dev;
    struct vnet_priv *priv;

    pr_info(DRV_NAME "-" DRV_VERSION ": initializing\n");

    dev = alloc_etherdev(sizeof(*priv));
    if (!dev)
        return -ENOMEM;

    strscpy(dev->name, DRV_NAME "%d", IFNAMSIZ);

    priv = netdev_priv(dev);
    priv->dev = dev;

    spin_lock_init(&priv->tx_lock);
    spin_lock_init(&priv->rx_lock);
    init_waitqueue_head(&priv->tx_wq);

    dev->netdev_ops = &vnet_netdev_ops;
    dev->flags |= IFF_NOARP;
    eth_hw_addr_random(dev);

    netif_napi_add(dev, &priv->napi, vnet_napi_poll);

    /* allocate shared memory region and initialize rings */
    ret = vnet_alloc_shmem(priv);
    if (ret) {
        free_netdev(dev);
        return ret;
    }

    /* create /dev/vnet0 (alloc_chrdev_region, cdev_add, class_create, device_create) */
    ret = vnet_chrdev_create(priv);
    if (ret) {
        vnet_free_shmem(priv);
        free_netdev(dev);
        return ret;
    }
  
    ret = register_netdev(dev);
    if (ret) {
        free_netdev(dev);
        return ret;
    }

    g_dev = dev;
    pr_info(DRV_NAME ": loaded (netdev vnet0)\n");
    return 0;
}

static void __exit vnet_exit(void)
{
    struct vnet_priv *priv;

    if (!g_dev)
        return;

    priv = netdev_priv(g_dev);

    /* TODO: destroy char device + free shmem */

    netif_napi_del(&priv->napi);
    unregister_netdev(g_dev);
    vnet_chrdev_destroy(priv);
    vnet_free_shmem(priv);
    free_netdev(g_dev);

    pr_info(DRV_NAME ": unloaded\n");
}

module_init(vnet_init);
module_exit(vnet_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("you");
MODULE_DESCRIPTION("Minimal virtual NIC + mmap rings");
