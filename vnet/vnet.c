// vnet.c - Virtual Network Device Driver (Kernel 6.6)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/icmp.h>

#define DRV_NAME "vnet"
#define DRV_VERSION "0.2-napi"

/*
 * NAPI poll weight - maximum number of packets to process in one poll 
 * Typical value is 64, but we'll use 16 for easier debugging.
 */
 #define VNET_NAPI_WEIGHT 16

struct vnet_priv {
    struct net_device *dev;
    struct napi_struct napi; /* NAPI structure for polling */

    struct sk_buff_head rx_queue; /* RX queue for incoming packets */
    spinlock_t lock;

    unsigned long napi_polls; /* Count of NAPI polls */
    unsigned long napi_completes; /* Count of NAPI completes */
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

static int vnet_open(struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);

    netdev_info(dev, "vnet_open: enabling NAPI\n");

    /* Enable NAPI - tells kernel we're ready to be polled */
    napi_enable(&priv->napi);

    netif_carrier_on(dev);
    netif_start_queue(dev);

    return 0;
}

static int vnet_stop(struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);

    netdev_info(dev, "vnet_stop: disabling NAPI\n");

    netif_stop_queue(dev);
    netif_carrier_off(dev);

    /* Disable NAPI - no more polling */
    napi_disable(&priv->napi);

    /* Drain any packets left in the RX queue */
    skb_queue_purge(&priv->rx_queue);

    return 0;
}

/*
 * NAPI poll function - called by kernel when we have work to do.
 *
 * @napi: The NAPI structure (contains pointer to our priv)
 * @budget: Maximum number of packets we should process in this call
 *
 * Returns: Number of packets actually processed
 *
 * The kernel calls this repeatedy until we return less than 'budget',
 * which signals "no more work available".
 */

 static int vnet_poll(struct napi_struct *napi, int budget)
{
    struct vnet_priv *priv = container_of(napi, struct vnet_priv, napi);
    struct net_device *dev = priv->dev;
    int work_done = 0;
    struct sk_buff *skb;

    priv->napi_polls++;

    netdev_info(dev, "vnet_poll: budget=%d, queue_len=%d\n", budget, skb_queue_len(&priv->rx_queue));

    while (work_done < budget) {
        skb = skb_dequeue(&priv->rx_queue);
        if (!skb)
            break;

        /* Update RX stats */
        dev->stats.rx_packets++;
        dev->stats.rx_bytes += skb->len;

        /* Deliver packet to network stack */
        netif_receive_skb(skb);
        work_done++;
    }

    netdev_info(dev, "vnet_poll: processed %d packets\n", work_done);

    /*
     * If we processed fewer packets than budget, we've drained the queue.
     * Tell NAPI we're done and it should stop polling us.
     */
    if (work_done < budget) {
        priv->napi_completes++;
        napi_complete_done(napi, work_done);
        netdev_info(dev, "vnet_poll: no more work -- NAPI complete\n");
    }

    return work_done;
}

static netdev_tx_t vnet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);
    struct sk_buff *rx_skb;
    
    netdev_dbg(dev, "start_xmit: skb->len=%u\n", skb->len);

    struct iphdr *iph;

    iph = ip_hdr(skb);

    if (iph) {
        netdev_info(dev, "IP Header Info Pre-Swap:\n");
        /* Use %pI4 for IP addresses; pass by reference */
        netdev_info(dev, "  Source:      %pI4\n", &iph->saddr);
        netdev_info(dev, "  Destination: %pI4\n", &iph->daddr);
        
        /* Use ntohs() for fields in network byte order */
        netdev_info(dev, "  Total Len:   %u bytes\n", ntohs(iph->tot_len));
        netdev_info(dev, "  Protocol:    %s\n", ip_proto_to_string(iph->protocol));
        netdev_info(dev, "  TTL:         %u\n", iph->ttl);
        netdev_info(dev, "  Version:     %u, IHL: %u\n", iph->version, iph->ihl);

        /* Swap source and destination IP addresses */
        __be32 temp = iph->saddr;
        iph->saddr = iph->daddr;
        iph->daddr = temp;

        iph->check = 0;
        iph->check = ip_fast_csum((unsigned char *)iph, iph->ihl);

        netdev_info(dev, "IP Header Info Post-Swap:\n");
        /* Use %pI4 for IP addresses; pass by reference */
        netdev_info(dev, "  Source:      %pI4\n", &iph->saddr);
        netdev_info(dev, "  Destination: %pI4\n", &iph->daddr);

        if (iph && iph->protocol == IPPROTO_ICMP) {
            struct icmphdr *icmph = icmp_hdr(skb);
            if (icmph) {
                netdev_info(dev, "ICMP Packet: Iface=%s, Type=%u, Code=%u, Csum=0x%04x\n", dev->name, icmph->type, icmph->code, ntohs(icmph->checksum));
            }
        }    
    
    }

    /* Update TX stats */
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;
    
    /*
     * Clone the packet - we need a copy for RX since the original
     * skb will be freed by the caller after we return.
     */
    rx_skb = skb_clone(skb, GFP_ATOMIC);
    if (!rx_skb) {
        netdev_err(dev, "Failed to clone skb\n");
        dev->stats.rx_dropped++;
        dev_kfree_skb(skb);
        return NETDEV_TX_OK;
    }
    
    /* Prepare the cloned packet for reception */
    rx_skb->protocol = eth_type_trans(rx_skb, dev);
    rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
    
    /* Add packet to our RX queue */
    skb_queue_tail(&priv->rx_queue, rx_skb);
    
    /*
     * Schedule NAPI to process the queued packet.
     * This is like saying "I have work, please poll me soon".
     * 
     * napi_schedule() only does something if NAPI isn't already scheduled.
     * If we're already being polled, this is a no-op.
     */
    napi_schedule(&priv->napi);
    
    netdev_dbg(dev, "start_xmit: queued packet, scheduled NAPI\n");
    
    /* Free the original packet */
    dev_kfree_skb(skb);
    
    return NETDEV_TX_OK;
}
static const struct net_device_ops vnet_netdev_ops = {
    .ndo_open = vnet_open,
    .ndo_stop = vnet_stop,
    .ndo_start_xmit = vnet_start_xmit,
    /* ndo_get_stats removed - using dev->stats directly */
};

static u32 vnet_get_link(struct net_device *dev)
{
    return 1;  /* Always report link as up */
}

static void vnet_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
    struct vnet_priv *priv = netdev_priv(dev);
    
    strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
    strlcpy(info->version, DRV_VERSION, sizeof(info->version));
    
    /* Add NAPI stats to driver info */
    snprintf(info->fw_version, sizeof(info->fw_version),
             "polls:%lu completes:%lu", priv->napi_polls, priv->napi_completes);
}

static const struct ethtool_ops vnet_ethtool_ops = {
    .get_link = vnet_get_link,
    .get_drvinfo = vnet_get_drvinfo,
};

static void vnet_setup(struct net_device *dev)
{
    struct vnet_priv *priv = netdev_priv(dev);
    
    ether_setup(dev);
    
    dev->netdev_ops = &vnet_netdev_ops;
    dev->ethtool_ops = &vnet_ethtool_ops;
    dev->flags |= IFF_NOARP;
    dev->features |= NETIF_F_HW_CSUM;
    
    /* Generate random MAC address */
    eth_hw_addr_random(dev);
    
    /* Initialize RX queue for packets waiting to be processed */
    skb_queue_head_init(&priv->rx_queue);
    
    spin_lock_init(&priv->lock);
    priv->dev = dev;
    
    /* 
     * Initialize NAPI with our poll function and weight.
     * Weight = max packets to process per poll.
     */
    netif_napi_add(dev, &priv->napi, vnet_poll);
    
    netdev_info(dev, "vnet_setup: NAPI initialized\n");
}

static struct net_device *vnet_dev;

static int __init vnet_init(void)
{
    int ret;
    
    vnet_dev = alloc_netdev(sizeof(struct vnet_priv), DRV_NAME "%d",
                            NET_NAME_UNKNOWN, vnet_setup);
    if (!vnet_dev)
        return -ENOMEM;
    
    ret = register_netdev(vnet_dev);
    if (ret) {
        pr_err("vnet: failed to register netdev\n");
        free_netdev(vnet_dev);
        return ret;
    }
    
    pr_info("vnet: Virtual network device loaded (version %s)\n", DRV_VERSION);
    return 0;
}

static void __exit vnet_exit(void)
{
    unregister_netdev(vnet_dev);
    free_netdev(vnet_dev);
    pr_info("vnet: Virtual network device unloaded\n");
}

module_init(vnet_init);
module_exit(vnet_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sameer");
MODULE_DESCRIPTION("Virtual Network Device Driver");
MODULE_VERSION(DRV_VERSION);