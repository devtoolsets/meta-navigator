#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>

#include "igb-uio.h"

// #define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
/*#include <linux/module.h>
#include <linux/circ_buf.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/crc32.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/inetdevice.h>
#include <linux/ip.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_data/macb.h>
#include <linux/pm_runtime.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/tcp.h>
#include <linux/udp.h>
*/

enum uio_intr_mode { GEM_INTR_MODE_NONE = 0, GEM_INTR_MODE_LEGACY, GEM_INTR_MODE_MSI, GEM_INTR_MODE_MSIX };

struct uio_gem_clock {
    union {
        struct {
            struct clk* pclk;
            struct clk* hclk;
            struct clk* tx;
            struct clk* rx;
            struct clk* tsu;
        };
        struct clk* clock_array[5];
    };
};

struct uio_gem_dev {
    struct uio_info info;
    struct platform_device* pdev;
    struct uio_gem_clock clock;
    enum uio_intr_mode mode;
    atomic_t refcnt;
};

typedef int (*uio_gem_clk_init)(struct platform_device*, struct uio_gem_dev*);
typedef int (*uio_gem_ether_init)(struct platform_device*);

static ssize_t uio_gem_show_max_vfs(struct device* dev, struct device_attribute* attr, char* buf)
{
    return snprintf(buf, 10, "%u\n", dev_num_vf(dev));
}

static ssize_t uio_gem_store_max_vfs(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

static DEVICE_ATTR(max_vfs, S_IRUGO | S_IWUSR, uio_gem_show_max_vfs, uio_gem_store_max_vfs);

static struct attribute* uio_gem_dev_attrs[] = {
    &dev_attr_max_vfs.attr,
    NULL,
};

static const struct attribute_group uio_gem_dev_attr_grp = {
    .attrs = uio_gem_dev_attrs,
};

static void uio_gem_release_iomem(struct uio_info* info)
{
    int i;

    for (i = 0; i < MAX_UIO_MAPS; i++) {
        if (info->mem[i].internal_addr)
            iounmap(info->mem[i].internal_addr);
    }
}

static int uio_gem_clock_init(struct platform_device* pdev, struct uio_gem_dev* udev)
{
    int err, i;

    struct clk* clk;
    struct uio_gem_clock* clocks = &udev->clock;
    const char* name[] = { "pclk", "hclk", "tx_clk", "rx_clk", "tsu_clk" };

    for (i = 0; i < (sizeof(name) / sizeof(name[0])); i++) {
        clk = devm_clk_get(&pdev->dev, name[i]);
        clocks->clock_array[i] = clk;
        if (IS_ERR(clk)) {
            err = PTR_ERR(clk);
            clocks->clock_array[i] = NULL;
            dev_err(&pdev->dev, "failed to get clock %s = %u\n", name[i], err);
        }
    }

    if (!udev->clock.pclk || !udev->clock.hclk) {
        return err;
    }

    for (i = 0; i < (sizeof(name) / sizeof(name[0])); i++) {
        err = clk_prepare_enable(clocks->clock_array[i]);
        if (err) {
            dev_err(&pdev->dev, "failed to enable %s clock (%u)\n", name[i], err);
            goto err_disable_clk;
        }
    }

    return 0;

err_disable_clk:
    for (i = 0; i < (sizeof(name) / sizeof(name[0])); i++) {
        clk_disable_unprepare(clocks->clock_array[i]);
        clocks->clock_array[i] = NULL;
    }

    return err;
}

static irqreturn_t uio_gem_irqhandler(int irq, void* dev_id)
{
    struct uio_gem_dev* udev = (struct uio_gem_dev*)dev_id;
    uio_event_notify(&udev->info);

    /* Message signal mode, no share IRQ and automasked */
    return IRQ_HANDLED;
}

static int uio_gem_open(struct uio_info* info, struct inode* inode)
{
    struct uio_gem_dev* udev = info->priv;
    struct platform_device* pdev = udev->pdev;
    int err;
    if (atomic_inc_return(&udev->refcnt) != 1)
        return 0;

    info->irq_flags = IRQF_SHARED;
    udev->info.irq = platform_get_irq(pdev, 0);
    if (udev->info.irq > 0)
        err = request_irq(udev->info.irq, uio_gem_irqhandler, udev->info.irq_flags, udev->info.name, udev);
    dev_info(&udev->pdev->dev, "uio device registered with irq %ld\n", udev->info.irq);

    return 0;
}

static int uio_gem_release(struct uio_info* info, struct inode* inode)
{
    struct uio_gem_dev* udev = info->priv;
    // struct platform_device *pdev = udev->pdev;

    if (atomic_dec_and_test(&udev->refcnt)) {
        /* disable interrupts */
        // igbuio_pci_disable_interrupts(udev);
    }

    return 0;
}

static int uio_gem_probe(struct platform_device* pdev)
{
    dma_addr_t map_dma_addr;
    void* map_addr;

    struct uio_gem_dev* udev;
    struct resource* regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    int err;

    udev = kzalloc(sizeof(struct uio_gem_dev), GFP_KERNEL);
    if (!udev)
        return -ENOMEM;

    /* remap IO memory */
    udev->info.mem[0].name = "xilinx-gem";
    udev->info.mem[0].addr = regs->start;
    udev->info.mem[0].internal_addr = NULL;
    udev->info.mem[0].size = regs->end - regs->start + 1;
    udev->info.mem[0].memtype = UIO_MEM_PHYS;

    uio_gem_clock_init(pdev, udev);

    /* fill uio infos */
    udev->info.name = "igb_uio";
    udev->info.version = "0.1";
    udev->info.open = uio_gem_open;
    udev->info.release = uio_gem_release;
    udev->info.priv = udev;
    udev->pdev = pdev;
    atomic_set(&udev->refcnt, 0);

    err = sysfs_create_group(&pdev->dev.kobj, &uio_gem_dev_attr_grp);
    if (err != 0)
        goto fail_release_iomem;

    /* register uio driver */
    err = uio_register_device(&pdev->dev, &udev->info);
    if (err != 0)
        goto fail_remove_group;

    platform_set_drvdata(pdev, udev);

    /* set 64-bit DMA mask */
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(44));
    if (err != 0) {
        dev_err(&pdev->dev, "Cannot set DMA mask\n");
        goto fail_release_iomem;
    }
    /*
     * Doing a harmless dma mapping for attaching the device to
     * the iommu identity mapping if kernel boots with iommu=pt.
     * Note this is not a problem if no IOMMU at all.
     */
    map_addr = dma_alloc_coherent(&pdev->dev, 1024, &map_dma_addr, GFP_KERNEL);
    if (map_addr)
        memset(map_addr, 0, 1024);

    if (!map_addr)
        dev_info(&pdev->dev, "dma mapping failed\n");
    else {
        dev_info(&pdev->dev, "mapping 1K dma=%#llx host=%p\n", (unsigned long long)map_dma_addr, map_addr);

        dma_free_coherent(&pdev->dev, 1024, map_addr, map_dma_addr);
        dev_info(&pdev->dev, "unmapping 1K dma=%#llx host=%p\n", (unsigned long long)map_dma_addr, map_addr);
    }

    return 0;

fail_remove_group:
    sysfs_remove_group(&pdev->dev.kobj, &uio_gem_dev_attr_grp);
fail_release_iomem:
    uio_gem_release_iomem(&udev->info);
    kfree(udev);

    return err;
}

static int uio_gem_remove(struct platform_device* pdev)
{
    struct uio_gem_dev* udev = platform_get_drvdata(pdev);

    sysfs_remove_group(&pdev->dev.kobj, &uio_gem_dev_attr_grp);
    uio_unregister_device(&udev->info);

    uio_gem_release_iomem(&udev->info);

    platform_set_drvdata(pdev, NULL);
    kfree(udev);
    return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id uio_gem_dt_ids[] = {
    { .compatible = "cdns,zynq-gem", .data = NULL /* &zynq_config */ },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, uio_gem_dt_ids);

#endif /* CONFIG_OF */

static struct platform_driver uio_gem_driver = {
	.probe		= uio_gem_probe,
	.remove		= uio_gem_remove,
	.driver		= {
		.name		= "igb_uio",//"uio-gem",
		.of_match_table	= of_match_ptr(uio_gem_dt_ids),
	},
};

module_platform_driver(uio_gem_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cadence MACB/GEM UIO Ethernet driver");
MODULE_AUTHOR("devtoolset@email.cn");
MODULE_ALIAS("platform:uio-gem");

#ifdef enable

struct gem_config {
    u32 caps;
    unsigned int dma_burst_length;
    gem_clk_init clk_init;
    gem_init init;
    int jumbo_max_len;
};

struct macb_dma_desc {
    u32 addr;
    u32 ctrl;
};

#ifdef MACB_EXT_DESC
#define HW_DMA_CAP_32B 0
#define HW_DMA_CAP_64B (1 << 0)
#define HW_DMA_CAP_PTP (1 << 1)
#define HW_DMA_CAP_64B_PTP (HW_DMA_CAP_64B | HW_DMA_CAP_PTP)

struct macb_dma_desc_64 {
    u32 addrh;
    u32 resvd;
};

struct macb_dma_desc_ptp {
    u32 ts_1;
    u32 ts_2;
};

struct macb;

struct gem_tx_ts {
    struct sk_buff* skb;
    struct macb_dma_desc_ptp desc_ptp;
};
#endif

struct macb_queue {
    struct macb* bp;
    int irq;

    unsigned int ISR;
    unsigned int IER;
    unsigned int IDR;
    unsigned int IMR;
    unsigned int TBQP;
    unsigned int TBQPH;
    unsigned int RBQP;

    unsigned int tx_head, tx_tail;
    struct macb_dma_desc* tx_ring;
    struct macb_tx_skb* tx_skb;
    dma_addr_t tx_ring_dma;
    struct work_struct tx_error_task;

#ifdef CONFIG_MACB_USE_HWSTAMP
    struct work_struct tx_ts_task;
    unsigned int tx_ts_head, tx_ts_tail;
    struct gem_tx_ts tx_timestamps[PTP_TS_BUFFER_SIZE];
#endif
};

struct macb {
    void __iomem* regs;
    bool native_io;

    /* hardware IO accessors */
    u32 (*macb_reg_readl)(struct macb* bp, int offset);
    void (*macb_reg_writel)(struct macb* bp, int offset, u32 value);

    unsigned int rx_tail;
    unsigned int rx_prepared_head;
    struct macb_dma_desc* rx_ring;
    struct macb_dma_desc* rx_ring_tieoff;
    struct sk_buff** rx_skbuff;
    void* rx_buffers;
    size_t rx_buffer_size;

    unsigned int rx_ring_size;
    unsigned int tx_ring_size;

    unsigned int num_queues;
    unsigned int queue_mask;
    struct macb_queue queues[MACB_MAX_QUEUES];

    spinlock_t lock;
    struct platform_device* pdev;
    struct clk* pclk;
    struct clk* hclk;
    struct clk* tx_clk;
    struct clk* rx_clk;
    struct clk* tsu_clk;
    struct net_device* dev;
    // struct napi_struct	napi;
    /*union {
        struct macb_stats	macb;
        struct gem_stats	gem;
    } hw_stats;*/

    dma_addr_t rx_ring_dma;
    dma_addr_t rx_ring_tieoff_dma;
    dma_addr_t rx_buffers_dma;

    // struct macb_or_gem_ops	macbgem_ops;

    struct mii_bus* mii_bus;
    struct phy_device* phy_dev;
    struct device_node* phy_node;
    int link;
    int speed;
    int duplex;

    u32 caps;
    unsigned int dma_burst_length;

    phy_interface_t phy_interface;
    struct gpio_desc* reset_gpio;

    /* AT91RM9200 transmit */
    struct sk_buff* skb;     /* holds skb until xmit interrupt completes */
    dma_addr_t skb_physaddr; /* phys addr from pci_map_single */
    int skb_length;          /* saved skb length for pci_unmap_single */
    unsigned int max_tx_length;

    // u64			ethtool_stats[GEM_STATS_LEN];

    unsigned int rx_frm_len_mask;
    unsigned int jumbo_max_len;

    unsigned int tsu_rate;

    struct tasklet_struct hresp_err_tasklet;

    /* holds value of rx watermark value for pbuf_rxcutthru register */
    u16 rx_watermark;

    // struct macb_ptp_info	*ptp_info;	/* macb-ptp interface */
#ifdef MACB_EXT_DESC
    uint8_t hw_dma_cap;
#endif
    spinlock_t tsu_clk_lock; /* gem tsu clock locking */
                             // struct ptp_clock *ptp_clock;
    // struct ptp_clock_info ptp_clock_info;
    // struct tsu_incr tsu_incr;
    // struct hwtstamp_config tstamp_config;
};

static bool hw_is_native_io(void __iomem* addr)
{
    u32 value = MACB_BIT(LLB);

    __raw_writel(value, addr + MACB_NCR);
    value = __raw_readl(addr + MACB_NCR);

    /* Write 0 back to disable everything */
    __raw_writel(0, addr + MACB_NCR);

    return value == MACB_BIT(LLB);
}

static int macb_init(struct platform_device* pdev)
{
    struct net_device* dev = platform_get_drvdata(pdev);
    unsigned int hw_q, q;
    struct macb* bp = netdev_priv(dev);
    struct macb_queue* queue;
    int err;
    u32 val;

    bp->tx_ring_size = DEFAULT_TX_RING_SIZE;
    bp->rx_ring_size = DEFAULT_RX_RING_SIZE;

    /* set the queue register mapping once for all: queue0 has a special
     * register mapping but we don't want to test the queue index then
     * compute the corresponding register offset at run time.
     */
    for (hw_q = 0, q = 0; hw_q < MACB_MAX_QUEUES; ++hw_q) {
        if (!(bp->queue_mask & (1 << hw_q)))
            continue;

        queue = &bp->queues[q];
        queue->bp = bp;
        if (hw_q) {
            queue->ISR = GEM_ISR(hw_q - 1);
            queue->IER = GEM_IER(hw_q - 1);
            queue->IDR = GEM_IDR(hw_q - 1);
            queue->IMR = GEM_IMR(hw_q - 1);
            queue->TBQP = GEM_TBQP(hw_q - 1);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
            if (bp->hw_dma_cap & HW_DMA_CAP_64B)
                queue->TBQPH = GEM_TBQPH(hw_q - 1);
#endif
            queue->RBQP = GEM_RBQP(hw_q - 1);
        } else {
            /* queue0 uses legacy registers */
            queue->ISR = MACB_ISR;
            queue->IER = MACB_IER;
            queue->IDR = MACB_IDR;
            queue->IMR = MACB_IMR;
            queue->TBQP = MACB_TBQP;
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
            if (bp->hw_dma_cap & HW_DMA_CAP_64B)
                queue->TBQPH = MACB_TBQPH;
#endif
            queue->RBQP = MACB_RBQP;
        }

        /* get irq: here we use the linux queue index, not the hardware
         * queue index. the queue irq definitions in the device tree
         * must remove the optional gaps that could exist in the
         * hardware queue mask.
         */
        queue->irq = platform_get_irq(pdev, q);
        err = devm_request_irq(&pdev->dev, queue->irq, macb_interrupt, IRQF_SHARED, dev->name, queue);
        if (err) {
            dev_err(&pdev->dev, "Unable to request IRQ %d (error %d)\n", queue->irq, err);
            return err;
        }

        INIT_WORK(&queue->tx_error_task, macb_tx_error_task);
        q++;
    }

    dev->netdev_ops = &macb_netdev_ops;
    netif_napi_add(dev, &bp->napi, macb_poll, 64);

    /* setup appropriated routines according to adapter type */
    if (macb_is_gem(bp)) {
        bp->max_tx_length = GEM_MAX_TX_LEN;
        bp->macbgem_ops.mog_alloc_rx_buffers = gem_alloc_rx_buffers;
        bp->macbgem_ops.mog_free_rx_buffers = gem_free_rx_buffers;
        bp->macbgem_ops.mog_init_rings = gem_init_rings;
        bp->macbgem_ops.mog_rx = gem_rx;
        dev->ethtool_ops = &gem_ethtool_ops;
    } else {
        bp->max_tx_length = MACB_MAX_TX_LEN;
        bp->macbgem_ops.mog_alloc_rx_buffers = macb_alloc_rx_buffers;
        bp->macbgem_ops.mog_free_rx_buffers = macb_free_rx_buffers;
        bp->macbgem_ops.mog_init_rings = macb_init_rings;
        bp->macbgem_ops.mog_rx = macb_rx;
        dev->ethtool_ops = &macb_ethtool_ops;
    }

    /* Set features */
    dev->hw_features = NETIF_F_SG;

    /* Check LSO capability */
    if (GEM_BFEXT(PBUF_LSO, gem_readl(bp, DCFG6)))
        dev->hw_features |= MACB_NETIF_LSO;

    /* Checksum offload is only available on gem with packet buffer */
    if (macb_is_gem(bp) && !(bp->caps & MACB_CAPS_FIFO_MODE))
        dev->hw_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
    if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD)
        dev->hw_features &= ~NETIF_F_RXCSUM;
    if (bp->caps & MACB_CAPS_SG_DISABLED)
        dev->hw_features &= ~NETIF_F_SG;
    dev->features = dev->hw_features;

    if (!(bp->caps & MACB_CAPS_USRIO_DISABLED)) {
        val = 0;
        if (bp->phy_interface == PHY_INTERFACE_MODE_RGMII)
            val = GEM_BIT(RGMII);
        else if (bp->phy_interface == PHY_INTERFACE_MODE_RMII && (bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
            val = MACB_BIT(RMII);
        else if (!(bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
            val = MACB_BIT(MII);

        if (bp->caps & MACB_CAPS_USRIO_HAS_CLKEN)
            val |= MACB_BIT(CLKEN);

        macb_or_gem_writel(bp, USRIO, val);
    }

    /* Set MII management clock divider */
    val = macb_mdc_clk_div(bp);
    val |= macb_dbw(bp);
    if (bp->phy_interface == PHY_INTERFACE_MODE_SGMII)
        val |= GEM_BIT(SGMIIEN) | GEM_BIT(PCSSEL);
    macb_writel(bp, NCFGR, val);

    if ((bp->phy_interface == PHY_INTERFACE_MODE_SGMII) && (bp->caps & MACB_CAPS_PCS))
        gem_writel(bp, PCSCNTRL, gem_readl(bp, PCSCNTRL) | GEM_BIT(PCSAUTONEG));

    return 0;
}

static const struct gem_config zynq_config = {
    .caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE | MACB_CAPS_NO_GIGABIT_HALF,
    .dma_burst_length = 16,
    .clk_init = macb_clk_init,
    .init = macb_init,
};
#endif
