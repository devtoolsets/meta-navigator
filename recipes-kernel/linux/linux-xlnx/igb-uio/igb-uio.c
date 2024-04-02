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
