/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2010 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/pm.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/reset-controller.h>

#include <asm/reboot.h>

#include <lantiq_soc.h>

#include "../prom.h"

/* reset request register */
#define RCU_RST_REQ		0x0010
/* reset status register */
#define RCU_RST_STAT		0x0014
/* vr9 gphy registers */
#define RCU_GFS_ADD0_XRX200	0x0020
#define RCU_GFS_ADD1_XRX200	0x0068
/* xRX300 gphy registers */
#define RCU_GFS_ADD0_XRX300	0x0020
#define RCU_GFS_ADD1_XRX300	0x0058
#define RCU_GFS_ADD2_XRX300	0x00AC
/* xRX330 gphy registers */
#define RCU_GFS_ADD0_XRX330	0x0020
#define RCU_GFS_ADD1_XRX330	0x0058
#define RCU_GFS_ADD2_XRX330	0x00AC
#define RCU_GFS_ADD3_XRX330	0x0264

/* xbar BE flag */
#define RCU_AHB_ENDIAN          0x004C
#define RCU_VR9_BE_AHB1S        0x00000008

/* reboot bit */
#define RCU_RD_GPHY0_XRX200	BIT(31)
#define RCU_RD_SRST		BIT(30)
#define RCU_RD_GPHY1_XRX200	BIT(29)
/* xRX300 bits */
#define RCU_RD_GPHY0_XRX300	BIT(31)
#define RCU_RD_GPHY1_XRX300	BIT(29)
#define RCU_RD_GPHY2_XRX300	BIT(28)
/* xRX330 bits */
#define RCU_RD_GPHY0_XRX330	BIT(31)
#define RCU_RD_GPHY1_XRX330	BIT(29)
#define RCU_RD_GPHY2_XRX330	BIT(28)
#define RCU_RD_GPHY3_XRX330	BIT(10)

/* reset cause */
#define RCU_STAT_SHIFT		26
/* boot selection */
#define RCU_BOOT_SEL(x)		((x >> 18) & 0x7)
#define RCU_BOOT_SEL_XRX200(x)	(((x >> 17) & 0xf) | ((x >> 8) & 0x10))

/* dwc2 USB configuration registers */
#define RCU_USB1CFG		0x0018
#define RCU_USB2CFG		0x0034

/* USB DMA endianness bits */
#define RCU_USBCFG_HDSEL_BIT	BIT(11)
#define RCU_USBCFG_HOST_END_BIT	BIT(10)
#define RCU_USBCFG_SLV_END_BIT	BIT(9)

/* USB reset bits */
#define RCU_USBRESET		0x0010

#define USBRESET_BIT		BIT(4)

#define RCU_USBRESET2		0x0048

#define USB1RESET_BIT		BIT(4)
#define USB2RESET_BIT		BIT(5)

#define RCU_CFG1A		0x0038
#define RCU_CFG1B		0x003C

/* USB PMU devices */
#define PMU_AHBM		BIT(15)
#define PMU_USB0		BIT(6)
#define PMU_USB1		BIT(27)

/* USB PHY PMU devices */
#define PMU_USB0_P		BIT(0)
#define PMU_USB1_P		BIT(26)

#define XLATE_OFFSET_SHIFT	8
#define XLATE_OFFSET_MASK	0xffffff
#define XLATE_BIT_MASK		0xff

/* remapped base addr of the reset control unit */
static void __iomem *ltq_rcu_membase;
static struct device_node *ltq_rcu_np;
static DEFINE_SPINLOCK(ltq_rcu_lock);

static void ltq_rcu_w32(uint32_t val, uint32_t reg_off)
{
	ltq_w32(val, ltq_rcu_membase + reg_off);
}

static uint32_t ltq_rcu_r32(uint32_t reg_off)
{
	return ltq_r32(ltq_rcu_membase + reg_off);
}

static void ltq_rcu_w32_mask(uint32_t clr, uint32_t set, uint32_t reg_off)
{
	unsigned long flags;

	spin_lock_irqsave(&ltq_rcu_lock, flags);
	ltq_rcu_w32((ltq_rcu_r32(reg_off) & ~(clr)) | (set), reg_off);
	spin_unlock_irqrestore(&ltq_rcu_lock, flags);
}

/* This function is used by the watchdog driver */
int ltq_reset_cause(void)
{
	u32 val = ltq_rcu_r32(RCU_RST_STAT);
	return val >> RCU_STAT_SHIFT;
}
EXPORT_SYMBOL_GPL(ltq_reset_cause);

/* allow platform code to find out what source we booted from */
unsigned char ltq_boot_select(void)
{
	u32 val = ltq_rcu_r32(RCU_RST_STAT);

	if (of_device_is_compatible(ltq_rcu_np, "lantiq,rcu-xrx200"))
		return RCU_BOOT_SEL_XRX200(val);

	return RCU_BOOT_SEL(val);
}

struct ltq_gphy_reset {
	u32 rd;
	u32 addr;
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx200_gphy[] = {
	{RCU_RD_GPHY0_XRX200, RCU_GFS_ADD0_XRX200},
	{RCU_RD_GPHY1_XRX200, RCU_GFS_ADD1_XRX200},
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx300_gphy[] = {
	{RCU_RD_GPHY0_XRX300, RCU_GFS_ADD0_XRX300},
	{RCU_RD_GPHY1_XRX300, RCU_GFS_ADD1_XRX300},
	{RCU_RD_GPHY2_XRX300, RCU_GFS_ADD2_XRX300},
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx330_gphy[] = {
	{RCU_RD_GPHY0_XRX330, RCU_GFS_ADD0_XRX330},
	{RCU_RD_GPHY1_XRX330, RCU_GFS_ADD1_XRX330},
	{RCU_RD_GPHY2_XRX330, RCU_GFS_ADD2_XRX330},
	{RCU_RD_GPHY3_XRX330, RCU_GFS_ADD3_XRX330},
};

static void xrx200_gphy_boot_addr(struct ltq_gphy_reset *phy_regs,
				  dma_addr_t dev_addr)
{
	ltq_rcu_w32_mask(0, phy_regs->rd, RCU_RST_REQ);
	ltq_rcu_w32(dev_addr, phy_regs->addr);
	ltq_rcu_w32_mask(phy_regs->rd, 0,  RCU_RST_REQ);
}

/* reset and boot a gphy. these phys only exist on xrx200 SoC */
int xrx200_gphy_boot(struct device *dev, unsigned int id, dma_addr_t dev_addr)
{
	struct clk *clk;

	if (!of_device_is_compatible(ltq_rcu_np, "lantiq,rcu-xrx200")) {
		dev_err(dev, "this SoC has no GPHY\n");
		return -EINVAL;
	}

	if (of_machine_is_compatible("lantiq,vr9")) {
		clk = clk_get_sys("1f203000.rcu", "gphy");
		if (IS_ERR(clk))
			return PTR_ERR(clk);
		clk_enable(clk);
	}

	dev_info(dev, "booting GPHY%u firmware at %X\n", id, dev_addr);

	if (of_machine_is_compatible("lantiq,vr9")) {
		if (id >= ARRAY_SIZE(xrx200_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx200_gphy[id], dev_addr);
	} else if (of_machine_is_compatible("lantiq,ar10")) {
		if (id >= ARRAY_SIZE(xrx300_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx300_gphy[id], dev_addr);
	} else if (of_machine_is_compatible("lantiq,grx390")) {
		if (id >= ARRAY_SIZE(xrx330_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx330_gphy[id], dev_addr);
	}
	return 0;
}

static inline u32 reset_id_to_reg_offset(unsigned long id)
{
	return (id >> XLATE_OFFSET_SHIFT) & XLATE_OFFSET_MASK;
}

static inline u32 reset_id_to_reg_bit(unsigned long id)
{
	return BIT(id & XLATE_BIT_MASK);
}

static inline int reset_reg_offset_and_bit_to_id(uint32_t offset, uint32_t bit)
{
	int val;

	val = ((offset & XLATE_OFFSET_MASK) << XLATE_OFFSET_SHIFT);
	val |= (bit & XLATE_BIT_MASK);

	return val;
}

static int ltq_assert_device(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	u32 offset, assert;

	offset = reset_id_to_reg_offset(id);
	assert = reset_id_to_reg_bit(id);

	ltq_rcu_w32_mask(0, assert, offset);

	return 0;
}

static int ltq_deassert_device(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	u32 offset, deassert;

	offset = reset_id_to_reg_offset(id);
	deassert = reset_id_to_reg_bit(id);

	ltq_rcu_w32_mask(deassert, 0, offset);

	return 0;
}

static int ltq_reset_device(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	int ret;

	ret = ltq_assert_device(rcdev, id);
	if (ret)
		return ret;

	return ltq_deassert_device(rcdev, id);
}

static int ltq_reset_xlate(struct reset_controller_dev *rcdev,
				const struct of_phandle_args *reset_spec)
{
	unsigned offset, bit;

	if (WARN_ON(reset_spec->args_count != rcdev->of_reset_n_cells))
		return -EINVAL;

	offset = reset_spec->args[0];
	bit = reset_spec->args[1];

	if (bit >= rcdev->nr_resets)
		return -EINVAL;

	return reset_reg_offset_and_bit_to_id(offset, bit);
}

static struct reset_control_ops xway_reset_ops = {
	.reset = ltq_reset_device,
	.assert = ltq_assert_device,
	.deassert = ltq_deassert_device,
};

static struct reset_controller_dev xway_reset_controller = {
	.ops			= &xway_reset_ops,
	.owner			= THIS_MODULE,
	.nr_resets		= 32,
	.of_reset_n_cells	= 2,
	.of_xlate		= ltq_reset_xlate,
};

static void ltq_machine_restart(char *command)
{
	u32 val = ltq_rcu_r32(RCU_RST_REQ);

	if (of_device_is_compatible(ltq_rcu_np, "lantiq,rcu-xrx200"))
		val |= RCU_RD_GPHY1_XRX200 | RCU_RD_GPHY0_XRX200;

	val |= RCU_RD_SRST;

	local_irq_disable();
	ltq_rcu_w32(val, RCU_RST_REQ);
	unreachable();
}

static void ltq_machine_halt(void)
{
	local_irq_disable();
	unreachable();
}

static void ltq_machine_power_off(void)
{
	local_irq_disable();
	unreachable();
}

static void ltq_usb_init(void)
{
	/* Power for USB cores 1 & 2 */
	ltq_pmu_enable(PMU_AHBM);
	ltq_pmu_enable(PMU_USB0);
	ltq_pmu_enable(PMU_USB1);

	ltq_rcu_w32(ltq_rcu_r32(RCU_CFG1A) | BIT(0), RCU_CFG1A);
	ltq_rcu_w32(ltq_rcu_r32(RCU_CFG1B) | BIT(0), RCU_CFG1B);

	/* Enable USB PHY power for cores 1 & 2 */
	ltq_pmu_enable(PMU_USB0_P);
	ltq_pmu_enable(PMU_USB1_P);

	/* Configure cores to host mode */
	ltq_rcu_w32(ltq_rcu_r32(RCU_USB1CFG) & ~RCU_USBCFG_HDSEL_BIT,
		RCU_USB1CFG);
	ltq_rcu_w32(ltq_rcu_r32(RCU_USB2CFG) & ~RCU_USBCFG_HDSEL_BIT,
		RCU_USB2CFG);

	/* Select DMA endianness (Host-endian: big-endian) */
	ltq_rcu_w32((ltq_rcu_r32(RCU_USB1CFG) & ~RCU_USBCFG_SLV_END_BIT)
		| RCU_USBCFG_HOST_END_BIT, RCU_USB1CFG);
	ltq_rcu_w32(ltq_rcu_r32((RCU_USB2CFG) & ~RCU_USBCFG_SLV_END_BIT)
		| RCU_USBCFG_HOST_END_BIT, RCU_USB2CFG);

	/* Hard reset USB state machines */
	ltq_rcu_w32(ltq_rcu_r32(RCU_USBRESET) | USBRESET_BIT, RCU_USBRESET);
	udelay(50 * 1000);
	ltq_rcu_w32(ltq_rcu_r32(RCU_USBRESET) & ~USBRESET_BIT, RCU_USBRESET);

	/* Soft reset USB state machines */
	ltq_rcu_w32(ltq_rcu_r32(RCU_USBRESET2)
		| USB1RESET_BIT | USB2RESET_BIT, RCU_USBRESET2);
	udelay(50 * 1000);
	ltq_rcu_w32(ltq_rcu_r32(RCU_USBRESET2)
		& ~(USB1RESET_BIT | USB2RESET_BIT), RCU_USBRESET2);
}

static int xway_reset_probe(struct platform_device *pdev)
{
	struct resource res;

	ltq_rcu_np = pdev->dev.of_node;

	xway_reset_controller.of_node = ltq_rcu_np;

	/* check if all the reset register range is available */
	if (!xway_reset_controller.of_node)
		panic("Failed to load reset resources from devicetree");

	if (of_address_to_resource(xway_reset_controller.of_node, 0, &res))
		panic("Failed to get rcu memory range");

	if (!request_mem_region(res.start, resource_size(&res), res.name))
		pr_err("Failed to request rcu memory");

	ltq_rcu_membase = ioremap_nocache(res.start, resource_size(&res));
	if (!ltq_rcu_membase)
		panic("Failed to remap core memory");

	return reset_controller_register(&xway_reset_controller);
}

static const struct of_device_id xway_reset_match[] = {
	{ .compatible = "lantiq,rcu-xway" },
	{ .compatible = "lantiq,rcu-xrx200" },
	{},
};

static struct platform_driver xway_reset_driver = {
	.probe = xway_reset_probe,
	.driver = {
		.name = "xway-reset",
		.of_match_table = xway_reset_match,
	},
};

static int __init mips_reboot_setup(void)
{
	int ret = platform_driver_register(&xway_reset_driver);

	if (ret) {
		pr_err("Error registering reset platform driver: %d\n", ret);
		return ret;
	}

	if (of_machine_is_compatible("lantiq,ar9") ||
	    of_machine_is_compatible("lantiq,vr9"))
		ltq_usb_init();

	if (of_machine_is_compatible("lantiq,vr9"))
		ltq_rcu_w32_mask(0, RCU_VR9_BE_AHB1S, RCU_AHB_ENDIAN);

	_machine_restart = ltq_machine_restart;
	_machine_halt = ltq_machine_halt;
	pm_power_off = ltq_machine_power_off;

	return 0;
}

arch_initcall(mips_reboot_setup);
