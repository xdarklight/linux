// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq XWAY PCI host controller driver.
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

#include "../pci.h"

#define PCI_CR_CLK_CTRL					0x0000
#define PCI_CR_CLK_CTRL_FPI_CLK_DIS			BIT(0)
#define PCI_CR_CLK_CTRL_EBU_PCI_SWITCH_EN		BIT(1)
#define PCI_CR_CLK_CTRL_FIXED_ARB_EN			BIT(2)
#define PCI_CR_CLK_CTRL_FPI_NORMAL_ACK			BIT(3)

#define PCI_CS_BASE_ADDR1				0x0010
#define PCI_CR_PCI_EOI					0x002c

#define PCI_CR_PCI_MOD					0x0030
#define PCI_CR_PCI_MOD_CONFIG_OK			BIT(24)

#define PCI_CR_BAR11MASK				0x0044

#define PCI_CR_BAR12MASK				0x0048
#define PCI_CR_BAR12MASK_BAR_DECODING_EN		BIT(31)

#define PCI_CR_BAR13MASK				0x004c
#define PCI_CR_BAR13MASK_BAR_DECODING_EN		BIT(31)

#define PCI_CR_BAR14MASK				0x0050
#define PCI_CR_BAR14MASK_BAR_DECODING_EN		BIT(31)

#define PCI_CR_BAR15MASK				0x0054
#define PCI_CR_BAR15MASK_BAR_DECODING_EN		BIT(31)

#define PCI_CR_BAR16MASK				0x0058
#define PCI_CR_BAR16MASK_BAR_DECODING_EN		BIT(31)

#define PCI_CR_PCI_ADDR_MAP11				0x0064
#define PCI_CR_PC_ARB					0x0080
#define PCI_CR_FPI_ADDR_MAP0				0x00c0
#define PCI_CR_FPI_ADDR_MAP1				0x00c4
#define PCI_CR_FPI_ADDR_MAP2				0x00c8
#define PCI_CR_FPI_ADDR_MAP3				0x00cc
#define PCI_CR_FPI_ADDR_MAP4				0x00d0
#define PCI_CR_FPI_ADDR_MAP5				0x00d4
#define PCI_CR_FPI_ADDR_MAP6				0x00d8
#define PCI_CR_FPI_ADDR_MAP7				0x00dc
#define PCI_CR_FPI_ADDR_MAP11_LOW			0x00e0
#define PCI_CR_FPI_ADDR_MAP11_HIGH			0x00e4
#define PCI_CR_FPI_BURST_LENGTH				0x00e8

#define PCI_MASTER0_REQ_MASK_2BITS			8
#define PCI_MASTER1_REQ_MASK_2BITS			10
#define PCI_MASTER2_REQ_MASK_2BITS			12
#define INTERNAL_ARB_ENABLE_BIT				0

#define LTQ_PCI_CFG_BUSNUM_SHF				16
#define LTQ_PCI_CFG_DEVNUM_SHF				11
#define LTQ_PCI_CFG_FUNNUM_SHF				8

#define LTQ_PCI_SLOT_SOC				0xd

struct lantiq_pci {
	void __iomem		*ctrl_base;
	void __iomem		*config_base;
	struct gpio_desc	*reset_gpiod;
	struct reset_control	*core_reset;
	u32			req_mask;
	struct clk		*pci_clk;
	/*
	 * TODO: pci_clk_rate can be configured by using the assigned-clocks
	 * and assigned-clock-rates properties via devicetree once the lantiq
	 * target is ported to the common clock framework.
	 */
	u32			pci_clk_rate;
	/*
	 * TODO: external_clk is a hack because it's a mux instead of a gate.
	 * Drop it when porting the lantiq target over to use the common clock
	 * framework.
	 */
	struct clk		*external_clk;
};

static void __iomem *lantiq_pci_map_bus_internal(struct pci_bus *bus,
					      unsigned int devfn, int where)
{
	struct lantiq_pci *lantiq_pci = bus->sysdata;
	u32 offset;

	offset = ALIGN_DOWN(where, 4);
	offset |= (bus->number << LTQ_PCI_CFG_BUSNUM_SHF);
	offset |= (PCI_SLOT(devfn) << LTQ_PCI_CFG_DEVNUM_SHF);
	offset |= (PCI_FUNC(devfn) << LTQ_PCI_CFG_FUNNUM_SHF);

	return lantiq_pci->config_base + offset;
}

static void lantiq_pci_clear_master_abort(struct pci_bus *bus)
{
	void __iomem *target;
	u32 tmp;

	tmp = readl(lantiq_pci_map_bus_internal(bus, PCI_DEVFN(0x0, 0x0),
					     PCI_COMMAND));

	target = lantiq_pci_map_bus_internal(bus,
					  PCI_DEVFN(LTQ_PCI_SLOT_SOC, 0x0),
					  PCI_COMMAND);

	/* the SoC itself uses big endian byte-order */
	iowrite32be(tmp, target);
}

static void __iomem *lantiq_pci_map_bus(struct pci_bus *bus, unsigned int devfn,
				     int where)
{
	if (bus->number != 0)
		return NULL;

	if (PCI_SLOT(devfn) == 0)
		return NULL;

	if (PCI_SLOT(devfn) == LTQ_PCI_SLOT_SOC)
		return NULL;

	return lantiq_pci_map_bus_internal(bus, devfn, where);
}

static int lantiq_pci_read_config(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 *val)
{
	int ret;

	ret = pci_generic_config_read32(bus, devfn, where, size, val);
	if (ret)
		return ret;

	lantiq_pci_clear_master_abort(bus);

	return PCIBIOS_SUCCESSFUL;
}

static int lantiq_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	int ret;

	ret = pci_generic_config_write32(bus, devfn, where, size, val);
	if (ret)
		return ret;

	lantiq_pci_clear_master_abort(bus);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops lantiq_pci_ops = {
	.map_bus	= lantiq_pci_map_bus,
	.read		= lantiq_pci_read_config,
	.write		= lantiq_pci_write_config,
};

static inline u32 ltq_calc_bar11mask(void)
{
	u32 mem, bar11mask;

	/* BAR11MASK value depends on available memory on system. */
	mem = max_low_pfn << PAGE_SHIFT;
	bar11mask = ((-roundup_pow_of_two(mem)) & 0x0f000000) | 8;

	return bar11mask;
}

static void lantiq_pci_startup(struct lantiq_pci *lantiq_pci)
{
	u32 temp_buffer;

	/* enable auto-switching between PCI and EBU */
	temp_buffer = PCI_CR_CLK_CTRL_EBU_PCI_SWITCH_EN;
	temp_buffer |= PCI_CR_CLK_CTRL_FPI_NORMAL_ACK;
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_CLK_CTRL);

	/*
	 * inform the hardware that configuration is not done (yet).
	 * PCI access has to be retried.
	 */
	temp_buffer = ioread32be(lantiq_pci->ctrl_base + PCI_CR_PCI_MOD);
	temp_buffer &= ~PCI_CR_PCI_MOD_CONFIG_OK;
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_PCI_MOD);

	/* BUS Master/IO/MEM access */
	temp_buffer = ioread32be(lantiq_pci->config_base + PCI_COMMAND);
	temp_buffer |= PCI_COMMAND_IO;
	temp_buffer |= PCI_COMMAND_MEMORY;
	temp_buffer |= PCI_COMMAND_MASTER;
	iowrite32be(temp_buffer, lantiq_pci->config_base + PCI_COMMAND);

	/* enable external 2 PCI masters */
	temp_buffer = ioread32be(lantiq_pci->ctrl_base + PCI_CR_PC_ARB);
	/* setup the request mask */
	temp_buffer &= ~((lantiq_pci->req_mask & 0xf) << 16);
	/* enable internal arbiter */
	temp_buffer |= (1 << INTERNAL_ARB_ENABLE_BIT);
	/* enable internal PCI master reqest */
	temp_buffer &= (~(3 << PCI_MASTER0_REQ_MASK_2BITS));

	/* enable EBU request */
	temp_buffer &= (~(3 << PCI_MASTER1_REQ_MASK_2BITS));

	/* enable all external masters request */
	temp_buffer &= (~(3 << PCI_MASTER2_REQ_MASK_2BITS));
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_PC_ARB);

	/* setup BAR memory regions */
	iowrite32be(0x18000000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP0);
	iowrite32be(0x18400000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP1);
	iowrite32be(0x18800000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP2);
	iowrite32be(0x18c00000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP3);
	iowrite32be(0x19000000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP4);
	iowrite32be(0x19400000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP5);
	iowrite32be(0x19800000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP6);
	iowrite32be(0x19c00000, lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP7);

	iowrite32be(0x1ae00000,
		    lantiq_pci->ctrl_base + PCI_CR_FPI_ADDR_MAP11_HIGH);

	iowrite32be(ltq_calc_bar11mask(),
		    lantiq_pci->ctrl_base + PCI_CR_BAR11MASK);

	iowrite32be(0, lantiq_pci->ctrl_base + PCI_CR_PCI_ADDR_MAP11);
	iowrite32be(0, lantiq_pci->ctrl_base + PCI_CS_BASE_ADDR1);

	/* both TX and RX endian swap are enabled */
	iowrite32be(ioread32be(lantiq_pci->ctrl_base + PCI_CR_PCI_EOI) | 3,
		    lantiq_pci->ctrl_base + PCI_CR_PCI_EOI);

	temp_buffer = ioread32be(lantiq_pci->ctrl_base + PCI_CR_BAR12MASK);
	temp_buffer |= PCI_CR_BAR12MASK_BAR_DECODING_EN;
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_BAR12MASK);

	temp_buffer = ioread32be(lantiq_pci->ctrl_base + PCI_CR_BAR13MASK);
	temp_buffer |= PCI_CR_BAR13MASK_BAR_DECODING_EN;
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_BAR13MASK);

	/* use 8 dw burst length */
	iowrite32be(0x303, lantiq_pci->ctrl_base + PCI_CR_FPI_BURST_LENGTH);

	/* let the hardware know that we're done with the configuration */
	temp_buffer = ioread32be(lantiq_pci->ctrl_base + PCI_CR_PCI_MOD);
	temp_buffer |= PCI_CR_PCI_MOD_CONFIG_OK;
	iowrite32be(temp_buffer, lantiq_pci->ctrl_base + PCI_CR_PCI_MOD);

	/* take the PCI bus out of reset */
	gpiod_set_value_cansleep(lantiq_pci->reset_gpiod, 0);
}

static int lantiq_pci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *host;
	struct lantiq_pci *lantiq_pci;
	int ret;

	host = devm_pci_alloc_host_bridge(dev, sizeof(*lantiq_pci));
	if (!host)
		return -ENOMEM;

	lantiq_pci = pci_host_bridge_priv(host);

	host->ops = &lantiq_pci_ops;
	host->sysdata = lantiq_pci;

	lantiq_pci->ctrl_base = devm_platform_ioremap_resource_byname(pdev,
								      "ctrl");
	if (IS_ERR(lantiq_pci->ctrl_base))
		return PTR_ERR(lantiq_pci->ctrl_base);

	lantiq_pci->config_base = devm_platform_ioremap_resource_byname(pdev,
									"config");
	if (IS_ERR(lantiq_pci->config_base))
		return PTR_ERR(lantiq_pci->config_base);

	lantiq_pci->pci_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lantiq_pci->pci_clk)) {
		ret = PTR_ERR(lantiq_pci->pci_clk);
		dev_err_probe(dev, ret, "failed to get 'pci' clock\n");
		return ret;
	}

	lantiq_pci->external_clk = devm_clk_get(dev, "external");
	if (IS_ERR(lantiq_pci->external_clk)) {
		ret = PTR_ERR(lantiq_pci->external_clk);
		dev_err_probe(dev, ret,
			      "failed to get 'external' pci clock\n");
		return ret;
	}

	lantiq_pci->reset_gpiod = devm_gpiod_get_optional(dev, "reset",
							  GPIOD_OUT_HIGH);
	if (IS_ERR(lantiq_pci->reset_gpiod)) {
		ret = PTR_ERR(lantiq_pci->reset_gpiod);
		dev_err_probe(dev, ret, "failed to get the 'reset-gpios'\n");
		return ret;
	}

	lantiq_pci->core_reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(lantiq_pci->core_reset)) {
		ret = PTR_ERR(lantiq_pci->core_reset);
		dev_err_probe(dev, ret, "failed to get the reset line\n");
		return ret;
	}

	ret = device_property_read_u32(dev, "req-mask",
				       &lantiq_pci->req_mask);
	if (ret)
		lantiq_pci->req_mask = 0xf;

	ret = device_property_read_u32(dev, "lantiq,bus-clock",
				       &lantiq_pci->pci_clk_rate);
	if (ret)
		/* default to 33.3MHz */
		lantiq_pci->pci_clk_rate = 33333333;

	ret = clk_set_rate(lantiq_pci->pci_clk, lantiq_pci->pci_clk_rate);
	if (ret) {
		dev_err(dev, "Failed to set PCI clock to %uHz\n",
			lantiq_pci->pci_clk_rate);
		return ret;
	}

	ret = clk_prepare_enable(lantiq_pci->pci_clk);
	if (ret) {
		dev_err(dev, "failed to enable the 'pci' clock\n");
		return ret;
	}

	if (device_property_read_bool(dev, "lantiq,external-clock"))
		clk_enable(lantiq_pci->external_clk);
	else
		clk_disable(lantiq_pci->external_clk);

	lantiq_pci_startup(lantiq_pci);

	ret = pci_scan_root_bus_bridge(host);
	if (ret) {
		dev_err(dev, "failed to scan host: %d\n", ret);
		return ret;
	}

	pci_bus_assign_resources(host->bus);
	pci_bus_add_devices(host->bus);

	return 0;
}

static const struct of_device_id lantiq_pci_of_match[] = {
	{ .compatible = "lantiq,danube-pci" },
	{ .compatible = "lantiq,xrx100-pci" },
	{ .compatible = "lantiq,xrx200-pci" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lantiq_pci_of_match);

static struct platform_driver lantiq_pci_driver = {
	.probe = lantiq_pci_probe,
	.driver = {
		.name = "pci-lantiq",
		.of_match_table = lantiq_pci_of_match,
	},
};
module_platform_driver(lantiq_pci_driver);

MODULE_DESCRIPTION("Lantiq Danube/xRX100/xRX200 PCI controller driver");
MODULE_LICENSE("GPL v2");
