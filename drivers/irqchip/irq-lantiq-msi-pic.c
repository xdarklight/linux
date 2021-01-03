// SPDX-License-Identifier: GPL-2.0
/*
 * Lantiq MSI Programmable Interrupt Controller.
 *
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the board support package (called "UGW") for the xRX500 SoC:
 *   Copyright (C) 2018 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/irqchip.h>
#include <linux/msi.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define LANTIQ_MSI_PIC_TABLE(n)			(0x4 * (n))
#define LANTIQ_MSI_PIC_TABLE_INT_DISABLE	BIT(31)
#define LANTIQ_MSI_PIC_TABLE_INT_LINE		GENMASK(30, 28)
#define LANTIQ_MSI_PIC_TABLE_MSG_ADDR		GENMASK(27, 16)
#define LANTIQ_MSI_PIC_TABLE_MSG_DATA		GENMASK(15, 0)

#define LANTIQ_MSI_PIC_ENDIAN			0x40
#define LANTIQ_MSI_PIC_ENDIAN_LITTLE		0
#define LANTIQ_MSI_PIC_ENDIAN_BIG		BIT(0)

#define LANTIQ_MSI_NUM_MIN			4
#define LANTIQ_MSI_NUM_MAX			16
#define LANTIQ_MSI_BASE_DATA			0x4AE0

struct ltq_msi_pic_data {
	struct clk		*clk;
	phys_addr_t		msi_addr;
	void __iomem		*pic_base;
	struct mutex		msi_map_lock;
	unsigned long		*msi_map;
	int			nr_irqs;
	u32			irqs[LANTIQ_MSI_NUM_MAX];
};

static void lantiq_msi_pic_set_msi_irq_enabled(struct ltq_msi_pic_data *priv,
					       unsigned int hwirq, bool enable)
{
	u32 tmp;

	tmp = ioread32be(priv->pic_base + LANTIQ_MSI_PIC_TABLE(hwirq));

	if (enable)
		tmp &= ~LANTIQ_MSI_PIC_TABLE_INT_DISABLE;
	else
		tmp |= LANTIQ_MSI_PIC_TABLE_INT_DISABLE;

	iowrite32be(tmp, priv->pic_base + LANTIQ_MSI_PIC_TABLE(hwirq));
}

static void lantiq_msi_pic_mask_msi_irq(struct irq_data *data)
{
	struct ltq_msi_pic_data *priv = irq_data_get_irq_chip_data(data);

	lantiq_msi_pic_set_msi_irq_enabled(priv, data->hwirq, false);

	pci_msi_mask_irq(data);
	irq_chip_mask_parent(data);
}

static void lantiq_msi_pic_unmask_msi_irq(struct irq_data *data)
{
	struct ltq_msi_pic_data *priv = irq_data_get_irq_chip_data(data);

	pci_msi_unmask_irq(data);
	irq_chip_unmask_parent(data);

	lantiq_msi_pic_set_msi_irq_enabled(priv, data->hwirq, true);
}

static void lantiq_msi_pic_write_msi_msg(struct irq_data *data,
					 struct msi_msg *msg)
{
	struct ltq_msi_pic_data *priv = irq_data_get_irq_chip_data(data);
	u32 packed_msg;

	packed_msg = FIELD_PREP(LANTIQ_MSI_PIC_TABLE_MSG_DATA, msg->data) |
		     FIELD_PREP(LANTIQ_MSI_PIC_TABLE_MSG_ADDR,
				msg->address_lo >> 12) |
		     FIELD_PREP(LANTIQ_MSI_PIC_TABLE_INT_LINE, data->hwirq);

	if (irqd_irq_disabled(data))
		packed_msg |= LANTIQ_MSI_PIC_TABLE_INT_DISABLE;

	iowrite32be(packed_msg,
		    priv->pic_base + LANTIQ_MSI_PIC_TABLE(data->hwirq));
}

static struct irq_chip lantiq_msi_pic_irq_chip_top = {
	.name			= "PCI-MSIx",
	.irq_mask		= lantiq_msi_pic_mask_msi_irq,
	.irq_unmask		= lantiq_msi_pic_unmask_msi_irq,
	.irq_write_msi_msg	= lantiq_msi_pic_write_msi_msg,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

static void lantiq_msi_pic_compose_msi_msg(struct irq_data *data,
					   struct msi_msg *msg)
{
	struct ltq_msi_pic_data *priv = irq_data_get_irq_chip_data(data);

	msg->address_hi = upper_32_bits(priv->msi_addr);
	msg->address_lo = lower_32_bits(priv->msi_addr);
	msg->data = LANTIQ_MSI_BASE_DATA | BIT(data->hwirq); // TODO: masking needed?
}

static struct msi_domain_info lantiq_msi_pic_domain_info = {
	.flags	= MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_PCI_MSIX,
	.chip	= &lantiq_msi_pic_irq_chip_top,
};

static struct irq_chip lantiq_msi_pic_irq_chip_middle = {
	.name			= "LANTIQ-MSI-PIC-middle",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_compose_msi_msg	= lantiq_msi_pic_compose_msi_msg,
};

static int lantiq_msi_pic_middle_allocate_hwirq(struct ltq_msi_pic_data *priv,
						int num_req)
{
	int first;

	mutex_lock(&priv->msi_map_lock);

	first = bitmap_find_free_region(priv->msi_map, priv->nr_irqs,
					get_count_order(num_req));
	if (first < 0) {
		mutex_unlock(&priv->msi_map_lock);
		return -ENOSPC;
	}

	mutex_unlock(&priv->msi_map_lock);

	return priv->irqs[0] + first;
}

static void lantiq_msi_pic_middle_free_hwirq(struct ltq_msi_pic_data *priv,
					     int hwirq, int num_req)
{
	int first = priv->irqs[0] + hwirq;

	mutex_lock(&priv->msi_map_lock);
	bitmap_release_region(priv->msi_map, first, get_count_order(num_req));
	mutex_unlock(&priv->msi_map_lock);
}

static int lantiq_msi_pic_middle_parent_domain_alloc(struct irq_domain *domain,
						     unsigned int virq,
						     int hwirq)
{
	struct irq_fwspec fwspec;

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 1;
	fwspec.param[0] = hwirq;

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
}

static int lantiq_msi_pic_middle_domain_alloc(struct irq_domain *domain,
					      unsigned int virq,
					      unsigned int nr_irqs, void *args)
{
	struct ltq_msi_pic_data *priv = domain->host_data;
	int hwirq, err, i;

	hwirq = lantiq_msi_pic_middle_allocate_hwirq(priv, nr_irqs);
	if (hwirq < 0)
		return hwirq;

	for (i = 0; i < nr_irqs; i++) {
		err = lantiq_msi_pic_middle_parent_domain_alloc(domain,
								virq + i,
								hwirq + i);
		if (err)
			goto err_hwirq;

		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &lantiq_msi_pic_irq_chip_middle,
					      priv);
	}

	return 0;

err_hwirq:
	lantiq_msi_pic_middle_free_hwirq(priv, hwirq, nr_irqs);
	irq_domain_free_irqs_parent(domain, virq, i - 1);
	return err;
}

static void lantiq_msi_pic_middle_domain_free(struct irq_domain *domain,
					      unsigned int virq,
					      unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct ltq_msi_pic_data *priv = irq_data_get_irq_chip_data(d);

	lantiq_msi_pic_middle_free_hwirq(priv, virq, nr_irqs);
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
}

static const struct irq_domain_ops lantiq_msi_pic_middle_domain_ops = {
	.alloc	= lantiq_msi_pic_middle_domain_alloc,
	.free	= lantiq_msi_pic_middle_domain_free,
};

static int lantiq_msi_pic_probe(struct platform_device *pdev)
{
	struct fwnode_handle *fwnode = of_node_to_fwnode(pdev->dev.of_node);
	struct irq_domain *middle_domain, *msi_domain, *icu_domain;
	struct device *dev = &pdev->dev;
	struct resource *msi_resource;
	struct ltq_msi_pic_data *priv;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	mutex_init(&priv->msi_map_lock);

	priv->nr_irqs = of_property_read_variable_u32_array(dev->of_node,
							    "lantiq,msi-irqs",
							    priv->irqs,
							    LANTIQ_MSI_NUM_MIN,
							    LANTIQ_MSI_NUM_MAX);
	if (priv->nr_irqs < 0)
		return priv->nr_irqs;

	for (i = 0; i < priv->nr_irqs; i++) {
		if ((i + 1) >= priv->nr_irqs)
			break;

		if ((priv->irqs[i] + 1) != priv->irqs[i + 1]) {
			dev_err(dev,
				"The MSI IRQ numbering must be continuous\n");
			return -EINVAL;
		}
	}

	priv->msi_map = bitmap_alloc(priv->nr_irqs, GFP_KERNEL);
	if (!priv->msi_map)
		return -ENOMEM;

	priv->pic_base = devm_platform_ioremap_resource_byname(pdev, "pic");
	if (IS_ERR(priv->pic_base)) {
		ret = PTR_ERR(priv->pic_base);
		goto err_free_msi_bitmap;
	}

	msi_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						    "msi");
	if (IS_ERR(msi_resource)) {
		ret = PTR_ERR(msi_resource);
		dev_err(dev, "Unable to get the MSI resource\n");
		goto err_free_msi_bitmap;
	}

	priv->msi_addr = msi_resource->start;

	icu_domain = irq_find_host(of_irq_find_parent(dev->of_node));
	if (!icu_domain) {
		dev_err(dev, "Failed to find the parent (ICU) domain\n");
		ret = -ENODEV;
		goto err_free_msi_bitmap;
	}

	priv->clk = devm_clk_get(&pdev->dev, 0);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_err_probe(dev, ret, "Failed to get clock\n");
		goto err_free_msi_bitmap;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "Failed to enable the MSI clock\n");
		goto err_free_msi_bitmap;
	}

	iowrite32be(LANTIQ_MSI_PIC_ENDIAN_BIG,
		    priv->pic_base + LANTIQ_MSI_PIC_ENDIAN);

	/* start with all MSI IRQs disabled */
	for (i = 0; i < LANTIQ_MSI_NUM_MAX; i++)
		lantiq_msi_pic_set_msi_irq_enabled(priv, i, false);

	middle_domain = irq_domain_create_linear(fwnode, priv->nr_irqs,
						 &lantiq_msi_pic_middle_domain_ops,
						 priv);
	if (!middle_domain) {
		dev_err(dev, "Failed to create the MSI middle domain\n");
		ret = -ENOMEM;
		goto err_disable_clk;
	}

	middle_domain->parent = icu_domain;

	msi_domain = pci_msi_create_irq_domain(fwnode,
					       &lantiq_msi_pic_domain_info,
					       middle_domain);
	if (!msi_domain) {
		dev_err(dev, "Failed to create MSI domain\n");
		ret = -ENOMEM;
		goto err_remove_mid_domain;
	}

	return 0;

err_remove_mid_domain:
	irq_domain_remove(middle_domain);
err_disable_clk:
	clk_disable_unprepare(priv->clk);
err_free_msi_bitmap:
	bitmap_free(priv->msi_map);
	return ret;
}

static int lantiq_msi_pic_remove(struct platform_device *pdev)
{
	struct ltq_msi_pic_data *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < LANTIQ_MSI_NUM_MAX; i++)
		lantiq_msi_pic_set_msi_irq_enabled(priv, i, false);

	clk_disable_unprepare(priv->clk);
	bitmap_free(priv->msi_map);

	return 0;
}

static const struct of_device_id lantiq_msi_pic_of_match[] = {
	{ .compatible = "lantiq,xrx200-msi-pic" },
	{ .compatible = "lantiq,xrx300-msi-pic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lantiq_msi_pic_of_match);

static struct platform_driver lantiq_msi_pic_driver = {
	.probe		= lantiq_msi_pic_probe,
	.remove		= lantiq_msi_pic_remove,
	.driver		= {
		.name	= "lantiq-msi-pic",
		.of_match_table	= lantiq_msi_pic_of_match,
	}
};
module_platform_driver(lantiq_msi_pic_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq MSI PIC");
MODULE_LICENSE("GPL v2");
