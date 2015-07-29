/*
 * Lantiq xway SoC USB 1.1/2.0 PHY driver
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

struct xway_usb2_phy_dev {
	struct device		*dev;
	struct xway_usb2_phy	**phys;
	int			nphys;
};

struct xway_usb2_phy {
	struct phy		*phy;
	u8			port;
	struct clk		*ctrl_gate_clk;
	struct clk		*phy_gate_clk;
	struct reset_control	*config_reset;
	struct reset_control	*hostmode_reset;
	struct reset_control	*slave_endian_reset;
	struct reset_control	*host_endian_reset;
	struct reset_control	*state_hard_reset;
	struct reset_control	*state_soft_reset;
	struct gpio_desc	*gpiod_vbus;
};

static int xway_usb2_phy_init(struct phy *phy)
{
	struct xway_usb2_phy *priv = phy_get_drvdata(phy);

	reset_control_assert(priv->config_reset);

	/* Configure cores to host mode */
	reset_control_deassert(priv->hostmode_reset);

	/* Select DMA endianness (Host-endian: big-endian) */
	reset_control_deassert(priv->slave_endian_reset);
	reset_control_assert(priv->host_endian_reset);

	/* Hard reset USB state machines */
	reset_control_assert(priv->state_hard_reset);
	udelay(50 * 1000);
	reset_control_deassert(priv->state_hard_reset);

	/* Soft reset USB state machines */
	reset_control_assert(priv->state_soft_reset);
	udelay(50 * 1000);
	reset_control_deassert(priv->state_soft_reset);

	return 0;
}

static int xway_usb2_phy_power_on(struct phy *phy)
{
	struct xway_usb2_phy *priv = phy_get_drvdata(phy);
	int ret;

	ret = clk_prepare_enable(priv->ctrl_gate_clk);
	if (ret)
		goto err_ctrl_clk;
	ret = clk_prepare_enable(priv->phy_gate_clk);
	if (ret)
		goto err_phy_clk;

	if (priv->gpiod_vbus)
		gpiod_set_value(priv->gpiod_vbus, 1);

	return 0;

err_phy_clk:
	clk_disable_unprepare(priv->ctrl_gate_clk);
err_ctrl_clk:
	return ret;
}

static int xway_usb2_phy_power_off(struct phy *phy)
{
	struct xway_usb2_phy *priv = phy_get_drvdata(phy);

	if (priv->gpiod_vbus)
		gpiod_set_value(priv->gpiod_vbus, 0);

	clk_disable_unprepare(priv->phy_gate_clk);
	clk_disable_unprepare(priv->ctrl_gate_clk);

	return 0;
}

static struct phy *xway_usb2_phy_xlate(struct device *dev,
				       struct of_phandle_args *args)
{
	struct xway_usb2_phy_dev *priv = dev_get_drvdata(dev);
	int i;

	if (WARN_ON(args->args[0] >= priv->nphys))
		return ERR_PTR(-ENODEV);

	for (i = 0; i < priv->nphys; i++) {
		if (priv->phys[i]->port == args->args[0])
			break;
	}

	if (i == priv->nphys)
		return ERR_PTR(-ENODEV);

	return priv->phys[i]->phy;
}

static struct phy_ops xway_usb2_phy_ops = {
	.init		= xway_usb2_phy_init,
	.power_on	= xway_usb2_phy_power_on,
	.power_off	= xway_usb2_phy_power_off,
	.owner		= THIS_MODULE,
};

static int xway_usb2_of_probe(struct device *dev,
			      struct device_node *phynode,
			      struct xway_usb2_phy *phy_priv)
{
	char gpio_name[9];
	struct reset_control *reset;
	unsigned vbus_gpio;

	if (of_property_read_u8(phynode, "reg", &phy_priv->port)) {
		dev_err(dev, "failed to read reg property\n");
		return -EINVAL;
	}

	vbus_gpio = of_get_named_gpio(phynode, "vbus", 0);
	if (gpio_is_valid(vbus_gpio)) {
		snprintf(gpio_name, sizeof(gpio_name) / sizeof(char),
				"usb-vbus%d", phy_priv->port);

		if (devm_gpio_request(dev, vbus_gpio, gpio_name)) {
			dev_err(dev, "failed to request %s GPIO\n", gpio_name);
			return -EINVAL;
		}

		phy_priv->gpiod_vbus = gpio_to_desc(vbus_gpio);
	}

	phy_priv->ctrl_gate_clk = of_clk_get_by_name(phynode, "ctrl");
	if (IS_ERR(phy_priv->ctrl_gate_clk)) {
		dev_err(dev, "Unable to get USB%d ctrl gate clk\n",
			phy_priv->port);
		return PTR_ERR(phy_priv->ctrl_gate_clk);
	}

	phy_priv->phy_gate_clk = of_clk_get_by_name(phynode, "phy");
	if (IS_ERR(phy_priv->phy_gate_clk)) {
		dev_err(dev, "Unable to get USB%d phy gate clk\n",
			phy_priv->port);
		return PTR_ERR(phy_priv->phy_gate_clk);
	}

	reset = of_reset_control_get(phynode, "config");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'config' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->config_reset = reset;

	reset = of_reset_control_get(phynode, "hostmode");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'hostmode' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->hostmode_reset = reset;

	reset = of_reset_control_get(phynode, "slave-endianness");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'slave-endianness' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->slave_endian_reset = reset;

	reset = of_reset_control_get(phynode, "host-endianness");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'host-endianness' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->host_endian_reset = reset;

	reset = of_reset_control_get(phynode, "statemachine-hard");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'statemachine-hard' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->state_hard_reset = reset;

	reset = of_reset_control_get(phynode, "statemachine-soft");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'statemachine-soft' reset\n");
		return PTR_ERR(reset);
	}

	phy_priv->state_soft_reset = reset;

	return 0;
}

static int xway_usb2_phy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct xway_usb2_phy_dev *xway_usbphy_dev;
	struct phy_provider *provider;
	struct phy *phy;
	int ret, port = 0;

	xway_usbphy_dev = devm_kzalloc(&pdev->dev, sizeof(*xway_usbphy_dev),
				       GFP_KERNEL);
	if (!xway_usbphy_dev)
		return -ENOMEM;

	xway_usbphy_dev->nphys = of_get_child_count(np);
	xway_usbphy_dev->phys = devm_kcalloc(&pdev->dev,
					     xway_usbphy_dev->nphys,
					     sizeof(*xway_usbphy_dev->phys),
					     GFP_KERNEL);
	if (!xway_usbphy_dev->phys)
		return -ENOMEM;

	xway_usbphy_dev->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, xway_usbphy_dev);

	for_each_child_of_node(np, child) {
		struct xway_usb2_phy *phy_priv;

		phy_priv = devm_kzalloc(&pdev->dev, sizeof(*phy_priv),
					GFP_KERNEL);
		if (!phy_priv)
			return -ENOMEM;

		xway_usbphy_dev->phys[port] = phy_priv;
		phy_priv->port = port;

		phy = devm_phy_create(&pdev->dev, child, &xway_usb2_phy_ops);
		if (IS_ERR(phy)) {
			devm_kfree(&pdev->dev, phy_priv);
			dev_err(&pdev->dev, "failed to create PHY %d\n", port);
			continue;
		}

		phy_priv->phy = phy;

		ret = xway_usb2_of_probe(&pdev->dev, child, phy_priv);
		if (ret) {
			devm_phy_destroy(&pdev->dev, phy);
			devm_kfree(&pdev->dev, phy_priv);
			dev_err(&pdev->dev, "failed to probe PHY %d\n", port);
			continue;
		}

		phy_set_drvdata(phy, xway_usbphy_dev->phys[port]);
		port++;
	}

	provider = devm_of_phy_provider_register(&pdev->dev,
						 xway_usb2_phy_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id xway_usb2_phy_of_match[] = {
	{ .compatible = "lantiq,arx100-usb2-phy", },
	{ .compatible = "lantiq,arx300-usb2-phy", },
	{ .compatible = "lantiq,ase-usb2-phy", },
	{ .compatible = "lantiq,danube-usb2-phy", },
	{ .compatible = "lantiq,grx390-usb2-phy", },
	{ .compatible = "lantiq,vrx200-usb2-phy", },
	{ .compatible = "lantiq,xway-usb2-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, xway_usb2_phy_of_match);

static struct platform_driver xway_usb2_phy_driver = {
	.probe	= xway_usb2_phy_probe,
	.driver = {
		.name	= "xway-usb2-phy",
		.of_match_table	= xway_usb2_phy_of_match,
	}
};
module_platform_driver(xway_usb2_phy_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("lantiq xway USB2 phy driver");
MODULE_LICENSE("GPL v2");
