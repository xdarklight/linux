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

#define MAX_VBUS_GPIO		2

struct xway_usb2_priv {
	struct device			*dev;
	struct xway_usb2_port_priv	*phys;
	int				nphys;
	struct gpio_desc		*gpiod_vbus[MAX_VBUS_GPIO];
};

struct xway_usb2_port_priv {
	struct phy		*phy;
	u8			port;
	struct clk		*ctrl_gate_clk;
	struct clk		*phy_gate_clk;
	struct reset_control	*analog_config_reset;
	struct reset_control	*hostmode_reset;
	struct reset_control	*slave_endian_reset;
	struct reset_control	*host_endian_reset;
	struct reset_control	*state_hard_reset;
	struct reset_control	*state_soft_reset;
	struct gpio_desc	*gpiod_vbus[MAX_VBUS_GPIO];
};

static void xway_usb2_set_vbus_gpio_value(struct gpio_desc **gpiods, int value)
{
	int i;

	for (i = 0; i < MAX_VBUS_GPIO; i++)
		if (!IS_ERR_OR_NULL(gpiods[i]))
			gpiod_set_value(gpiods[i], value);
}

static int xway_usb2_phy_power_on(struct phy *phy)
{
	struct xway_usb2_port_priv *port_priv = phy_get_drvdata(phy);
	struct xway_usb2_priv *priv = dev_get_drvdata(phy->dev.parent);

	if (port_priv->analog_config_reset)
		reset_control_assert(port_priv->analog_config_reset);

	/* enable the port-specific VBUS GPIOs if available */
	xway_usb2_set_vbus_gpio_value(port_priv->gpiod_vbus, 1);

	/* enable the shared VBUS GPIO if available */
	xway_usb2_set_vbus_gpio_value(priv->gpiod_vbus, 1);

	return 0;
}

static int xway_usb2_phy_power_off(struct phy *phy)
{
	struct xway_usb2_port_priv *port_priv = phy_get_drvdata(phy);

	/*
	 * only disable the port-specific VBUS GPIO here (if available), the
	 * shared VBUS GPIO might still be used by another port
	 */
	xway_usb2_set_vbus_gpio_value(port_priv->gpiod_vbus, 0);

	if (port_priv->analog_config_reset)
		reset_control_deassert(port_priv->analog_config_reset);

	return 0;
}

static struct phy *xway_usb2_phy_xlate(struct device *dev,
				       struct of_phandle_args *args)
{
	struct xway_usb2_priv *priv = dev_get_drvdata(dev);
	int idx;

	if (WARN_ON(args->args[0] >= priv->nphys))
		return ERR_PTR(-ENODEV);

	for (idx = 0; idx < priv->nphys; idx++) {
		if (priv->phys[idx].port == args->args[0])
			break;
	}

	/*
	 * idx must be a valid phy index and probing of the phy at that index
	 * must not have failed.
	 */
	if (idx== priv->nphys || !priv->phys[idx].phy)
		return ERR_PTR(-ENODEV);

	return priv->phys[idx].phy;
}

static struct phy_ops xway_usb2_phy_ops = {
	.power_on	= xway_usb2_phy_power_on,
	.power_off	= xway_usb2_phy_power_off,
	.owner		= THIS_MODULE,
};

static void xway_usb2_start_cores(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xway_usb2_port_priv *port_priv;
	struct xway_usb2_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->nphys; i++) {
		/* Skip PHYs for which probing has failed */
		if (!priv->phys[i].phy)
			continue;

		port_priv = phy_get_drvdata(priv->phys[i].phy);

		/* Power on the USB core. */
		if (clk_prepare_enable(port_priv->ctrl_gate_clk)) {
			dev_err(dev, "failed to enable port %d CTRL gate\n",
				port_priv->port);
			continue;
		}

		/*
		 * Power on the USB PHY. We have to do it early because
		 * otherwise the second core won't turn on properly.
		 */
		if (clk_prepare_enable(port_priv->phy_gate_clk)) {
			dev_err(dev, "failed to enable port %d PHY gate\n",
				port_priv->port);
			continue;
		}

		/* Configure core to host mode */
		reset_control_deassert(port_priv->hostmode_reset);

		/* Select DMA endianness (Host-endian: big-endian) */
		reset_control_deassert(port_priv->slave_endian_reset);
		reset_control_assert(port_priv->host_endian_reset);

		/* Hard reset USB state machines */
		reset_control_assert(port_priv->state_hard_reset);
		udelay(50 * 1000);
		reset_control_deassert(port_priv->state_hard_reset);

		/* Soft reset USB state machines */
		reset_control_assert(port_priv->state_soft_reset);
		udelay(50 * 1000);
		reset_control_deassert(port_priv->state_soft_reset);
	}
}

static int xway_usb2_get_vbus_gpios(struct device *dev,
				    struct gpio_desc **gpios)
{
	int i;

	for (i = 0; i < MAX_VBUS_GPIO; i++) {
		gpios[i] = devm_gpiod_get_index_optional(dev, "vbus", i,
							 GPIOD_OUT_LOW);
		if (IS_ERR(gpios[i]))
			return PTR_ERR(gpios[i]);
	}

	return 0;
}

static int xway_usb2_of_probe_phy_port(struct phy *phy)
{
	struct device *dev = &phy->dev;
	struct xway_usb2_port_priv *port_priv = phy_get_drvdata(phy);
	struct reset_control *reset;
	int ret;

	if (of_property_read_u8(dev->of_node, "reg", &port_priv->port)) {
		dev_err(dev, "failed to read reg property\n");
		return -EINVAL;
	}

	ret = xway_usb2_get_vbus_gpios(dev, port_priv->gpiod_vbus);
	if (ret) {
		dev_err(dev, "failed to request USB%d VBUS GPIOs\n",
			port_priv->port);
		return ret;
	}

	port_priv->ctrl_gate_clk = devm_clk_get(dev, "ctrl");
	if (IS_ERR(port_priv->ctrl_gate_clk)) {
		dev_err(dev, "Unable to get USB%d ctrl gate clk\n",
			port_priv->port);
		return PTR_ERR(port_priv->ctrl_gate_clk);
	}

	port_priv->phy_gate_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(port_priv->phy_gate_clk)) {
		dev_err(dev, "Unable to get USB%d phy gate clk\n",
			port_priv->port);
		return PTR_ERR(port_priv->phy_gate_clk);
	}

	reset = devm_reset_control_get_optional(dev, "analog-config");
	if (IS_ERR(reset)) {
		if (PTR_ERR(reset) == -EPROBE_DEFER) {
			dev_err(dev, "failed to get 'analog-config' reset\n");
			return PTR_ERR(reset);
		} else {
			reset = NULL;
		}
	}

	port_priv->analog_config_reset = reset;

	reset = devm_reset_control_get(dev, "hostmode");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'hostmode' reset\n");
		return PTR_ERR(reset);
	}

	port_priv->hostmode_reset = reset;

	reset = devm_reset_control_get(dev, "slave-endianness");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'slave-endianness' reset\n");
		return PTR_ERR(reset);
	}

	port_priv->slave_endian_reset = reset;

	reset = devm_reset_control_get(dev, "host-endianness");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'host-endianness' reset\n");
		return PTR_ERR(reset);
	}

	port_priv->host_endian_reset = reset;

	reset = devm_reset_control_get(dev, "statemachine-hard");
	if (IS_ERR(reset)) {
		dev_err(dev, "failed to get 'statemachine-hard' reset\n");
		return PTR_ERR(reset);
	}

	port_priv->state_hard_reset = reset;

	reset = devm_reset_control_get_optional(dev, "statemachine-soft");
	if (IS_ERR(reset)) {
		if (PTR_ERR(reset) == -EPROBE_DEFER) {
			dev_err(dev,
				"failed to get 'statemachine-soft' reset\n");
			return PTR_ERR(reset);
		} else {
			reset = NULL;
		}
	}

	port_priv->state_soft_reset = reset;

	return 0;
}

static int xway_usb2_of_probe(struct device_node *phynode,
				   struct xway_usb2_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	ret = xway_usb2_get_vbus_gpios(dev, priv->gpiod_vbus);
	if (ret) {
		dev_err(dev, "failed to request shared USB VBUS GPIO\n");
		return ret;
	}

	return 0;
}

static int xway_usb2_phy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct xway_usb2_priv *priv;
	struct phy_provider *provider;
	int ret, port = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv),
				       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->nphys = of_get_child_count(np);
	priv->phys = devm_kcalloc(&pdev->dev, priv->nphys, sizeof(priv->phys),
				  GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	dev_set_drvdata(priv->dev, priv);

	ret = xway_usb2_of_probe(np, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to probe PHY\n");
		return ret;
	}

	for_each_child_of_node(np, child) {
		struct phy *phy;
		struct xway_usb2_port_priv *port_priv = &priv->phys[port];

		port_priv->port = port++;

		phy = devm_phy_create(&pdev->dev, child, &xway_usb2_phy_ops);
		if (IS_ERR(phy)) {
			dev_err(&pdev->dev, "failed to create PHY %d\n", port);
			continue;
		}

		phy_set_drvdata(phy, port_priv);

		ret = xway_usb2_of_probe_phy_port(phy);
		if (ret) {
			devm_phy_destroy(&pdev->dev, phy);
			dev_err(&pdev->dev, "failed to probe port %d\n", port);
			continue;
		}

		port_priv->phy = phy;
	}

	xway_usb2_start_cores(pdev);

	provider = devm_of_phy_provider_register(&pdev->dev,
						 xway_usb2_phy_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id xway_usb2_phy_of_match[] = {
	{ .compatible = "lantiq,arx300-usb2-phy", },
	{ .compatible = "lantiq,ase-usb2-phy", },
	{ .compatible = "lantiq,danube-usb2-phy", },
	{ .compatible = "lantiq,grx300-usb2-phy", },
	{ .compatible = "lantiq,xrx100-usb2-phy", },
	{ .compatible = "lantiq,xrx200-usb2-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, xway_usb2_phy_of_match);

static struct platform_driver xway_usb2_port_priv_driver = {
	.probe	= xway_usb2_phy_probe,
	.driver = {
		.name	= "xway-usb2-phy",
		.of_match_table	= xway_usb2_phy_of_match,
	}
};
module_platform_driver(xway_usb2_port_priv_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY USB2 PHY driver");
MODULE_LICENSE("GPL v2");
