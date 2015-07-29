/*
 * Lantiq XWAY SoC RCU module based USB 1.1/2.0 PHY driver
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define MAX_VBUS_GPIO		2

struct ltq_rcu_usb2_bits {
	struct {
		u8 hostmode;
		u8 slave_endianness;
		u8 host_endianness;
	} phy;
	struct {
		u8 hardreset;
	} mac;
};

struct ltq_rcu_usb2_priv {
	struct regmap			*regmap;
	u32				phy_reg_offset;
	u32				mac_reg_offset;
	const struct ltq_rcu_usb2_bits	*reg_bits;
	struct device			*dev;
	struct ltq_rcu_usb2_port_priv	*phys;
	int				num_phys;
	struct gpio_desc		*gpiod_vbus[MAX_VBUS_GPIO];
};

struct ltq_rcu_usb2_port_priv {
	struct phy		*phy;
	u8			port;
	struct clk		*ctrl_gate_clk;
	struct clk		*phy_gate_clk;
	struct reset_control	*analog_config_reset;
	struct reset_control	*state_soft_reset;
	struct gpio_desc	*gpiod_vbus[MAX_VBUS_GPIO];
};

static struct ltq_rcu_usb2_bits xway_rcu_usb2_reg_bits = {
	.phy = {
		.hostmode = 11,
		.slave_endianness = 9,
		.host_endianness = 10,
	},
	.mac = {
		.hardreset = 4,
	},
};

static struct ltq_rcu_usb2_bits xrx100_rcu_usb2_reg_bits = {
	.phy = {
		.hostmode = 11,
		.slave_endianness = 17,
		.host_endianness = 10,
	},
	.mac = {
		.hardreset = 28,
	},
};

static const struct of_device_id ltq_rcu_usb2_phy_of_match[] = {
	{
		.compatible = "lantiq,ase-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,danube-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx100-rcu-usb2-phy",
		.data = &xrx100_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx200-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx300-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ltq_rcu_usb2_phy_of_match);

static void ltq_rcu_usb2_set_vbus_gpio_value(struct gpio_desc **gpiods,
						int value)
{
	int i;

	for (i = 0; i < MAX_VBUS_GPIO; i++)
		if (!IS_ERR_OR_NULL(gpiods[i]))
			gpiod_set_value(gpiods[i], value);
}

static int ltq_rcu_usb2_phy_power_on(struct phy *phy)
{
	struct ltq_rcu_usb2_port_priv *port_priv = phy_get_drvdata(phy);
	struct ltq_rcu_usb2_priv *priv = dev_get_drvdata(phy->dev.parent);

	if (port_priv->analog_config_reset)
		reset_control_assert(port_priv->analog_config_reset);

	/* enable the port-specific VBUS GPIOs if available */
	ltq_rcu_usb2_set_vbus_gpio_value(port_priv->gpiod_vbus, 1);

	/* enable the shared VBUS GPIO if available */
	ltq_rcu_usb2_set_vbus_gpio_value(priv->gpiod_vbus, 1);

	return 0;
}

static int ltq_rcu_usb2_phy_power_off(struct phy *phy)
{
	struct ltq_rcu_usb2_port_priv *port_priv = phy_get_drvdata(phy);

	/*
	 * only disable the port-specific VBUS GPIO here (if available), the
	 * shared VBUS GPIO might still be used by another port
	 */
	ltq_rcu_usb2_set_vbus_gpio_value(port_priv->gpiod_vbus, 0);

	if (port_priv->analog_config_reset)
		reset_control_deassert(port_priv->analog_config_reset);

	return 0;
}

static struct phy *ltq_rcu_usb2_phy_xlate(struct device *dev,
					     struct of_phandle_args *args)
{
	struct ltq_rcu_usb2_priv *priv = dev_get_drvdata(dev);
	int idx;

	if (WARN_ON(args->args[0] >= priv->num_phys))
		return ERR_PTR(-ENODEV);

	for (idx = 0; idx < priv->num_phys; idx++) {
		if (priv->phys[idx].port == args->args[0])
			break;
	}

	if (idx >= priv->num_phys)
		return ERR_PTR(-ENODEV);

	return priv->phys[idx].phy;
}

static struct phy_ops ltq_rcu_usb2_phy_ops = {
	.power_on	= ltq_rcu_usb2_phy_power_on,
	.power_off	= ltq_rcu_usb2_phy_power_off,
	.owner		= THIS_MODULE,
};

static void ltq_rcu_usb2_start_cores(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ltq_rcu_usb2_port_priv *port_priv;
	struct ltq_rcu_usb2_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->num_phys; i++) {
		/* Skip PHYs for which probing has failed */
		if (IS_ERR_OR_NULL(priv->phys[i].phy))
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
		regmap_update_bits(priv->regmap, priv->phy_reg_offset,
				   BIT(priv->reg_bits->phy.hostmode), 0);

		/* Select DMA endianness (Host-endian: big-endian) */
		regmap_update_bits(priv->regmap, priv->phy_reg_offset,
			BIT(priv->reg_bits->phy.slave_endianness), 0);
		regmap_update_bits(priv->regmap, priv->phy_reg_offset,
			BIT(priv->reg_bits->phy.host_endianness),
			BIT(priv->reg_bits->phy.host_endianness));

		/* Hard reset USB state machines */
		regmap_update_bits(priv->regmap, priv->mac_reg_offset,
			BIT(priv->reg_bits->mac.hardreset),
			BIT(priv->reg_bits->mac.hardreset));
		udelay(50 * 1000);
		regmap_update_bits(priv->regmap, priv->mac_reg_offset,
			BIT(priv->reg_bits->mac.hardreset), 0);

		/* Soft reset USB state machines (not available on all SoCs) */
		if (port_priv->state_soft_reset)
			reset_control_reset(port_priv->state_soft_reset);
	}
}

static int ltq_rcu_usb2_get_vbus_gpios(struct device *dev,
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

static int ltq_rcu_usb2_of_probe_phy_port(struct phy *phy)
{
	struct device *dev = &phy->dev;
	struct ltq_rcu_usb2_port_priv *port_priv = phy_get_drvdata(phy);
	struct reset_control *reset;
	int ret;

	if (of_property_read_u8(dev->of_node, "reg", &port_priv->port)) {
		dev_err(dev, "failed to read reg property\n");
		return -EINVAL;
	}

	ret = ltq_rcu_usb2_get_vbus_gpios(dev, port_priv->gpiod_vbus);
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
		}

		reset = NULL;
	}

	port_priv->analog_config_reset = reset;

	reset = devm_reset_control_get_optional(dev, "statemachine-soft");
	if (IS_ERR(reset)) {
		if (PTR_ERR(reset) == -EPROBE_DEFER) {
			dev_err(dev,
				"failed to get 'statemachine-soft' reset\n");
			return PTR_ERR(reset);
		}

		reset = NULL;
	}

	port_priv->state_soft_reset = reset;

	return 0;
}

static int ltq_rcu_usb2_of_probe(struct device_node *phynode,
				    struct ltq_rcu_usb2_priv *priv)
{
	struct device *dev = priv->dev;
	const struct of_device_id *match =
		of_match_node(ltq_rcu_usb2_phy_of_match, phynode);
	int ret;

	if (!match) {
		dev_err(dev, "Not a compatible Lantiq RCU USB PHY\n");
		return -EINVAL;
	}

	priv->reg_bits = match->data;

	priv->regmap = syscon_regmap_lookup_by_phandle(phynode,
						       "lantiq,rcu-syscon");
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = of_property_read_u32_index(phynode, "lantiq,rcu-syscon", 1,
					 &priv->phy_reg_offset);
	if (ret) {
		dev_err(dev, "Failed to get RCU PHY reg offset\n");
		return ret;
	}

	ret = of_property_read_u32_index(phynode, "lantiq,rcu-syscon", 2,
					 &priv->mac_reg_offset);
	if (ret) {
		dev_err(dev, "Failed to get RCU MAC reg offset\n");
		return ret;
	}

	ret = ltq_rcu_usb2_get_vbus_gpios(dev, priv->gpiod_vbus);
	if (ret) {
		dev_err(dev, "failed to request shared USB VBUS GPIO\n");
		return ret;
	}

	return 0;
}

static int ltq_rcu_usb2_phy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct ltq_rcu_usb2_priv *priv;
	struct phy_provider *provider;
	int ret, port = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv),
				       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->num_phys = of_get_child_count(np);
	priv->phys = devm_kcalloc(&pdev->dev, priv->num_phys,
				  sizeof(priv->phys), GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	dev_set_drvdata(priv->dev, priv);

	ret = ltq_rcu_usb2_of_probe(np, priv);
	if (ret)
		return ret;

	for_each_child_of_node(np, child) {
		struct ltq_rcu_usb2_port_priv *port_priv =
			&priv->phys[port];

		port_priv->port = port++;

		port_priv->phy = devm_phy_create(&pdev->dev, child,
						 &ltq_rcu_usb2_phy_ops);
		if (IS_ERR(port_priv->phy)) {
			dev_err(&pdev->dev, "failed to create PHY %d\n", port);
			continue;
		}

		phy_set_drvdata(port_priv->phy, port_priv);

		ret = ltq_rcu_usb2_of_probe_phy_port(port_priv->phy);
		if (ret) {
			devm_phy_destroy(&pdev->dev, port_priv->phy);
			port_priv->phy = ERR_PTR(ret);
			dev_err(&pdev->dev, "failed to probe port %d\n", port);
			continue;
		}
	}

	ltq_rcu_usb2_start_cores(pdev);

	provider = devm_of_phy_provider_register(&pdev->dev,
						 ltq_rcu_usb2_phy_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static struct platform_driver ltq_rcu_usb2_port_priv_driver = {
	.probe	= ltq_rcu_usb2_phy_probe,
	.driver = {
		.name	= "lantiq-rcu-usb2-phy",
		.of_match_table	= ltq_rcu_usb2_phy_of_match,
	}
};
module_platform_driver(ltq_rcu_usb2_port_priv_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY USB2 PHY driver");
MODULE_LICENSE("GPL v2");
