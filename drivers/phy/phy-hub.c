/*
 * PHY hub driver - a virtual PHY device which passes all phy_* functions to
 * multiple (actual) PHY devices. This can be handy when (for example) a USB
 * controller provides a USB hub where the controller wants to keep the PHYs
 * for all ports in the same state at all times.
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/of.h>
#include <linux/workqueue.h>

struct phy_hub_priv {
	int			num_phy_ports;
	struct phy		**phy_ports;
};

#define APPLY_TO_EACH_PHY_PORT(func, phy, args...)			\
	{								\
		int i, ret;						\
		struct phy_hub_priv *priv = phy_get_drvdata(phy);	\
									\
		for (i = 0; i < priv->num_phy_ports; i++) {		\
			if (!priv->phy_ports[i])			\
				continue;				\
			ret = phy_##func(priv->phy_ports[i], ##args);	\
			if (ret) {					\
				dev_err(&phy->dev,			\
					"Failed to #func port %d\n", i);\
				return ret;				\
			}						\
		}							\
	}

static int phy_usb_hub_init(struct phy *phy)
{
	APPLY_TO_EACH_PHY_PORT(init, phy);

	return 0;
}

static int phy_usb_hub_exit(struct phy *phy)
{
	APPLY_TO_EACH_PHY_PORT(exit, phy);

	return 0;
}

static int phy_usb_hub_power_on(struct phy *phy)
{
	APPLY_TO_EACH_PHY_PORT(power_on, phy);

	return 0;
}

static int phy_usb_hub_power_off(struct phy *phy)
{
	APPLY_TO_EACH_PHY_PORT(power_off, phy);

	return 0;
}

static int phy_usb_hub_set_mode(struct phy *phy, enum phy_mode mode)
{
	APPLY_TO_EACH_PHY_PORT(set_mode, phy, mode);

	return 0;
}

static int phy_usb_hub_reset(struct phy *phy)
{
	APPLY_TO_EACH_PHY_PORT(reset, phy);

	return 0;
}

static const struct phy_ops phy_usb_hub_ops = {
	.init		= phy_usb_hub_init,
	.exit		= phy_usb_hub_exit,
	.power_on	= phy_usb_hub_power_on,
	.power_off	= phy_usb_hub_power_off,
	.set_mode	= phy_usb_hub_set_mode,
	.reset		= phy_usb_hub_reset,
	.owner		= THIS_MODULE,
};

/**
 * Create a new virtual PHY hub device based on the child nodes of the given
 * hub_node.
 *
 * @parent The parent device for which all memory will be allocated
 * @hub_node The devicetree node for which the children will be scanned for the
 *           actual PHYs
 * @con_id The phy-name based on which the actual PHYs will be looked up
 */
struct phy *phy_hub_create_from_child_nodes(struct device *parent,
					    struct device_node *hub_node,
					    const char *con_id)
{
	struct device_node *port_np;
	struct phy_hub_priv *priv;
	struct phy *phy;
	int ret, i = 0;

	if (!hub_node) {
		dev_warn(parent, "cannot create PHY hub without node\n");
		return ERR_PTR(-ENODEV);
	}

	priv = devm_kzalloc(parent, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->num_phy_ports = of_get_child_count(hub_node);
	if (!priv->num_phy_ports) {
		dev_err(parent,
			"cannot create PHY hub for node without children\n");
		return ERR_PTR(-ENODEV);
	}

	priv->phy_ports = devm_kcalloc(parent, priv->num_phy_ports,
				       sizeof(*priv->phy_ports), GFP_KERNEL);
	if (!priv->phy_ports)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(hub_node, port_np) {
		priv->phy_ports[i] = devm_of_phy_get(parent, port_np, con_id);
		if (IS_ERR(priv->phy_ports[i])) {
			ret = PTR_ERR(priv->phy_ports[i]);
			if (ret == -ENOSYS || ret == -ENODEV)
				priv->phy_ports[i] = NULL;
			else if (ret == -EPROBE_DEFER)
				return ERR_PTR(ret);
		}

		i++;
	}

	phy = devm_phy_create(parent, NULL, &phy_usb_hub_ops);
	if (IS_ERR(phy)) {
		dev_err(parent, "failed to create PHY hub\n");
		return phy;
	}

	phy_set_drvdata(phy, priv);

	return phy;
}
EXPORT_SYMBOL(phy_hub_create_from_child_nodes);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Generic PHY hub driver");
MODULE_LICENSE("GPL");
