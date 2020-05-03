// SPDX-License-Identifier: GPL-2.0-only
/*
 * Meson8, Meson8b and GXBB USB B connector driver
 *
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#define REG_ADP_BC					0x0c
	#define REG_ADP_BC_ID_DIG			BIT(12)

struct meson8b_usb_b_connector_priv {
	struct device			*dev;
	struct regmap			*regmap;
	struct phy			*phy;
	struct delayed_work		otg_det;
	enum usb_role			last_role;
	struct usb_role_switch		*role_switch;
	struct regulator		*vbus;
};

static void
meson8b_usb_b_connector_set_role(struct meson8b_usb_b_connector_priv *priv,
				 enum usb_role new_role)
{
	int ret;

	if (new_role == priv->last_role)
		return;

	dev_err(priv->dev, "Changing role from %u to %u\n", priv->last_role,
		new_role);

	if (priv->last_role == USB_ROLE_HOST)
		regulator_disable(priv->vbus);

	ret = usb_role_switch_set_role(priv->role_switch, new_role);
	if (ret)
		dev_err(priv->dev, "Failed to set role %u\n", new_role);

	if (new_role == USB_ROLE_HOST) {
		ret = regulator_enable(priv->vbus);
		if (ret)
			dev_err(priv->dev,
				"Failed to enable VBUS regulator\n");
	}

	priv->last_role = new_role;
}

static void meson8b_usb_b_connector_otg_detect(struct work_struct *work)
{
	struct meson8b_usb_b_connector_priv *priv;
	unsigned int val;

	priv = container_of(to_delayed_work(work),
			    struct meson8b_usb_b_connector_priv, otg_det);

	regmap_read(priv->regmap, REG_ADP_BC, &val);
	if (val & REG_ADP_BC_ID_DIG)
		meson8b_usb_b_connector_set_role(priv, USB_ROLE_DEVICE);
	else
		meson8b_usb_b_connector_set_role(priv, USB_ROLE_HOST);

	queue_delayed_work(system_power_efficient_wq, &priv->otg_det,
			   msecs_to_jiffies(500));
}

static int meson8b_usb_b_connector_probe(struct platform_device *pdev)
{
	struct meson8b_usb_b_connector_priv *priv;
	struct phy *phy;

	phy = devm_of_phy_get_by_index(&pdev->dev, pdev->dev.of_node->parent,
				       0);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = dev_get_regmap(phy->dev.parent, NULL);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);
	else if (!priv->regmap)
		return -ENODEV;

	priv->vbus = devm_regulator_get(&pdev->dev, "vbus");
	if (IS_ERR(priv->vbus))
		return PTR_ERR(priv->vbus);

	priv->role_switch = usb_role_switch_get(&pdev->dev);
	if (IS_ERR(priv->role_switch))
		return PTR_ERR(priv->role_switch);
	else if (!priv->role_switch)
		dev_info(&pdev->dev, "Operating without USB role switch\n");

	priv->dev = &pdev->dev;
	priv->last_role = USB_ROLE_NONE;
	INIT_DELAYED_WORK(&priv->otg_det, meson8b_usb_b_connector_otg_detect);

	platform_set_drvdata(pdev, priv);

	queue_delayed_work(system_power_efficient_wq, &priv->otg_det, 0);

	return 0;
}

static int meson8b_usb_b_connector_remove(struct platform_device *pdev)
{
	struct meson8b_usb_b_connector_priv *priv = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&priv->otg_det);

	if (priv->last_role == USB_ROLE_HOST)
		regulator_disable(priv->vbus);

	usb_role_switch_put(priv->role_switch);

	return 0;
}

static const struct of_device_id meson8b_usb_b_connector_of_match[] = {
	{ .compatible = "amlogic,meson8b-usb-b-connector", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_conn_dt_match);

static struct platform_driver meson8b_usb_b_connector_driver = {
	.probe	= meson8b_usb_b_connector_probe,
	.remove	= meson8b_usb_b_connector_remove,
	.driver	= {
		.name = "meson8b-usb-b-connector",
		.of_match_table = meson8b_usb_b_connector_of_match,
	},
};
module_platform_driver(meson8b_usb_b_connector_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Meson8, Meson8b, Meson8m2 and GXBB USB B connector driver");
MODULE_LICENSE("GPL");
