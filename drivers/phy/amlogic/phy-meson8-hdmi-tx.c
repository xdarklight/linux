// SPDX-License-Identifier: GPL-2.0+
/*
 * Meson8, Meson8b and Meson8m2 HDMI TX PHY.
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define HHI_HDMI_PHY_CNTL0				0x3a0

#define HHI_HDMI_PHY_CNTL1				0x3a4
	#define HHI_HDMI_PHY_CNTL1_CLOCK_ENABLE		BIT(1)
	#define HHI_HDMI_PHY_CNTL1_SOFT_RESET		BIT(0)

#define HHI_HDMI_PHY_CNTL2				0x3a8

struct phy_meson8_hdmi_tx_priv {
	struct regmap		*hhi;
};

static int phy_meson8_hdmi_tx_reset(struct phy *phy)
{
	struct phy_meson8_hdmi_tx_priv *priv = phy_get_drvdata(phy);


	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1,
		     HHI_HDMI_PHY_CNTL1_CLOCK_ENABLE |
		     HHI_HDMI_PHY_CNTL1_SOFT_RESET);
	usleep_range(1000, 2000);

	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1,
		     HHI_HDMI_PHY_CNTL1_CLOCK_ENABLE);
	usleep_range(1000, 2000);

	return 0;
}

static int phy_meson8_hdmi_tx_power_on(struct phy *phy)
{
	struct phy_meson8_hdmi_tx_priv *priv = phy_get_drvdata(phy);

	// TODO: this value is for < 4k resolutions only
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x08c31e8b);

	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1, 0x00000000);

	/* FIXME: why 3 times? */
	phy_meson8_hdmi_tx_reset(phy);
	phy_meson8_hdmi_tx_reset(phy);
	phy_meson8_hdmi_tx_reset(phy);

	return 0;
}

static int phy_meson8_hdmi_tx_power_off(struct phy *phy)
{
	struct phy_meson8_hdmi_tx_priv *priv = phy_get_drvdata(phy);

	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, 0x08418d00);

	return 0;
}

static const struct phy_ops phy_meson8_hdmi_tx_ops = {
	.reset		= phy_meson8_hdmi_tx_reset,
	.power_on	= phy_meson8_hdmi_tx_power_on,
	.power_off	= phy_meson8_hdmi_tx_power_off,
	.owner		= THIS_MODULE,
};

static int phy_meson8_hdmi_tx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct phy_meson8_hdmi_tx_priv *priv;
	struct phy_provider *phy_provider;
	struct phy *phy;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->hhi = syscon_node_to_regmap(np->parent);
	if (IS_ERR(priv->hhi))
		return PTR_ERR(priv->hhi);

	phy = devm_phy_create(&pdev->dev, np, &phy_meson8_hdmi_tx_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id phy_meson8_hdmi_tx_of_match[] = {
	{ .compatible = "amlogic,meson8-hdmi-tx-phy" },
	{ .compatible = "amlogic,meson8b-hdmi-tx-phy" },
	{ .compatible = "amlogic,meson8m2-hdmi-tx-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_meson8_hdmi_tx_of_match);

static struct platform_driver phy_meson8_hdmi_tx_driver = {
	.probe	= phy_meson8_hdmi_tx_probe,
	.driver	= {
		.name		= "phy-meson8-hdmi-tx",
		.of_match_table	= phy_meson8_hdmi_tx_of_match,
	},
};
module_platform_driver(phy_meson8_hdmi_tx_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Meson8, Meson8b and Meson8m2 HDMI TX PHY driver");
MODULE_LICENSE("GPL v2");
