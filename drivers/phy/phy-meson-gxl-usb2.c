/*
 * Meson GXL USB2 PHY driver
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
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/of.h>

/* bits [31:27] are read-only */
#define U2P_R0							0x0
	#define U2P_R0_BYPASS_SEL				BIT(0)
	#define U2P_R0_BYPASS_DM_EN				BIT(1)
	#define U2P_R0_BYPASS_DP_EN				BIT(2)
	#define U2P_R0_TXBITSTUFF_ENH				BIT(3)
	#define U2P_R0_TXBITSTUFF_EN				BIT(4)
	#define U2P_R0_DM_PULLDOWN				BIT(5)
	#define U2P_R0_DP_PULLDOWN				BIT(6)
	#define U2P_R0_DP_VBUS_VLD_EXT_SEL			BIT(7)
	#define U2P_R0_DP_VBUS_VLD_EXT				BIT(8)
	#define U2P_R0_ADP_PRB_EN				BIT(9)
	#define U2P_R0_ADP_DISCHARGE				BIT(10)
	#define U2P_R0_ADP_CHARGE				BIT(11)
	#define U2P_R0_DRV_VBUS					BIT(12)
	#define U2P_R0_ID_PULLUP				BIT(13)
	#define U2P_R0_LOOPBACK_EN_B				BIT(14)
	#define U2P_R0_OTG_DISABLE				BIT(15)
	#define U2P_R0_COMMON_ONN				BIT(16)
	#define U2P_R0_FSEL_SHIFT				17
	#define U2P_R0_FSEL_MASK				GENMASK(19, 17)
	#define U2P_R0_REF_CLK_SEL_SHIFT			20
	#define U2P_R0_REF_CLK_SEL_MASK				GENMASK(21, 20)
	#define U2P_R0_POWER_ON_RESET				BIT(22)
	#define U2P_R0_V_ATE_TEST_EN_B_SHIFT			23
	#define U2P_R0_V_ATE_TEST_EN_B_MASK			GENMASK(24, 23)
	#define U2P_R0_ID_SET_ID_DQ				BIT(25)
	#define U2P_R0_ATE_RESET				BIT(26)
	#define U2P_R0_FSV_MINUS				BIT(27)
	#define U2P_R0_FSV_PLUS					BIT(28)
	#define U2P_R0_BYPASS_DM_DATA				BIT(29)
	#define U2P_R0_BYPASS_DP_DATA				BIT(30)

#define U2P_R1							0x4
	#define U2P_R1_BURN_IN_TEST				BIT(0)
	#define U2P_R1_ACA_ENABLE				BIT(1)
	#define U2P_R1_DCD_ENABLE				BIT(2)
	#define U2P_R1_VDAT_SRC_EN_B				BIT(3)
	#define U2P_R1_VDAT_DET_EN_B				BIT(4)
	#define U2P_R1_CHARGES_SEL				BIT(5)
	#define U2P_R1_TX_PREEMP_PULSE_TUNE			BIT(6)
	#define U2P_R1_TX_PREEMP_AMP_TUNE_SHIFT			7
	#define U2P_R1_TX_PREEMP_AMP_TUNE_MASK			GENMASK(8, 7)
	#define U2P_R1_TX_RES_TUNE_SHIFT			9
	#define U2P_R1_TX_RES_TUNE_MASK				GENMASK(10, 9)
	#define U2P_R1_TX_RISE_TUNE_SHIFT			11
	#define U2P_R1_TX_RISE_TUNE_MASK			GENMASK(12, 11)
	#define U2P_R1_TX_VREF_TUNE_SHIFT			13
	#define U2P_R1_TX_VREF_TUNE_MASK			GENMASK(16, 13)
	#define U2P_R1_TX_FSLS_TUNE_SHIFT			17
	#define U2P_R1_TX_FSLS_TUNE_MASK			GENMASK(20, 17)
	#define U2P_R1_TX_HSXV_TUNE_SHIFT			21
	#define U2P_R1_TX_HSXV_TUNE_MASK			GENMASK(22, 21)
	#define U2P_R1_OTG_TUNE_SHIFT				23
	#define U2P_R1_OTG_TUNE_MASK				GENMASK(25, 23)
	#define U2P_R1_SQRX_TUNE_SHIFT				26
	#define U2P_R1_SQRX_TUNE_MASK				GENMASK(28, 26)
	#define U2P_R1_COMP_DIS_TUNE_SHIFT			29
	#define U2P_R1_COMP_DIS_TUNE_MASK			GENMASK(31, 29)

/* bits [31:14] are read-only */
#define U2P_R2							0x8
	#define U2P_R2_DATA_IN_SHIFT				0
	#define U2P_R2_DATA_IN_MASK				GENMASK(3, 0)
	#define U2P_R2_DATA_IN_EN_SHIFT				4
	#define U2P_R2_DATA_IN_EN_MASK				GENMASK(7, 4)
	#define U2P_R2_ADDR_SHIFT				8
	#define U2P_R2_ADDR_MASK				GENMASK(11, 8)
	#define U2P_R2_DATA_OUT_SEL				BIT(12)
	#define U2P_R2_CLK					BIT(13)
	#define U2P_R2_DATA_OUT_SHIFT				14
	#define U2P_R2_DATA_OUT_MASK				GENMASK(17, 14)
	#define U2P_R2_ACA_PIN_RANGE_C				BIT(18)
	#define U2P_R2_ACA_PIN_RANGE_B				BIT(19)
	#define U2P_R2_ACA_PIN_RANGE_A				BIT(20)
	#define U2P_R2_ACA_PIN_GND				BIT(21)
	#define U2P_R2_ACA_PIN_FLOAT				BIT(22)
	#define U2P_R2_CHARGE_DETECT				BIT(23)
	#define U2P_R2_DEVICE_SESSION_VALID			BIT(24)
	#define U2P_R2_ADP_PROBE				BIT(25)
	#define U2P_R2_ADP_SENSE				BIT(26)
	#define U2P_R2_SESSION_END				BIT(27)
	#define U2P_R2_VBUS_VALID				BIT(28)
	#define U2P_R2_B_VALID					BIT(29)
	#define U2P_R2_A_VALID					BIT(30)
	#define U2P_R2_ID_DIG					BIT(31)

#define U2P_R3							0xc

#define RESET_COMPLETE_TIME				500

struct phy_meson_gxl_usb2_priv {
	struct regmap		*regmap;
	enum phy_mode		mode;
	struct clk		*clk_usb_general;
	struct clk		*clk_usb;
	struct reset_control	*reset;
};

static const struct regmap_config phy_meson_gxl_usb2_regmap_conf = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = U2P_R3,
};

static int phy_meson_gxl_usb2_set_mode(struct phy *phy, enum phy_mode mode)
{
	struct phy_meson_gxl_usb2_priv *priv = phy_get_drvdata(phy);

	/* take PHY out of reset mode to power it up */
	regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_POWER_ON_RESET, 0);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_OTG:
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_DM_PULLDOWN,
				   U2P_R0_DM_PULLDOWN);
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_DP_PULLDOWN,
				   U2P_R0_DP_PULLDOWN);
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_ID_PULLUP, 0);
		break;

	case PHY_MODE_USB_DEVICE:
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_DM_PULLDOWN,
				   0);
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_DP_PULLDOWN,
				   0);
		regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_ID_PULLUP,
				   U2P_R0_ID_PULLUP);
		break;

	default:
		return -EINVAL;
	}

	/* reset the PHY and wait until settings are stabilized */
	regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_POWER_ON_RESET,
			   U2P_R0_POWER_ON_RESET);
	udelay(RESET_COMPLETE_TIME);
	regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_POWER_ON_RESET, 0);
	udelay(RESET_COMPLETE_TIME);

	priv->mode = mode;

	return 0;
}

static int phy_meson_gxl_usb2_power_on(struct phy *phy)
{
	struct phy_meson_gxl_usb2_priv *priv = phy_get_drvdata(phy);
	int ret;

	if (!IS_ERR_OR_NULL(priv->reset)) {
		ret = reset_control_reset(priv->reset);
		if (ret) {
			dev_err(&phy->dev, "Failed to trigger USB reset\n");
			return ret;
		}
	}

	ret = clk_prepare_enable(priv->clk_usb_general);
	if (ret) {
		dev_err(&phy->dev, "Failed to enable USB general clock\n");
		return ret;
	}

	if (priv->clk_usb) {
		ret = clk_prepare_enable(priv->clk_usb);
		if (ret) {
			dev_err(&phy->dev, "Failed to enable USB DDR clock\n");
			return ret;
		}
	}

	ret = phy_meson_gxl_usb2_set_mode(phy, priv->mode);
	if (ret) {
		dev_err(&phy->dev, "Failed to initialize PHY with mode %d\n",
			priv->mode);
		return ret;
	}

	return 0;
}

static int phy_meson_gxl_usb2_power_off(struct phy *phy)
{
	struct phy_meson_gxl_usb2_priv *priv = phy_get_drvdata(phy);

	/* put PHY in reset mode to power it off */
	regmap_update_bits(priv->regmap, U2P_R0, U2P_R0_POWER_ON_RESET,
			   U2P_R0_POWER_ON_RESET);

	if (priv->clk_usb)
		clk_disable_unprepare(priv->clk_usb);

	clk_disable_unprepare(priv->clk_usb_general);

	return 0;
}

static const struct phy_ops phy_meson_gxl_usb2_ops = {
	.power_on	= phy_meson_gxl_usb2_power_on,
	.power_off	= phy_meson_gxl_usb2_power_off,
	.set_mode	= phy_meson_gxl_usb2_set_mode,
	.owner		= THIS_MODULE,
};

static int phy_meson_gxl_usb2_probe(struct platform_device *pdev)
{
	struct phy_meson_gxl_usb2_priv *priv;
	struct resource *res;
	struct phy *phy;
	struct phy_provider *phy_provider;
	void __iomem *base;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &phy_meson_gxl_usb2_regmap_conf);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->clk_usb_general = devm_clk_get(&pdev->dev, "usb_general");
	if (IS_ERR(priv->clk_usb_general))
		return PTR_ERR(priv->clk_usb_general);

	priv->clk_usb = devm_clk_get(&pdev->dev, "usb");
	if (PTR_ERR(priv->clk_usb))
		priv->clk_usb = NULL;
	else if (IS_ERR(priv->clk_usb))
		return PTR_ERR(priv->clk_usb);

	priv->reset = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (PTR_ERR(priv->reset) == -EPROBE_DEFER)
		return PTR_ERR(priv->reset);

	switch (of_usb_get_dr_mode_by_phy(pdev->dev.of_node, -1)) {
	case USB_DR_MODE_PERIPHERAL:
		priv->mode = PHY_MODE_USB_DEVICE;
		break;
	case USB_DR_MODE_OTG:
		priv->mode = PHY_MODE_USB_OTG;
		break;
	case USB_DR_MODE_HOST:
	default:
		priv->mode = PHY_MODE_USB_HOST;
		break;
	}

	phy = devm_phy_create(&pdev->dev, NULL, &phy_meson_gxl_usb2_ops);
	if (IS_ERR(phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, priv);

	phy_provider =
		devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id phy_meson_gxl_usb2_of_match[] = {
	{ .compatible = "amlogic,meson-gxl-usb2-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_meson_gxl_usb2_of_match);

static struct platform_driver phy_meson_gxl_usb2_driver = {
	.probe	= phy_meson_gxl_usb2_probe,
	.driver	= {
		.name		= "phy-meson-gxl-usb2",
		.of_match_table	= phy_meson_gxl_usb2_of_match,
	},
};
module_platform_driver(phy_meson_gxl_usb2_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Meson GXL USB2 PHY driver");
MODULE_LICENSE("GPL");
