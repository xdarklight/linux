/*
 * Meson GXL USB3 PHY driver
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

#define USB_R0							0x00
	#define USB_R0_P30_FSEL_SHIFT				0
	#define USB_R0_P30_FSEL_MASK				GENMASK(5, 0)
	#define USB_R0_P30_PHY_RESET				BIT(6)
	#define USB_R0_P30_TEST_POWERDOWN_HSP			BIT(7)
	#define USB_R0_P30_TEST_POWERDOWN_SSP			BIT(8)
	#define USB_R0_P30_ACJT_LEVEL_SHIFT			9
	#define USB_R0_P30_ACJT_LEVEL_MASK			GENMASK(13, 9)
	#define USB_R0_P30_TX_BOOST_LEVEL_SHIFT			14
	#define USB_R0_P30_TX_BOOST_LEVEL_MASK			GENMASK(16, 14)
	#define USB_R0_P30_LANE0_TX2RX_LOOPBACK			BIT(17)
	#define USB_R0_P30_LANE0_EXT_PCLK_REQ			BIT(18)
	#define USB_R0_P30_PCS_RX_LOS_MASK_VAL_SHIFT		19
	#define USB_R0_P30_PCS_RX_LOS_MASK_VAL_MASK		GENMASK(28, 19)
	#define USB_R0_U2D_SS_SCALEDOWN_MODE_SHIFT		29
	#define USB_R0_U2D_SS_SCALEDOWN_MODE_MASK		GENMASK(30, 29)
	#define USB_R0_U2D_ACT					BIT(31)

#define USB_R1							0x04
	#define USB_R1_U3H_BIGENDIAN_GS				BIT(0)
	#define USB_R1_U3H_PME_ENABLE				BIT(1)
	#define USB_R1_U3H_HUB_PORT_OVERCURRENT_SHIFT		2
	#define USB_R1_U3H_HUB_PORT_OVERCURRENT_MASK		GENMASK(6, 2)
	#define USB_R1_U3H_HUB_PORT_PERM_ATTACH_SHIFT		7
	#define USB_R1_U3H_HUB_PORT_PERM_ATTACH_MASK		GENMASK(11, 7)
	#define USB_R1_U3H_HOST_U2_PORT_DISABLE_SHIFT		12
	#define USB_R1_U3H_HOST_U2_PORT_DISABLE_MASK		GENMASK(15, 12)
	#define USB_R1_U3H_HOST_U3_PORT_DISABLE			BIT(16)
	#define USB_R1_U3H_HOST_PORT_POWER_CONTROL_PRESENT	BIT(17)
	#define USB_R1_U3H_HOST_MSI_ENABLE			BIT(18)
	#define USB_R1_U3H_FLADJ_30MHZ_REG_SHIFT		19
	#define USB_R1_U3H_FLADJ_30MHZ_REG_MASK			GENMASK(24, 19)
	#define USB_R1_P30_PCS_TX_SWING_FULL_SHIFT		25
	#define USB_R1_P30_PCS_TX_SWING_FULL_MASK		GENMASK(31, 25)

#define USB_R2							0x08
	#define USB_R2_P30_CR_DATA_IN_SHIFT			0
	#define USB_R2_P30_CR_DATA_IN_MASK			GENMASK(15, 0)
	#define USB_R2_P30_CR_READ				BIT(16)
	#define USB_R2_P30_CR_WRITE				BIT(17)
	#define USB_R2_P30_CR_CAP_ADDR				BIT(18)
	#define USB_R2_P30_CR_CAP_DATA				BIT(19)
	#define USB_R2_P30_PCS_TX_DEEMPH_3P5DB_SHIFT		20
	#define USB_R2_P30_PCS_TX_DEEMPH_3P5DB_MASK		GENMASK(25, 20)
	#define USB_R2_P30_PCS_TX_DEEMPH_6DB_SHIFT		26
	#define USB_R2_P30_PCS_TX_DEEMPH_6DB_MASK		GENMASK(31, 26)

#define USB_R3							0x0c
	#define USB_R3_P30_SSC_ENABLE				BIT(0)
	#define USB_R3_P30_SSC_RANGE_SHIFT			1
	#define USB_R3_P30_SSC_RANGE_MASK			GENMASK(3, 1)
	#define USB_R3_P30_SSC_REF_CLK_SEL_SHIFT		4
	#define USB_R3_P30_SSC_REF_CLK_SEL_MASK			GENMASK(12, 4)
	#define USB_R3_P30_REF_SSP_EN				BIT(13)
	#define USB_R3_P30_LOS_BIAS_SHIFT			16
	#define USB_R3_P30_LOS_BIAS_MASK			GENMASK(18, 16)
	#define USB_R3_P30_LOS_LEVEL_SHIFT			19
	#define USB_R3_P30_LOS_LEVEL_MASK			GENMASK(23, 19)
	#define USB_R3_P30_MPLL_MULTIPLIER_SHIFT		24
	#define USB_R3_P30_MPLL_MULTIPLIER_MASK			GENMASK(30, 24)

#define USB_R4							0x10
	#define USB_R4_P21_PORT_RESET_0				BIT(0)
	#define USB_R4_P21_SLEEP_M0				BIT(1)
	#define USB_R4_MEM_PD_SHIFT				2
	#define USB_R4_MEM_PD_MASK				GENMASK(3, 2)
	#define USB_R4_P21_ONLY					BIT(4)

#define USB_R5							0x14
	#define USB_R5_ID_DIG_SYNC				BIT(0)
	#define USB_R5_ID_DIG_REG				BIT(1)
	#define USB_R5_ID_DIG_CFG_SHIFT				2
	#define USB_R5_ID_DIG_CFG_MASK				GENMASK(3, 2)
	#define USB_R5_ID_DIG_EN_0				BIT(4)
	#define USB_R5_ID_DIG_EN_1				BIT(5)
	#define USB_R5_ID_DIG_CURR				BIT(6)
	#define USB_R5_ID_DIG_IRQ				BIT(7)
	#define USB_R5_ID_DIG_TH_SHIFT				8
	#define USB_R5_ID_DIG_TH_MASK				GENMASK(15, 8)
	#define USB_R5_ID_DIG_CNT_SHIFT				16
	#define USB_R5_ID_DIG_CNT_MASK				GENMASK(23, 16)

/* read-only register */
#define USB_R6							0x18
	#define USB_R6_P30_CR_DATA_OUT_SHIFT			0
	#define USB_R6_P30_CR_DATA_OUT_MASK			GENMASK(15, 0)
	#define USB_R6_P30_CR_ACK				BIT(16)

#define RESET_COMPLETE_TIME				500

struct phy_meson_gxl_usb3_priv {
	struct regmap		*regmap;
	struct delayed_work	otg_work;
	struct phy		*this_phy;
	struct phy		*usb2_phy;
};

static const struct regmap_config phy_meson_gxl_usb3_regmap_conf = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = USB_R6,
};

static int phy_meson_gxl_usb3_update_mode(struct phy_meson_gxl_usb3_priv *priv)
{
	u32 val;
	enum phy_mode mode;
	int ret;

	ret = regmap_read(priv->regmap, USB_R5, &val);
	if (ret)
		return ret;

	if (val & USB_R5_ID_DIG_CURR) {
		mode = PHY_MODE_USB_DEVICE;

		regmap_update_bits(priv->regmap, USB_R0, USB_R0_U2D_ACT,
				   USB_R0_U2D_ACT);
		regmap_update_bits(priv->regmap, USB_R4, USB_R4_P21_SLEEP_M0,
				   USB_R4_P21_SLEEP_M0);
	} else {
		mode = PHY_MODE_USB_HOST;

		regmap_update_bits(priv->regmap, USB_R0, USB_R0_U2D_ACT, 0);
		regmap_update_bits(priv->regmap, USB_R4, USB_R4_P21_SLEEP_M0,
				   0);
	}

	/* inform the USB2 PHY that we have changed the mode */
	ret = phy_set_mode(priv->usb2_phy, mode);
	if (ret) {
		dev_err(&priv->this_phy->dev,
			"Failed to update usb2-phy mode to %d\n", mode);
		return ret;
	}

	return ret;
}

static void phy_meson_gxl_usb3_work(struct work_struct *data)
{
	struct phy_meson_gxl_usb3_priv *priv =
		container_of(data, struct phy_meson_gxl_usb3_priv,
			     otg_work.work);

	phy_meson_gxl_usb3_update_mode(priv);

	/* unmask IRQs which may have arrived in the meantime */
	regmap_update_bits(priv->regmap, USB_R5, USB_R5_ID_DIG_IRQ, 0);
}

static int phy_meson_gxl_usb3_power_on(struct phy *phy)
{
	struct phy_meson_gxl_usb3_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_init(priv->usb2_phy);
	if (ret) {
		dev_err(&phy->dev, "Failed to initialize usb2-phy\n");
		return ret;
	}

	ret = phy_power_on(priv->usb2_phy);
	if (ret) {
		dev_err(&phy->dev, "Failed to power-on usb2-phy\n");
		return ret;
	}

	regmap_update_bits(priv->regmap, USB_R1,
			   USB_R1_U3H_FLADJ_30MHZ_REG_MASK,
			   0x20 << USB_R1_U3H_FLADJ_30MHZ_REG_SHIFT);

	regmap_update_bits(priv->regmap, USB_R5, USB_R5_ID_DIG_EN_0,
			   USB_R5_ID_DIG_EN_0);
	regmap_update_bits(priv->regmap, USB_R5, USB_R5_ID_DIG_EN_1,
			   USB_R5_ID_DIG_EN_1);
	regmap_update_bits(priv->regmap, USB_R5, USB_R5_ID_DIG_TH_MASK,
			   0xff << USB_R5_ID_DIG_TH_SHIFT);

	return phy_meson_gxl_usb3_update_mode(priv);
}

static int phy_meson_gxl_usb3_power_off(struct phy *phy)
{
	struct phy_meson_gxl_usb3_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_power_off(priv->usb2_phy);
	if (ret) {
		dev_err(&phy->dev, "Failed to power-off usb2-phy\n");
		return ret;
	}

	ret = phy_exit(priv->usb2_phy);
	if (ret) {
		dev_err(&phy->dev, "Failed to exit usb2-phy\n");
		return ret;
	}

	return 0;
}

static irqreturn_t phy_meson_gxl_usb3_irq(int irq, void *data)
{
	u32 val;
	struct phy_meson_gxl_usb3_priv *priv = data;

	regmap_read(priv->regmap, USB_R5, &val);
	if (!(val & USB_R5_ID_DIG_IRQ)) {
		dev_err(&priv->this_phy->dev, "spurious interrupt\n");
		return IRQ_NONE;
	}

	schedule_delayed_work(&priv->otg_work, msecs_to_jiffies(10));

	/* acknowledge the IRQ */
	regmap_update_bits(priv->regmap, USB_R5, USB_R5_ID_DIG_IRQ, 0);

	return IRQ_HANDLED;
}

static const struct phy_ops phy_meson_gxl_usb3_ops = {
	.power_on	= phy_meson_gxl_usb3_power_on,
	.power_off	= phy_meson_gxl_usb3_power_off,
	.owner		= THIS_MODULE,
};

static int phy_meson_gxl_usb3_probe(struct platform_device *pdev)
{
	struct phy_meson_gxl_usb3_priv *priv;
	struct resource *res;
	struct phy_provider *phy_provider;
	void __iomem *base;
	int irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &phy_meson_gxl_usb3_regmap_conf);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		INIT_DELAYED_WORK(&priv->otg_work, phy_meson_gxl_usb3_work);

		irq = devm_request_irq(&pdev->dev, irq, phy_meson_gxl_usb3_irq,
				       IRQF_SHARED, dev_name(&pdev->dev),
				       priv);
		if (irq < 0) {
			dev_err(&pdev->dev,
				"could not register IRQ handler (%d)\n", irq);
			return -EINVAL;
		}
	}

	priv->usb2_phy = devm_phy_get(&pdev->dev, "usb2-phy");
	if (IS_ERR(priv->usb2_phy)) {
		dev_err(&pdev->dev, "failed to get companion usb2-phy");
		return PTR_ERR(priv->usb2_phy);
	}

	priv->this_phy = devm_phy_create(&pdev->dev, pdev->dev.of_node,
					 &phy_meson_gxl_usb3_ops);
	if (IS_ERR(priv->this_phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(priv->this_phy);
	}

	phy_set_drvdata(priv->this_phy, priv);

	phy_provider =
		devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id phy_meson_gxl_usb3_of_match[] = {
	{ .compatible = "amlogic,meson-gxl-usb3-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_meson_gxl_usb3_of_match);

static struct platform_driver phy_meson_gxl_usb3_driver = {
	.probe	= phy_meson_gxl_usb3_probe,
	.driver	= {
		.name		= "phy-meson-gxl-usb3",
		.of_match_table	= phy_meson_gxl_usb3_of_match,
	},
};
module_platform_driver(phy_meson_gxl_usb3_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Meson GXL USB3 PHY driver");
MODULE_LICENSE("GPL");
