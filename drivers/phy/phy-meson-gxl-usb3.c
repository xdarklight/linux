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
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/of.h>
#include <linux/workqueue.h>

#define USB_R0							0x00
	#define USB_R0_P30_FSEL_SHIFT				0
	#define USB_R0_P30_FSEL_MASK				GENMASK(0, 5)
	#define USB_R0_P30_PHY_RESET				BIT(6)
	#define USB_R0_P30_TEST_POWERDOWN_HSP			BIT(7)
	#define USB_R0_P30_TEST_POWERDOWN_SSP			BIT(8)
	#define USB_R0_P30_ACJT_LEVEL_SHIFT			9
	#define USB_R0_P30_ACJT_LEVEL_MASK			GENMASK(9, 13)
	#define USB_R0_P30_TX_BOOST_LEVEL_SHIFT			14
	#define USB_R0_P30_TX_BOOST_LEVEL_MASK			GENMASK(14, 16)
	#define USB_R0_P30_LANE0_TX2RX_LOOPBACK			BIT(17)
	#define USB_R0_P30_LANE0_EXT_PCLK_REQ			BIT(18)
	#define USB_R0_P30_PCS_RX_LOS_MASK_VAL_SHIFT		19
	#define USB_R0_P30_PCS_RX_LOS_MASK_VAL_MASK		GENMASK(19, 28)
	#define USB_R0_U2D_SS_SCALEDOWN_MODE_SHIFT		29
	#define USB_R0_U2D_SS_SCALEDOWN_MODE_MASK		GENMASK(29, 30)
	#define USB_R0_U2D_ACT					BIT(31)

#define USB_R1							0x04
	#define USB_R1_U3H_BIGENDIAN_GS				BIT(0)
	#define USB_R1_U3H_PME_ENABLE				BIT(1)
	#define USB_R1_U3H_HUB_PORT_OVERCURRENT_SHIFT		2
	#define USB_R1_U3H_HUB_PORT_OVERCURRENT_MASK		GENMASK(2, 6)
	#define USB_R1_U3H_HUB_PORT_PERM_ATTACH_SHIFT		7
	#define USB_R1_U3H_HUB_PORT_PERM_ATTACH_MASK		GENMASK(7, 11)
	#define USB_R1_U3H_HOST_U2_PORT_DISABLE_SHIFT		12
	#define USB_R1_U3H_HOST_U2_PORT_DISABLE_MASK		GENMASK(12, 15)
	#define USB_R1_U3H_HOST_U3_PORT_DISABLE			BIT(16)
	#define USB_R1_U3H_HOST_PORT_POWER_CONTROL_PRESENT	BIT(17)
	#define USB_R1_U3H_HOST_MSI_ENABLE			BIT(18)
	#define USB_R1_U3H_FLADJ_30MHZ_REG_SHIFT		19
	#define USB_R1_U3H_FLADJ_30MHZ_REG_MASK			GENMASK(19, 24)
	#define USB_R1_P30_PCS_TX_SWING_FULL_SHIFT		25
	#define USB_R1_P30_PCS_TX_SWING_FULL_MASK		GENMASK(25, 31)

#define USB_R2							0x08
	#define USB_R2_P30_CR_DATA_IN_SHIFT			0
	#define USB_R2_P30_CR_DATA_IN_MASK			GENMASK(0, 15)
	#define USB_R2_P30_CR_READ				BIT(16)
	#define USB_R2_P30_CR_WRITE				BIT(17)
	#define USB_R2_P30_CR_CAP_ADDR				BIT(18)
	#define USB_R2_P30_CR_CAP_DATA				BIT(19)
	#define USB_R2_P30_PCS_TX_DEEMPH_3P5DB_SHIFT		20
	#define USB_R2_P30_PCS_TX_DEEMPH_3P5DB_MASK		GENMASK(20, 25)
	#define USB_R2_P30_PCS_TX_DEEMPH_6DB_SHIFT		26
	#define USB_R2_P30_PCS_TX_DEEMPH_6DB_MASK		GENMASK(26, 31)

#define USB_R3							0x0c
	#define USB_R3_P30_SSC_ENABLE				BIT(0)
	#define USB_R3_P30_SSC_RANGE_SHIFT			1
	#define USB_R3_P30_SSC_RANGE_MASK			GENMASK(1, 3)
	#define USB_R3_P30_SSC_REF_CLK_SEL_SHIFT		4
	#define USB_R3_P30_SSC_REF_CLK_SEL_MASK			GENMASK(4, 12)
	#define USB_R3_P30_REF_SSP_EN				BIT(13)
	#define USB_R3_P30_LOS_BIAS_SHIFT			16
	#define USB_R3_P30_LOS_BIAS_MASK			GENMASK(16, 18)
	#define USB_R3_P30_LOS_LEVEL_SHIFT			19
	#define USB_R3_P30_LOS_LEVEL_MASK			GENMASK(19, 23)
	#define USB_R3_P30_MPLL_MULTIPLIER_SHIFT		24
	#define USB_R3_P30_MPLL_MULTIPLIER_MASK			GENMASK(24, 30)

#define USB_R4							0x10
	#define USB_R4_P21_PORT_RESET_0				BIT(0)
	#define USB_R4_P21_SLEEP_M0				BIT(1)
	#define USB_R4_MEM_PD_SHIFT				2
	#define USB_R4_MEM_PD_MASK				GENMASK(2, 3)
	#define USB_R4_P21_ONLY					BIT(4)

#define USB_R5							0x14
	#define USB_R5_ID_DIG_SYNC				BIT(0)
	#define USB_R5_ID_DIG_REG				BIT(1)
	#define USB_R5_ID_DIG_CFG_SHIFT				2
	#define USB_R5_ID_DIG_CFG_MASK				GENMASK(2, 3)
	#define USB_R5_ID_DIG_EN_0				BIT(4)
	#define USB_R5_ID_DIG_EN_1				BIT(5)
	#define USB_R5_ID_DIG_CURR				BIT(6)
	#define USB_R5_ID_DIG_IRQ				BIT(7)
	#define USB_R5_ID_DIG_TH_SHIFT				8
	#define USB_R5_ID_DIG_TH_MASK				GENMASK(8, 15)
	#define USB_R5_ID_DIG_CNT_SHIFT				16
	#define USB_R5_ID_DIG_CNT_MASK				GENMASK(16, 23)

#define USB_R6							0x18
	#define USB_R6_P30_CR_DATA_OUT_SHIFT			0
	#define USB_R6_P30_CR_DATA_OUT_MASK			GENMASK(0, 15)
	#define USB_R6_P30_CR_ACK				BIT(16)

#define RESET_COMPLETE_TIME				500

struct phy_meson_gxl_usb3_priv {
	void __iomem		*regs;
	struct delayed_work	otg_work;
	struct phy		*this_phy;
	struct phy		*usb2_phy;
};

static u32 phy_meson_gxl_usb3_read(struct phy_meson_gxl_usb3_priv *priv, u32 reg)
{
	return readl(priv->regs + reg);
}

static void phy_meson_gxl_usb3_mask_bits(struct phy_meson_gxl_usb3_priv *priv,
					 u32 reg, u32 mask, u32 value)
{
	u32 data;

	data = phy_meson_gxl_usb3_read(priv, reg);
	data &= ~mask;
	data |= (value & mask);

	writel(data, priv->regs + reg);
}

static int phy_meson_gxl_usb3_update_mode(struct phy_meson_gxl_usb3_priv *priv)
{
	enum phy_mode mode;
	int ret;

	if (phy_meson_gxl_usb3_read(priv, USB_R5) & USB_R5_ID_DIG_CURR) {
		mode = PHY_MODE_USB_DEVICE;

		phy_meson_gxl_usb3_mask_bits(priv, USB_R0, USB_R0_U2D_ACT,
				USB_R0_U2D_ACT);
		phy_meson_gxl_usb3_mask_bits(priv, USB_R4, USB_R4_P21_SLEEP_M0,
					     USB_R4_P21_SLEEP_M0);
	} else {
		mode = PHY_MODE_USB_HOST;

		phy_meson_gxl_usb3_mask_bits(priv, USB_R0, USB_R0_U2D_ACT, 0);
		phy_meson_gxl_usb3_mask_bits(priv, USB_R4, USB_R4_P21_SLEEP_M0,
					     0);
	}

	ret = phy_set_mode(priv->usb2_phy, mode);
	if (ret) {
		dev_err(&priv->this_phy->dev,
			"Failed to update USB2 mode to %d: %d\n", mode, ret);
		return ret;
	}

	{
		int i;
		for (i = 0; i < 0x20; i+=0x4)
			printk("0x%lx = 0x%08x\n", priv->regs + i, phy_meson_gxl_usb3_read(priv, i));
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
	phy_meson_gxl_usb3_mask_bits(priv, USB_R5, USB_R5_ID_DIG_IRQ, 0);
}

static int phy_meson_gxl_usb3_power_on(struct phy *phy)
{
	struct phy_meson_gxl_usb3_priv *priv = phy_get_drvdata(phy);

	phy_meson_gxl_usb3_mask_bits(priv, USB_R1,
				     USB_R1_U3H_FLADJ_30MHZ_REG_MASK,
				     0x20 << USB_R1_U3H_FLADJ_30MHZ_REG_SHIFT);

	phy_meson_gxl_usb3_mask_bits(priv, USB_R5, USB_R5_ID_DIG_EN_0,
				     USB_R5_ID_DIG_EN_0);
	phy_meson_gxl_usb3_mask_bits(priv, USB_R5, USB_R5_ID_DIG_EN_1,
				     USB_R5_ID_DIG_EN_1);
	phy_meson_gxl_usb3_mask_bits(priv, USB_R5, USB_R5_ID_DIG_TH_MASK,
				     0xff << USB_R5_ID_DIG_TH_SHIFT);

	return phy_meson_gxl_usb3_update_mode(priv);
}

static const struct phy_ops phy_meson_gxl_usb3_ops = {
	.power_on	= phy_meson_gxl_usb3_power_on,
	.owner		= THIS_MODULE,
};

static irqreturn_t phy_meson_gxl_usb3_irq(int irq, void *data)
{
	struct phy_meson_gxl_usb3_priv *priv = data;

	if (!(phy_meson_gxl_usb3_read(priv, USB_R5) & USB_R5_ID_DIG_IRQ)) {
		dev_err(&priv->this_phy->dev, "spurious interrupt\n");
		return IRQ_NONE;
	}

	schedule_delayed_work(&priv->otg_work, msecs_to_jiffies(10));

	/* acknowledge the IRQ */
	phy_meson_gxl_usb3_mask_bits(priv, USB_R5, USB_R5_ID_DIG_IRQ, 0);

	return IRQ_HANDLED;
}

static int phy_meson_gxl_usb3_probe(struct platform_device *pdev)
{
	struct phy_meson_gxl_usb3_priv *priv;
	struct resource *res;
	struct phy_provider *phy_provider;
	int irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		INIT_DELAYED_WORK(&priv->otg_work, phy_meson_gxl_usb3_work);

		irq = devm_request_irq(&pdev->dev, irq, phy_meson_gxl_usb3_irq,
				       IRQF_SHARED, dev_name(&pdev->dev),
				       priv);
		if (irq < 0) {
			dev_err(&pdev->dev,
				"Could not register IRQ handler (%d)\n", irq);
			return -EINVAL;
		}

		priv->usb2_phy = devm_phy_get(&pdev->dev, "usb2-phy");
		if (IS_ERR(priv->usb2_phy))
			return PTR_ERR(priv->usb2_phy);
	}

	priv->this_phy = devm_phy_create(&pdev->dev, NULL,
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
