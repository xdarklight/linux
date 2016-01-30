/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2010 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 *  Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/of_platform.h>

#define LANTIQ_RCU_RESET_TIMEOUT	1000000

struct lantiq_rcu_reset_priv {
	struct reset_controller_dev rcdev;
	struct device *dev;
	struct regmap *regmap;
	u32 reset_offset;
	u32 status_offset;
};

static struct lantiq_rcu_reset_priv *to_lantiq_rcu_reset_priv(
	struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct lantiq_rcu_reset_priv, rcdev);
}

static int lantiq_rcu_reset_status(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct lantiq_rcu_reset_priv *priv = to_lantiq_rcu_reset_priv(rcdev);
	u32 val;
	int ret;

	if (id >= rcdev->nr_resets)
		return -EINVAL;

	ret = regmap_read(priv->regmap, priv->status_offset, &val);
	if (ret)
		return ret;

	return !!(val & BIT(id));
}

static int lantiq_rcu_reset_update(struct reset_controller_dev *rcdev,
				   unsigned long id, bool assert)
{
	struct lantiq_rcu_reset_priv *priv = to_lantiq_rcu_reset_priv(rcdev);
	u32 val;
	int ret, retry = LANTIQ_RCU_RESET_TIMEOUT;

	if (id >= rcdev->nr_resets)
		return -EINVAL;

	if (assert)
		val = BIT(id);
	else
		val = 0;

	ret = regmap_update_bits(priv->regmap, priv->reset_offset, BIT(id),
				 val);
	if (ret) {
		dev_err(priv->dev, "Failed to set reset bit %lu\n", id);
		return ret;
	}

	do {} while (--retry && lantiq_rcu_reset_status(rcdev, id) != assert);
	if (!retry) {
		dev_err(priv->dev, "Failed to %s bit %lu\n",
			assert ? "assert" : "deassert", id);
		return -EIO;
	}

	return 0;
}

static int lantiq_rcu_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return lantiq_rcu_reset_update(rcdev, id, true);
}

static int lantiq_rcu_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return lantiq_rcu_reset_update(rcdev, id, false);
}

static int lantiq_rcu_reset_reset(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	int ret;

	ret = lantiq_rcu_reset_assert(rcdev, id);
	if (ret)
		return ret;

	return lantiq_rcu_reset_deassert(rcdev, id);
}

static struct reset_control_ops lantiq_rcu_reset_ops = {
	.assert = lantiq_rcu_reset_assert,
	.deassert = lantiq_rcu_reset_deassert,
	.status = lantiq_rcu_reset_status,
	.reset	= lantiq_rcu_reset_reset,
};

static int lantiq_rcu_reset_of_probe(struct platform_device *pdev,
			       struct lantiq_rcu_reset_priv *priv)
{
	struct device_node *np = pdev->dev.of_node;

	priv->regmap = syscon_regmap_lookup_by_phandle(np,
							"lantiq,rcu-syscon");
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(priv->regmap);
	}

	if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 1,
		&priv->reset_offset)) {
		dev_err(&pdev->dev, "Failed to get RCU reset offset\n");
		return -EINVAL;
	}

	if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 2,
		&priv->status_offset)) {
		dev_err(&pdev->dev, "Failed to get RCU status offset\n");
		return -EINVAL;
	}

	return 0;
}

static int lantiq_rcu_reset_probe(struct platform_device *pdev)
{
	struct lantiq_rcu_reset_priv *priv;
	int err;

	priv = devm_kzalloc(&pdev->dev,
				sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	err = lantiq_rcu_reset_of_probe(pdev, priv);
	if (err)
		return err;

	priv->rcdev.ops = &lantiq_rcu_reset_ops;
	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.of_node = pdev->dev.of_node;
	priv->rcdev.of_reset_n_cells = 1;
	priv->rcdev.nr_resets = 32;

	err = reset_controller_register(&priv->rcdev);
	if (err)
		return err;

	return 0;
}

static const struct of_device_id lantiq_rcu_reset_dt_ids[] = {
	{ .compatible = "lantiq,rcu-reset", },
	{ },
};
MODULE_DEVICE_TABLE(of, lantiq_rcu_reset_dt_ids);

static struct platform_driver lantiq_rcu_reset_driver = {
	.probe	= lantiq_rcu_reset_probe,
	.driver = {
		.name		= "lantiq-rcu-reset",
		.of_match_table	= lantiq_rcu_reset_dt_ids,
	},
};
module_platform_driver(lantiq_rcu_reset_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY RCU Reset Controller Driver");
MODULE_LICENSE("GPL");
