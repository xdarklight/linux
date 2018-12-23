/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include "meson-regmap-pwrc.h"

static int meson_regmap_pwrc_clear_bits(struct generic_pm_domain *genpd)
{
	struct meson_pwrc_regmap_domain *pd = to_meson_pd(genpd);
	int err;

dev_err(&genpd->dev, "clearing bits...\n");
	err = regmap_field_write(pd->regmap_field, 0);
	if (err)
		return err;

	udelay(5);

	return 0;
}

static int meson_regmap_pwrc_set_bits(struct generic_pm_domain *genpd)
{
	struct meson_pwrc_regmap_domain *pd = to_meson_pd(genpd);
	int err;

dev_err(&genpd->dev, "setting bits...\n");
	err = regmap_field_write(pd->regmap_field, ~0);
	if (err)
		return err;

	udelay(5);

	return 0;
}

static int meson_regmap_pwrc_init_domain(struct meson_regmap_pwrc *pwrc,
				const struct meson_regmap_pwrc_domain *data,
				int id)
{
	struct meson_pwrc_regmap_domain *pd;
	u32 val;

	pd = devm_kzalloc(pwrc->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->regmap_field = devm_regmap_field_alloc(pwrc->dev, pwrc->regmap,
						   data->reg_field);
	if (IS_ERR(pd->regmap_field)) {
		dev_err(pwrc->dev, "Failed to create regmap field for %s\n",
			data->name);
		return PTR_ERR(pd->regmap_field);
	}

	pd->pwrc = pwrc;
	pd->genpd.name = data->name;

	pd->genpd.power_on = meson_regmap_pwrc_clear_bits;
	pd->genpd.power_off = meson_regmap_pwrc_set_bits;

	pwrc->domains[id] = &pd->genpd;

	regmap_field_read(pd->regmap_field, &val);

	return pm_genpd_init(&pd->genpd, NULL, val != 0);
}

int meson_regmap_pwrc_init(struct device *dev, struct regmap *regmap,
			   const struct meson_regmap_pwrc_data *pwrc_data)
{
	const struct meson_regmap_pwrc_domain *regmap_domain;
	struct genpd_onecell_data *genpd_data;
	struct meson_regmap_pwrc *pwrc;
	int i, ret;

	genpd_data = devm_kzalloc(dev, sizeof(*genpd_data), GFP_KERNEL);
	if (!genpd_data)
		return -ENOMEM;

	pwrc = devm_kzalloc(dev, struct_size(pwrc, domains,
					     pwrc_data->num_domains),
			    GFP_KERNEL);
	if (!pwrc)
		return -ENOMEM;

	genpd_data->domains = pwrc->domains;
	genpd_data->num_domains = pwrc_data->num_domains;

	pwrc->dev = dev;
	pwrc->regmap = regmap;

	for (i = 0; i < pwrc_data->num_domains; i++) {
		regmap_domain = &pwrc_data->domains[i];

		/* the array might be sparse */
		if (!regmap_domain->name)
			continue;

		ret = meson_regmap_pwrc_init_domain(pwrc, regmap_domain, i);
		if (ret)
			return ret;
	}

	ret = of_genpd_add_provider_onecell(dev->of_node, genpd_data);
	if (ret) {
		dev_err(dev, "Failed to add PD provider\n");
		return ret;
	}

	return 0;
}
