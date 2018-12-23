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
				      struct genpd_onecell_data *genpd_data,
				      int id)
{
	const struct meson_pwrc_domain_data *field;
	struct meson_pwrc_regmap_domain *pd;

	field = &pwrc->pwrc_data->domains[id];

	pd = devm_kzalloc(pwrc->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->regmap_field = devm_regmap_field_alloc(pwrc->dev, pwrc->regmap,
						   field->reg_field);
	if (IS_ERR(pd->regmap_field))
		return PTR_ERR(pd->regmap_field);

	pd->pwrc = pwrc;
	pd->genpd.name = field->name;

	pd->genpd.power_on = meson_regmap_pwrc_clear_bits;
	pd->genpd.power_off = meson_regmap_pwrc_set_bits;

	genpd_data->domains[id] = &pd->genpd;

	return pm_genpd_init(&pd->genpd, NULL, false);
}

int meson_regmap_pwrc_init(struct device *dev, struct regmap *regmap,
			   const struct meson_regmap_pwrc_data *pwrc_data)
{
	struct genpd_onecell_data *genpd_data;
	struct meson_regmap_pwrc *pwrc;
	int i, ret;

	genpd_data = devm_kzalloc(dev, struct_size(genpd_data, domains,
						   pwrc_data->num_domains),
				  GFP_KERNEL);
	if (!genpd_data)
		return -ENOMEM;

	pwrc = devm_kzalloc(dev, sizeof(*pwrc), GFP_KERNEL);
	if (!pwrc)
		return -ENOMEM;

	genpd_data->num_domains = pwrc_data->num_domains;

	pwrc->dev = dev;
	pwrc->pwrc_data = pwrc_data;
	pwrc->regmap = regmap;

	for (i = 0; i < pwrc_data->num_domains; i++) {
		/* the array might be sparse */
		if (!pwrc->pwrc_data->domains[i].name)
			return 0;

		ret = meson_regmap_pwrc_init_domain(pwrc, genpd_data, i);
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
