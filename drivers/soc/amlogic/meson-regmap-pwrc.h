/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/pm_domain.h>
#include <linux/regmap.h>

#ifndef MESON_PWRC_H
#define MESON_PWRC_H

struct meson_regmap_pwrc_domain {
	const char *name;
	struct reg_field reg_field;
};

struct meson_regmap_pwrc_data {
	const struct meson_regmap_pwrc_domain *domains;
	const int num_domains;
};

struct meson_pwrc_regmap_domain {
	struct generic_pm_domain genpd;
	struct regmap_field *regmap_field;
	u32 value_mask;
	struct meson_regmap_pwrc *pwrc;
};

struct meson_regmap_pwrc {
	struct device *dev;
	struct regmap *regmap;
	struct generic_pm_domain *domains[];
};

static inline
struct meson_pwrc_regmap_domain *to_meson_pd(struct generic_pm_domain *d)
{
	return container_of(d, struct meson_pwrc_regmap_domain, genpd);
}

int meson_regmap_pwrc_init(struct device *dev, struct regmap *regmap,
			   const struct meson_regmap_pwrc_data *pwrc_data);

int meson_regmap_pwrc_power_off(struct generic_pm_domain *genpd);
int meson_regmap_pwrc_power_on(struct generic_pm_domain *genpd);

#endif /* MESON_PWRC_H */
