/*
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DRIVERS_PHY_HUB_H
#define __DRIVERS_PHY_HUB_H

#ifdef CONFIG_GENERIC_PHY_HUB

struct phy *phy_hub_create_from_child_nodes(struct device *parent,
					    struct device_node *hub_node,
					    const char *con_id);

#else /* CONFIG_GENERIC_PHY_HUB */

static inline struct phy *phy_hub_create_from_child_nodes(
					struct device *parent,
					struct device_node *hub_node,
					const char *con_id)
{
	return ERR_PTR(-ENODEV);
}

#endif /* CONFIG_GENERIC_PHY_HUB */

#endif /* __DRIVERS_PHY_HUB_H */
