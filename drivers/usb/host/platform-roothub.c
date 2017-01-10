/*
 * platform roothub driver - a virtual PHY device which passes all phy_*
 * function calls to multiple (actual) PHY devices. This is comes handy when
 * initializing all PHYs on a root-hub (to keep them all in the same state).
 *
 * Copyright (C) 2017 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/phy/phy.h>
#include <linux/of.h>
#include <linux/usb/of.h>

#include "platform-roothub.h"

#define ROOTHUB_PORTNUM		0

struct platform_roothub {
	struct phy		*phy;
	struct list_head	list;
};

static struct platform_roothub *platform_roothub_alloc(struct device *dev)
{
	struct platform_roothub *roothub_entry;

	roothub_entry = devm_kzalloc(dev, sizeof(*roothub_entry), GFP_KERNEL);
	if (!roothub_entry)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&roothub_entry->list);

	return roothub_entry;
}

static int platform_roothub_add_phy(struct device *dev,
				    struct device_node *port_np,
				    const char *con_id, struct list_head *list)
{
	struct platform_roothub *roothub_entry;
	struct phy *phy = devm_of_phy_get(dev, port_np, con_id);

	if (IS_ERR_OR_NULL(phy)) {
		if (!phy || PTR_ERR(phy) == -ENODEV)
			return 0;
		else
			return PTR_ERR(phy);
	}

	roothub_entry = platform_roothub_alloc(dev);
	if (IS_ERR(roothub_entry))
		return PTR_ERR(roothub_entry);

	roothub_entry->phy = phy;

	list_add_tail(&roothub_entry->list, list);

	return 0;
}

struct platform_roothub *platform_roothub_init(struct device *dev)
{
	struct device_node *roothub_np, *port_np;
	struct platform_roothub *plat_roothub;
	struct platform_roothub *roothub_entry;
	struct list_head *head;
	int err;

	roothub_np = usb_of_get_child_node(dev->of_node, ROOTHUB_PORTNUM);
	if (!of_device_is_available(roothub_np))
		return NULL;

	plat_roothub = platform_roothub_alloc(dev);
	if (IS_ERR(plat_roothub))
		return plat_roothub;

	for_each_available_child_of_node(roothub_np, port_np) {
		err = platform_roothub_add_phy(dev, port_np, "usb2-phy",
					       &plat_roothub->list);
		if (err)
			goto err_out;

		err = platform_roothub_add_phy(dev, port_np, "usb3-phy",
					       &plat_roothub->list);
		if (err)
			goto err_out;
	}

	head = &plat_roothub->list;

	list_for_each_entry(roothub_entry, head, list) {
		err = phy_init(roothub_entry->phy);
		if (err)
			goto err_exit_phys;
	}

	return plat_roothub;

err_exit_phys:
	list_for_each_entry_continue_reverse(roothub_entry, head, list)
		phy_exit(roothub_entry->phy);

err_out:
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(platform_roothub_init);

int platform_roothub_exit(struct platform_roothub *plat_roothub)
{
	struct platform_roothub *roothub_entry;
	struct list_head *head;
	int err, ret = 0;

	if (!plat_roothub)
		return 0;

	head = &plat_roothub->list;

	list_for_each_entry(roothub_entry, head, list) {
		err = phy_exit(roothub_entry->phy);
		if (err)
			ret = ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(platform_roothub_exit);

int platform_roothub_power_on(struct platform_roothub *plat_roothub)
{
	struct platform_roothub *roothub_entry;
	struct list_head *head;
	int err;

	if (!plat_roothub)
		return 0;

	head = &plat_roothub->list;

	list_for_each_entry(roothub_entry, head, list) {
		err = phy_power_on(roothub_entry->phy);
		if (err)
			goto err_out;
	}

	return 0;

err_out:
	list_for_each_entry_continue_reverse(roothub_entry, head, list)
		phy_power_off(roothub_entry->phy);

	return err;
}
EXPORT_SYMBOL_GPL(platform_roothub_power_on);

int platform_roothub_power_off(struct platform_roothub *plat_roothub)
{
	struct platform_roothub *roothub_entry;
	int err, ret = 0;

	if (!plat_roothub)
		return 0;

	list_for_each_entry_reverse(roothub_entry, &plat_roothub->list, list) {
		err = phy_power_off(roothub_entry->phy);
		if (err)
			ret = err;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(platform_roothub_power_off);
