/*
 * OF helpers for IEEE 802.11 devices.
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file contains the code used to make IRQ descriptions in the
 * device tree to actual irq numbers on an interrupt controller
 * driver.
 */
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_ieee80211.h>

/**
 * of_ieee80211_is_2ghz_enabled - Check if the IEEE 802.11 2.4GHz frequency
 * band is enabled for the given device_node
 * @np:	Pointer to the given device_node
 *
 * When the the 2.4GHz band is defined in an conflicting state (both enabled
 * and disabled are set) then we leave it disabled.
 */
bool of_ieee80211_is_2ghz_enabled(struct device_node *np)
{
	bool enabled = of_property_read_bool(np, "enable-ieee80211-2ghz");

	if (enabled && of_ieee80211_is_2ghz_disabled(np)) {
		pr_err("%s: conflicting configuration for the 2.4GHz band\n",
		       of_node_full_name(np));
		return false;
	}

	return enabled;
}
EXPORT_SYMBOL(of_ieee80211_is_2ghz_enabled);

/**
 * of_ieee80211_is_5ghz_enabled - Check if the IEEE 802.11 5GHz frequency band
 * is enabled for the given device_node
 * @np:	Pointer to the given device_node
 *
 * When the the 5GHz band is defined in an conflicting state (both enabled
 * and disabled are set) then we leave it disabled.
 */
bool of_ieee80211_is_5ghz_enabled(struct device_node *np)
{
	bool enabled = of_property_read_bool(np, "enable-ieee80211-5ghz");

	if (enabled && of_ieee80211_is_2ghz_disabled(np)) {
		pr_err("%s: conflicting configuration for the 5GHz band\n",
		       of_node_full_name(np));
		return false;
	}

	return enabled;
}
EXPORT_SYMBOL(of_ieee80211_is_5ghz_enabled);

/**
 * of_ieee80211_is_2ghz_disabled - Check if the IEEE 802.11 2.4GHz frequency
 * band is disabled for the given device_node
 * @np:	Pointer to the given device_node
 */
bool of_ieee80211_is_2ghz_disabled(struct device_node *np)
{
	return of_property_read_bool(np, "disable-ieee80211-2ghz");
}
EXPORT_SYMBOL(of_ieee80211_is_2ghz_disabled);

/**
 * of_ieee80211_is_5ghz_disabled - Check if the IEEE 802.11 5GHz frequency
 * band is disabled for the given device_node
 * @np:	Pointer to the given device_node
 */
bool of_ieee80211_is_5ghz_disabled(struct device_node *np)
{
	return of_property_read_bool(np, "disable-ieee80211-5ghz");
}
EXPORT_SYMBOL(of_ieee80211_is_5ghz_disabled);
