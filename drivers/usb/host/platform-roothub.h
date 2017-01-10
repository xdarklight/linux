/*
 * Generic USB roothub initialization.
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

#ifndef USB_HOST_PLATFORM_ROOTHUB_H
#define USB_HOST_PLATFORM_ROOTHUB_H

struct phy;
struct device_node;

struct platform_roothub;

struct platform_roothub *platform_roothub_init(struct device *dev);

int platform_roothub_power_on(struct platform_roothub *plat_roothub);
void platform_roothub_power_off(struct platform_roothub *plat_roothub);

#endif /* USB_HOST_PLATFORM_ROOTHUB_H */
