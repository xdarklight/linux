// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com> */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/property.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "xhci.h"
#include "xhci-pci.h"

#define EJ1X8_GPIO_INIT					0x44
#define EJ1X8_GPIO_WRITE				0x68
#define EJ1X8_GPIO_READ					0x6c

#define EJ1X8_GPIO_CTRL					0x18005020
#define EJ1X8_GPIO_CTRL_READ_ALL_MASK			GENMASK(7, 0)
#define EJ1X8_GPIO_CTRL_WRITE_ALL_MASK			GENMASK(23, 16)
#define EJ1X8_GPIO_CTRL_OUT_LOW				0x0
#define EJ1X8_GPIO_CTRL_OUT_HIGH			0x1
#define EJ1X8_GPIO_CTRL_IN				0x2
#define EJ1X8_GPIO_CTRL_MASK				0x3

#define EJ1X8_GPIO_MODE					0x18005022
#define EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK		GENMASK(23, 16)
#define EJ1X8_GPIO_MODE_DISABLE				0x0
#define EJ1X8_GPIO_MODE_ENABLE				0x1
#define EJ1X8_GPIO_MODE_MASK				0x3

struct xhci_pci_etron_priv {
	spinlock_t			lock;
	struct pci_dev			*pci_dev;
	struct gpio_chip		chip;
};

static u8 xhci_pci_etron_gpio_shift(unsigned int gpio, u8 mask)
{
	return (gpio * fls(mask));
}

static u8 xhci_pci_etron_gpio_mask(unsigned int gpio, u8 mask)
{
	return mask << xhci_pci_etron_gpio_shift(gpio, mask);
}

static int xhci_pci_etron_gpio_read(struct gpio_chip *gc, u32 reg, u32 *value)
{
	struct xhci_pci_etron_priv *etron_priv = gpiochip_get_data(gc);
	int err;

	err = pci_write_config_dword(etron_priv->pci_dev, EJ1X8_GPIO_WRITE, reg);
	if (err) {
		dev_err(gc->parent, "Failed to select 0x%08x register\n", reg);
		return err;
	}

	usleep_range(1000, 10000);

	err = pci_read_config_dword(etron_priv->pci_dev, EJ1X8_GPIO_READ, value);
	if (err) {
		dev_err(gc->parent, "Failed to read 0x%08x register\n", reg);
		return err;
	}

	return 0;
}

static int xhci_pci_etron_gpio_write(struct gpio_chip *gc, u32 reg, u32 value)
{
	struct xhci_pci_etron_priv *etron_priv = gpiochip_get_data(gc);
	int err;

	err = pci_write_config_dword(etron_priv->pci_dev, EJ1X8_GPIO_WRITE,
				     reg | value | BIT(24));
	if (err) {
		dev_err(gc->parent, "Failed to write 0x%08x register\n", reg);
		return err;
	}

	usleep_range(1000, 10000);

	return 0;
}

static int xhci_pci_etron_gpio_config(struct gpio_chip *gc, unsigned int gpio,
				      u8 mode, u8 ctrl)
{
	struct xhci_pci_etron_priv *etron_priv = gpiochip_get_data(gc);
	u8 all_gpio_ctrl, all_gpio_mode;
	u32 temp;
	int err;

	spin_lock(&etron_priv->lock);

	err = pci_read_config_dword(etron_priv->pci_dev, EJ1X8_GPIO_INIT, &temp);
	if (err) {
		dev_err(gc->parent, "Failed to read INIT register\n");
		return err;
	}

	err = pci_write_config_dword(etron_priv->pci_dev, EJ1X8_GPIO_INIT,
				     temp | 0x1);
	if (err) {
		dev_err(gc->parent, "Failed to write INIT register\n");
		return err;
	}

	err = xhci_pci_etron_gpio_read(gc, EJ1X8_GPIO_CTRL, &temp);
	if (err)
		goto err_unlock;

	all_gpio_ctrl = FIELD_GET(EJ1X8_GPIO_CTRL_READ_ALL_MASK, temp);
	all_gpio_ctrl &= ~xhci_pci_etron_gpio_mask(gpio, EJ1X8_GPIO_CTRL_MASK);
	all_gpio_ctrl |= ctrl << xhci_pci_etron_gpio_shift(gpio,
							   EJ1X8_GPIO_CTRL_MASK);

	err = xhci_pci_etron_gpio_read(gc, EJ1X8_GPIO_MODE, &temp);
	if (err)
		goto err_unlock;

	all_gpio_mode = FIELD_GET(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK, temp);
	all_gpio_mode &= ~xhci_pci_etron_gpio_mask(gpio, EJ1X8_GPIO_MODE_MASK);
	all_gpio_mode |= mode << xhci_pci_etron_gpio_shift(gpio,
							   EJ1X8_GPIO_MODE_MASK);

	err = xhci_pci_etron_gpio_write(gc, EJ1X8_GPIO_CTRL,
					FIELD_PREP(EJ1X8_GPIO_CTRL_WRITE_ALL_MASK,
						   all_gpio_ctrl));
	if (err)
		goto err_unlock;

	err = xhci_pci_etron_gpio_write(gc, EJ1X8_GPIO_MODE,
					FIELD_PREP(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK,
						   all_gpio_mode));
	if (err)
		goto err_unlock;

	spin_unlock(&etron_priv->lock);

	return 0;

err_unlock:
	spin_unlock(&etron_priv->lock);
	return err;
}

static int xhci_pci_etron_gpio_get_mode(struct gpio_chip *gc, unsigned int gpio,
					u8 *mode)
{
	struct xhci_pci_etron_priv *etron_priv = gpiochip_get_data(gc);
	u32 temp, all_gpio_mode;
	int err;

	spin_lock(&etron_priv->lock);
	err = xhci_pci_etron_gpio_read(gc, EJ1X8_GPIO_MODE, &temp);
	spin_unlock(&etron_priv->lock);

	if (err)
		return err;

	all_gpio_mode = FIELD_GET(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK, temp);
	*mode = all_gpio_mode >> xhci_pci_etron_gpio_shift(gpio,
							   EJ1X8_GPIO_MODE_MASK);
	*mode &= EJ1X8_GPIO_MODE_MASK;

	return 0;
}

static void xhci_pci_etron_gpio_free(struct gpio_chip *gc, unsigned int gpio)
{
	xhci_pci_etron_gpio_config(gc, gpio, EJ1X8_GPIO_MODE_DISABLE,
				   EJ1X8_GPIO_CTRL_IN);
}

static int xhci_pci_etron_gpio_get_direction(struct gpio_chip *gc,
					     unsigned int gpio)
{
	u8 mode;
	int err;

	err = xhci_pci_etron_gpio_get_mode(gc, gpio, &mode);
	if (err)
		return err;

	switch (mode) {
	case EJ1X8_GPIO_CTRL_IN:
		return GPIO_LINE_DIRECTION_IN;

	case EJ1X8_GPIO_CTRL_OUT_HIGH:
	case EJ1X8_GPIO_CTRL_OUT_LOW:
		return GPIO_LINE_DIRECTION_OUT;

	default:
		return -EINVAL;
	}
}

static int xhci_pci_etron_gpio_direction_output(struct gpio_chip *gc,
						unsigned int gpio, int value)
{
	u8 gpio_ctrl;

	if (value)
		gpio_ctrl = EJ1X8_GPIO_CTRL_OUT_HIGH;
	else
		gpio_ctrl = EJ1X8_GPIO_CTRL_OUT_LOW;

	return xhci_pci_etron_gpio_config(gc, gpio, EJ1X8_GPIO_MODE_ENABLE,
					  gpio_ctrl);
}

static int xhci_pci_etron_gpio_get_value(struct gpio_chip *gc, unsigned int gpio)
{
	u8 mode;
	int err;

	err = xhci_pci_etron_gpio_get_mode(gc, gpio, &mode);
	if (err)
		return err;

	switch (mode) {
	case EJ1X8_GPIO_CTRL_OUT_HIGH:
		return 1;

	case EJ1X8_GPIO_CTRL_OUT_LOW:
		return 0;

	default:
		return -EINVAL;
	}
}

static void xhci_pci_etron_gpio_set_value(struct gpio_chip *gc,
					  unsigned int gpio, int value)
{
	xhci_pci_etron_gpio_direction_output(gc, gpio, value);
}

static int xhci_pci_etron_probe(struct pci_dev *pci_dev,
				const struct pci_device_id *id)
{
	struct xhci_pci_etron_priv *etron_priv;
	struct device *dev = &pci_dev->dev;
	int err;

	if (!IS_ENABLED(CONFIG_GPIOLIB) ||
	    !device_property_present(dev, "gpio-controller"))
		return xhci_pci_common_probe(pci_dev, id);

	etron_priv = devm_kzalloc(dev, sizeof(*etron_priv), GFP_KERNEL);
	if (!etron_priv)
		return -ENOMEM;

	spin_lock_init(&etron_priv->lock);
	etron_priv->pci_dev = pci_dev;

	/* TODO: input mode is supported by the hardware but not the driver */
	etron_priv->chip.label = dev_name(dev);
	etron_priv->chip.owner = THIS_MODULE;
	etron_priv->chip.parent = dev;
	etron_priv->chip.free = xhci_pci_etron_gpio_free;
	etron_priv->chip.get_direction = xhci_pci_etron_gpio_get_direction;
	etron_priv->chip.direction_output = xhci_pci_etron_gpio_direction_output;
	etron_priv->chip.get = xhci_pci_etron_gpio_get_value;
	etron_priv->chip.set = xhci_pci_etron_gpio_set_value;
	etron_priv->chip.base = -1;
	etron_priv->chip.ngpio = 4;

	err = devm_gpiochip_add_data(dev, &etron_priv->chip, etron_priv);
	if (err) {
		dev_warn(dev, "Failed to register GPIO device: %d\n", err);
		return err;
	}

	return xhci_pci_common_probe(pci_dev, id);
}

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_ETRON, PCI_DEVICE_ID_ETRON_EJ168) },
	{ PCI_DEVICE(PCI_VENDOR_ID_ETRON, PCI_DEVICE_ID_ETRON_EJ188) },
	{ /* end: all zeroes */ }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver xhci_etron_pci_driver = {
	.name =		"xhci-pci-etron",
	.id_table =	pci_ids,

	.probe =	xhci_pci_etron_probe,
	.remove =	xhci_pci_remove,

	.shutdown = 	usb_hcd_pci_shutdown,
	.driver = {
		.pm = pm_ptr(&usb_hcd_pci_pm_ops),
	},
};
module_pci_driver(xhci_etron_pci_driver);

MODULE_DESCRIPTION("Etron EJ168/EJ188/EJ198 xHCI PCI Host Controller Driver");
MODULE_IMPORT_NS("xhci");
MODULE_LICENSE("GPL v2");
