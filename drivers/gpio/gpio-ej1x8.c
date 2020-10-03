// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com> */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/property.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>

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

static LIST_HEAD(ej1x8_gpios);

struct ej1x8_gpio {
	spinlock_t			lock;
	struct pci_dev			*pci_dev;
	struct gpio_chip		chip;
	struct list_head		list_head;
};

static u8 ej1x8_gpio_shift(unsigned int gpio, u8 mask)
{
	return (gpio * fls(mask));
}

static u8 ej1x8_gpio_mask(unsigned int gpio, u8 mask)
{
	return mask << ej1x8_gpio_shift(gpio, mask);
}

static int ej1x8_gpio_read(struct gpio_chip *gc, u32 reg, u32 *value)
{
	struct ej1x8_gpio *ej1x8 = gpiochip_get_data(gc);
	int err;

	err = pci_write_config_dword(ej1x8->pci_dev, EJ1X8_GPIO_WRITE, reg);
	if (err) {
		dev_err(gc->parent, "Failed to select 0x%08x register\n", reg);
		return err;
	}

	usleep_range(1000, 10000);

	err = pci_read_config_dword(ej1x8->pci_dev, EJ1X8_GPIO_READ, value);
	if (err) {
		dev_err(gc->parent, "Failed to read 0x%08x register\n", reg);
		return err;
	}

	return 0;
}

static int ej1x8_gpio_write(struct gpio_chip *gc, u32 reg, u32 value)
{
	struct ej1x8_gpio *ej1x8 = gpiochip_get_data(gc);
	int err;

	err = pci_write_config_dword(ej1x8->pci_dev, EJ1X8_GPIO_WRITE,
				     reg | value | BIT(24));
	if (err) {
		dev_err(gc->parent, "Failed to write 0x%08x register\n", reg);
		return err;
	}

	usleep_range(1000, 10000);

	return 0;
}

static int ej1x8_gpio_config(struct gpio_chip *gc, unsigned int gpio, u8 mode,
			     u8 ctrl)
{
	struct ej1x8_gpio *ej1x8 = gpiochip_get_data(gc);
	u8 all_gpio_ctrl, all_gpio_mode;
	u32 temp;
	int err;

	spin_lock(&ej1x8->lock);

	err = pci_read_config_dword(ej1x8->pci_dev, EJ1X8_GPIO_INIT, &temp);
	if (err) {
		dev_err(gc->parent, "Failed to read INIT register\n");
		return err;
	}

	err = pci_write_config_dword(ej1x8->pci_dev, EJ1X8_GPIO_INIT,
				     temp | 0x1);
	if (err) {
		dev_err(gc->parent, "Failed to write INIT register\n");
		return err;
	}

	err = ej1x8_gpio_read(gc, EJ1X8_GPIO_CTRL, &temp);
	if (err)
		goto err_unlock;

	all_gpio_ctrl = FIELD_GET(EJ1X8_GPIO_CTRL_READ_ALL_MASK, temp);
	all_gpio_ctrl &= ~ej1x8_gpio_mask(gpio, EJ1X8_GPIO_CTRL_MASK);
	all_gpio_ctrl |= ctrl << ej1x8_gpio_shift(gpio, EJ1X8_GPIO_CTRL_MASK);

	err = ej1x8_gpio_read(gc, EJ1X8_GPIO_MODE, &temp);
	if (err)
		goto err_unlock;

	all_gpio_mode = FIELD_GET(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK, temp);
	all_gpio_mode &= ~ej1x8_gpio_mask(gpio, EJ1X8_GPIO_MODE_MASK);
	all_gpio_mode |= mode << ej1x8_gpio_shift(gpio, EJ1X8_GPIO_MODE_MASK);

	err = ej1x8_gpio_write(gc, EJ1X8_GPIO_CTRL,
			       FIELD_PREP(EJ1X8_GPIO_CTRL_WRITE_ALL_MASK,
					  all_gpio_ctrl));
	if (err)
		goto err_unlock;

	err = ej1x8_gpio_write(gc, EJ1X8_GPIO_MODE,
			       FIELD_PREP(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK,
					  all_gpio_mode));
	if (err)
		goto err_unlock;

	spin_unlock(&ej1x8->lock);

	return 0;

err_unlock:
	spin_unlock(&ej1x8->lock);
	return err;
}

static int ej1x8_gpio_get_mode(struct gpio_chip *gc, unsigned int gpio, u8 *mode)
{
	struct ej1x8_gpio *ej1x8 = gpiochip_get_data(gc);
	u32 temp, all_gpio_mode;
	int err;

	spin_lock(&ej1x8->lock);
	err = ej1x8_gpio_read(gc, EJ1X8_GPIO_MODE, &temp);
	spin_unlock(&ej1x8->lock);

	if (err)
		return err;

	all_gpio_mode = FIELD_GET(EJ1X8_GPIO_MODE_READ_WRITE_ALL_MASK, temp);
	*mode = all_gpio_mode >> ej1x8_gpio_shift(gpio, EJ1X8_GPIO_MODE_MASK);
	*mode &= EJ1X8_GPIO_MODE_MASK;

	return 0;
}

static void ej1x8_gpio_free(struct gpio_chip *gc, unsigned int gpio)
{
	ej1x8_gpio_config(gc, gpio, EJ1X8_GPIO_MODE_DISABLE, EJ1X8_GPIO_CTRL_IN);
}

static int ej1x8_gpio_get_direction(struct gpio_chip *gc, unsigned int gpio)
{
	u8 mode;
	int err;

	err = ej1x8_gpio_get_mode(gc, gpio, &mode);
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

static int ej1x8_gpio_direction_output(struct gpio_chip *gc, unsigned int gpio,
				       int value)
{
	u8 gpio_ctrl;

	if (value)
		gpio_ctrl = EJ1X8_GPIO_CTRL_OUT_HIGH;
	else
		gpio_ctrl = EJ1X8_GPIO_CTRL_OUT_LOW;

	return ej1x8_gpio_config(gc, gpio, EJ1X8_GPIO_MODE_ENABLE, gpio_ctrl);
}

static int ej1x8_gpio_get_value(struct gpio_chip *gc, unsigned int gpio)
{
	u8 mode;
	int err;

	err = ej1x8_gpio_get_mode(gc, gpio, &mode);
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

static void ej1x8_gpio_set_value(struct gpio_chip *gc, unsigned int gpio, int value)
{
	ej1x8_gpio_direction_output(gc, gpio, value);
}

static const struct pci_device_id ej1x8_gpio_pci_ids[] __initconst = {
	{ PCI_DEVICE(PCI_VENDOR_ID_ETRON, PCI_DEVICE_ID_ETRON_EJ168) },
	{ PCI_DEVICE(PCI_VENDOR_ID_ETRON, PCI_DEVICE_ID_ETRON_EJ188) },
	{ /* sentinel */ }
};

static int __init ej1x8_gpio_init(void)
{
	struct pci_dev *pci_dev = NULL;
	struct ej1x8_gpio *ej1x8;
	int err;

	for_each_pci_dev(pci_dev) {
		if (!pci_match_id(ej1x8_gpio_pci_ids, pci_dev))
			continue;

		if (!device_property_read_bool(&pci_dev->dev, "gpio-controller"))
			continue;

		ej1x8 = kzalloc(sizeof(*ej1x8), GFP_KERNEL);
		if (!ej1x8)
			continue;

		spin_lock_init(&ej1x8->lock);
		ej1x8->pci_dev = pci_dev_get(pci_dev);

		/* TODO: input mode is supported by the hardware but not the driver */
		ej1x8->chip.label = dev_name(&pci_dev->dev);
		ej1x8->chip.owner = THIS_MODULE;
		ej1x8->chip.parent = &pci_dev->dev;
		ej1x8->chip.of_node = pci_dev->dev.of_node;
		ej1x8->chip.free = ej1x8_gpio_free;
		ej1x8->chip.get_direction = ej1x8_gpio_get_direction;
		ej1x8->chip.direction_output = ej1x8_gpio_direction_output;
		ej1x8->chip.get = ej1x8_gpio_get_value;
		ej1x8->chip.set = ej1x8_gpio_set_value;
		ej1x8->chip.base = -1;
		ej1x8->chip.ngpio = 4;

		err = gpiochip_add_data(&ej1x8->chip, ej1x8);
		if (err) {
			dev_warn(&pci_dev->dev,
				 "Failed to register GPIO device: %d\n", err);
			pci_dev_put(ej1x8->pci_dev);
			kfree(ej1x8);
			continue;
		}

		list_add(&ej1x8->list_head, &ej1x8_gpios);
	}

	return 0;
}

static void __exit ej1x8_gpio_exit(void)
{
	struct ej1x8_gpio *ej1x8, *tmp;

	list_for_each_entry_safe(ej1x8, tmp, &ej1x8_gpios, list_head) {
		gpiochip_remove(&ej1x8->chip);
		pci_dev_put(ej1x8->pci_dev);
		list_del(&ej1x8->list_head);
		kfree(ej1x8);
	}
}

module_init(ej1x8_gpio_init);
module_exit(ej1x8_gpio_exit);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Etron Technology Inc. EJ168/EJ188/EJ198 GPIO driver");
MODULE_LICENSE("GPL v2");
