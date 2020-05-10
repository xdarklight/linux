// SPDX-License-Identifier: GPL-2.0
/*
 * USB EXTCON Based Connection Detection Driver
 *
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on USB GPIO Based Connection Detection Driver:
 *   Copyright (C) 2019 MediaTek Inc.
 *   Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <linux/device.h>
#include <linux/extcon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>

struct extcon_usb_conn_info {
	struct device *dev;
	struct usb_role_switch *role_sw;
	enum usb_role last_role;
	struct regulator *vbus;
	struct delayed_work dw_det;

	struct extcon_dev *edev;
	struct notifier_block edev_nb;
};

static int extcon_usb_conn_edev_notifier(struct notifier_block *nb,
					 unsigned long event, void *ptr)
{
	struct extcon_usb_conn_info *info;
	enum usb_role new_role;
	int ret;

	info = container_of(nb, struct extcon_usb_conn_info, edev_nb);

	if (extcon_get_state(info->edev, EXTCON_USB_HOST) > 0)
		new_role = USB_ROLE_HOST;
	else if (extcon_get_state(info->edev, EXTCON_USB) > 0)
		new_role = USB_ROLE_DEVICE;
	else
		new_role = USB_ROLE_NONE;

	dev_dbg(info->dev, "Switching from role %u to %u\n", info->last_role,
		new_role);

	if (info->last_role == new_role) {
		dev_dbg(info->dev, "Repeated role: %u\n", new_role);
		return NOTIFY_DONE;
	}

	if (info->last_role == USB_ROLE_HOST)
		regulator_disable(info->vbus);

	ret = usb_role_switch_set_role(info->role_sw, new_role);
	if (ret)
		dev_err(info->dev, "Failed to set role: %d\n", ret);

	if (new_role == USB_ROLE_HOST) {
		ret = regulator_enable(info->vbus);
		if (ret)
			dev_err(info->dev,
				"Failed to enable the VBUS regulator\n");
	}

	info->last_role = new_role;

	dev_dbg(info->dev, "VBUS regulator is %s\n",
		regulator_is_enabled(info->vbus) ? "enabled" : "disabled");

	return NOTIFY_OK;
}

static int extcon_usb_conn_probe(struct platform_device *pdev)
{
	struct extcon_usb_conn_info *info;
	struct device *dev = &pdev->dev;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->last_role = USB_ROLE_NONE;
	info->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(info->edev))
		return PTR_ERR(info->edev);

	info->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(info->vbus)) {
		if (PTR_ERR(info->vbus) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get VBUS regulator\n");
		return PTR_ERR(info->vbus);
	}

	info->role_sw = usb_role_switch_get(dev);
	if (IS_ERR(info->role_sw)) {
		if (PTR_ERR(info->role_sw) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get USB role switch\n");

		return PTR_ERR(info->role_sw);
	}

	info->edev_nb.notifier_call = extcon_usb_conn_edev_notifier;
	ret = extcon_register_notifier_all(info->edev, &info->edev_nb);
	if (ret)
		goto err_put_role_sw;

	platform_set_drvdata(pdev, info);

	return 0;

err_put_role_sw:
	usb_role_switch_put(info->role_sw);
	return ret;
}

static int extcon_usb_conn_remove(struct platform_device *pdev)
{
	struct extcon_usb_conn_info *info = platform_get_drvdata(pdev);

	usb_role_switch_put(info->role_sw);

	return 0;
}

static const struct of_device_id extcon_usb_conn_of_match[] = {
	{ .compatible = "extcon-usb-b-connector", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, extcon_usb_conn_dt_match);

static struct platform_driver extcon_usb_conn_driver = {
	.probe		= extcon_usb_conn_probe,
	.remove		= extcon_usb_conn_remove,
	.driver		= {
		.name	= "extcon-usb-conn-gpio",
		.of_match_table = extcon_usb_conn_of_match,
	},
};

module_platform_driver(extcon_usb_conn_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("USB EXTCON based connection detection driver");
MODULE_LICENSE("GPL v2");
