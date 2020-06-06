/*
 * Copyright (c) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define LTQ_MPS_CHIPID						0x0
#define LTS_MPS_CHIPID_PART					GENMASK(27, 12)
#define LTQ_MPS_CHIPID_PART_VRX220				0x000
#define LTQ_MPS_CHIPID_PART_ARX362				0x004
#define LTQ_MPS_CHIPID_PART_ARX368				0x005
#define LTQ_MPS_CHIPID_PART_ARX382				0x007
#define LTQ_MPS_CHIPID_PART_ARX388				0x008
#define LTQ_MPS_CHIPID_PART_URX388				0x009
#define LTQ_MPS_CHIPID_PART_GRX383				0x010
#define LTQ_MPS_CHIPID_PART_GRX369				0x011
#define LTQ_MPS_CHIPID_PART_GRX389				0x012
#define LTQ_MPS_CHIPID_PART_VRX288_2				0x00b
#define LTQ_MPS_CHIPID_PART_VRX268_2				0x00c
#define LTQ_MPS_CHIPID_PART_GRX288_2				0x00d
#define LTQ_MPS_CHIPID_PART_GRX282_2				0x00e
#define LTQ_MPS_CHIPID_PART_GRX387				0x00f
#define LTQ_MPS_CHIPID_PART_DANUBE1				0x129
#define LTQ_MPS_CHIPID_PART_DANUBE2				0x12b
#define LTQ_MPS_CHIPID_PART_TWINPASS				0x12d
#define LTQ_MPS_CHIPID_PART_AMAZON_SE_1				0x152
#define LTQ_MPS_CHIPID_PART_AMAZON_SE_2				0x153
#define LTQ_MPS_CHIPID_PART_ARX188				0x16c
#define LTQ_MPS_CHIPID_PART_ARX168_1				0x16d
#define LTQ_MPS_CHIPID_PART_ARX168_2				0x16e
#define LTQ_MPS_CHIPID_PART_ARX182				0x16f
#define LTQ_MPS_CHIPID_PART_GRX188				0x170
#define LTQ_MPS_CHIPID_PART_GRX168				0x171
#define LTQ_MPS_CHIPID_PART_VRX288				0x1c0
#define LTQ_MPS_CHIPID_PART_VRX282				0x1c1
#define LTQ_MPS_CHIPID_PART_VRX268				0x1c2
#define LTQ_MPS_CHIPID_PART_GRX268				0x1c8
#define LTQ_MPS_CHIPID_PART_GRX288				0x1c9
#define LTS_MPS_CHIPID_REV					GENMASK(31, 28)

static int __init ltq_xway_mps_socinfo_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	void __iomem *chipid_regs;
	struct device_node *np;
	u32 chipid;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "lantiq,xway-mps-chipid");
	if (!np)
		return -ENODEV;

	chipid_regs = of_iomap(np, 0);
	if (!chipid_regs) {
		ret = -ENODEV;
		goto err_put_np;
	}

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	chipid = ioread32be(chipid_regs + LTQ_MPS_CHIPID);

	iounmap(chipid_regs);
	of_node_put(np);

	np = of_find_node_by_path("/");
	of_property_read_string(np, "model", &soc_dev_attr->machine);
	of_node_put(np);

	regmap_read(mps_regmap, LTQ_MPS_CHIPID, &val);

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "1.%u",
					   FIELD_GET(LTS_MPS_CHIPID_REV,
						     chipid));

	switch (FIELD_GET(LTS_MPS_CHIPID_PART, chipid)) {
	case LTQ_MPS_CHIPID_PART_DANUBE1:
	case LTQ_MPS_CHIPID_PART_DANUBE2:
		soc_dev_attr->family = "Lantiq XWAY DANUBE";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "DANUBE");
		break;

	case LTQ_MPS_CHIPID_PART_TWINPASS:
		soc_dev_attr->family = "Lantiq XWAY TwinPass";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "TwinPass");
		break;

	case LTQ_MPS_CHIPID_PART_ARX188:
		soc_dev_attr->family = "Lantiq XWAY ARX100";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX188");
		break;

	case LTQ_MPS_CHIPID_PART_ARX168_1:
	case LTQ_MPS_CHIPID_PART_ARX168_2:
		soc_dev_attr->family = "Lantiq XWAY ARX100";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX168");
		break;

	case LTQ_MPS_CHIPID_PART_ARX182:
		soc_dev_attr->family = "Lantiq XWAY ARX100";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX182");
		break;

	case LTQ_MPS_CHIPID_PART_GRX188:
		soc_dev_attr->family = "Lantiq XWAY GRX100";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX188");
		break;

	case LTQ_MPS_CHIPID_PART_GRX168:
		soc_dev_attr->family = "Lantiq XWAY GRX100";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX168");
		break;

	case LTQ_MPS_CHIPID_PART_AMAZON_SE_1:
	case LTQ_MPS_CHIPID_PART_AMAZON_SE_2:
		soc_dev_attr->family = "Lantiq XWAY AMAZON-SE";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "AMAZON-SE");
		break;

	case LTQ_MPS_CHIPID_PART_VRX282:
		soc_dev_attr->family = "Lantiq XWAY VRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "VRX282");
		break;

	case LTQ_MPS_CHIPID_PART_VRX268:
		soc_dev_attr->family = "Lantiq XWAY VRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "VRX268");
		break;

	case LTQ_MPS_CHIPID_PART_VRX288:
		soc_dev_attr->family = "Lantiq XWAY VRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "VRX288");
		break;

	case LTQ_MPS_CHIPID_PART_GRX268:
		soc_dev_attr->family = "Lantiq XWAY GRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX268");
		break;

	case LTQ_MPS_CHIPID_PART_GRX288:
		soc_dev_attr->family = "Lantiq XWAY GRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX288");
		break;

	case LTQ_MPS_CHIPID_PART_VRX268_2:
	case LTQ_MPS_CHIPID_PART_VRX288_2:
		soc_dev_attr->family = "Lantiq XWAY VRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "VRX268");
		break;

	case LTQ_MPS_CHIPID_PART_VRX220:
		soc_dev_attr->family = "Lantiq XWAY VRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "VRX220");
		break;

	case LTQ_MPS_CHIPID_PART_GRX282_2:
		soc_dev_attr->family = "Lantiq XWAY GRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX282");
		break;

	case LTQ_MPS_CHIPID_PART_GRX288_2:
		soc_dev_attr->family = "Lantiq XWAY GRX200";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX288");
		break;

	case LTQ_MPS_CHIPID_PART_ARX362:
		soc_dev_attr->family = "Lantiq XWAY ARX300";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX362");
		break;

	case LTQ_MPS_CHIPID_PART_ARX368:
		soc_dev_attr->family = "Lantiq XWAY ARX300";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX368");
		break;

	case LTQ_MPS_CHIPID_PART_ARX382:
		soc_dev_attr->family = "Lantiq XWAY ARX300";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX382");
		break;

	case LTQ_MPS_CHIPID_PART_ARX388:
		soc_dev_attr->family = "Lantiq XWAY ARX300";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "ARX388");
		break;

	case LTQ_MPS_CHIPID_PART_URX388:
		soc_dev_attr->family = "Lantiq XWAY ARX300";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "URX388");
		break;

	case LTQ_MPS_CHIPID_PART_GRX383:
		soc_dev_attr->family = "Lantiq XWAY GRX330";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX383");
		break;

	case LTQ_MPS_CHIPID_PART_GRX369:
		soc_dev_attr->family = "Lantiq XWAY GRX330";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX369");
		break;

	case LTQ_MPS_CHIPID_PART_GRX387:
		soc_dev_attr->family = "Lantiq XWAY GRX330";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX387");
		break;

	case LTQ_MPS_CHIPID_PART_GRX389:
		soc_dev_attr->family = "Lantiq XWAY GRX330";
		soc_dev_attr->soc_id = kstrdup_const(GFP_KERNEL, "GRX389");
		break;

	default:
		soc_dev_attr->family = "Lantiq XWAY UNKNOWN";
		soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "0x%04x",
						 FIELD_GET(LTS_MPS_CHIPID_PART,
							   chipid));
		break;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree_const(soc_dev_attr->revision);
		kfree_const(soc_dev_attr->soc_id);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	dev_info(soc_device_to_device(soc_dev), "%s (%s) rev %s detected\n",
		 soc_dev_attr->family, soc_dev_attr->soc_id,
		 soc_dev_attr->revision);

	return 0;

err_iounmap:
	iounmap(chipid_regs);
err_put_np:
	of_node_put(np);
	return ret;
}
device_initcall(ltq_xway_mps_socinfo_init);
