// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on rtw88/pci.c:
 *   Copyright(c) 2018-2019  Realtek Corporation
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include "main.h"
#include "sdio.h"
#include "reg.h"
#include "tx.h"
#include "rx.h"
#include "fw.h"
#include "ps.h"
#include "debug.h"

#define RTW_SDIO_INDIRECT_RW_RETRIES		50

extern struct rtw_chip_info rtw8822c_hw_spec;

static bool rtw_sdio_is_bus_addr(u32 addr)
{
	return (addr & RTW_SDIO_BUS_MSK) != 0;
}

static u32 rtw_sdio_to_bus_offset(struct rtw_dev *rtwdev, u32 addr)
{
	switch (addr & RTW_SDIO_BUS_MSK) {
	case WLAN_IOREG_OFFSET:
		addr &= WLAN_IOREG_REG_MSK;
		addr |= FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				   REG_SDIO_CMD_ADDR_MAC_REG);
		break;
	case SDIO_LOCAL_OFFSET:
		addr &= SDIO_LOCAL_REG_MSK;
		addr |= FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				   REG_SDIO_CMD_ADDR_SDIO_REG);
		break;
	default:
		rtw_warn(rtwdev, "Cannot convert addr 0x%08x to bus offset",
			 addr);
	}

	return addr;
}

static u32 rtw_sdio_mask_addr(u32 addr, bool f0)
{
	/*
	 * Mask addr to remove driver defined bit and
	 * make sure addr is in valid range
	 */
	if (f0)
		return addr & 0xfff;
	else
		return addr & 0x1ffff;
}

static void* rtw_sdio_cmd53_safe_buffer(struct rtw_dev *rtwdev, void *buf,
					size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	if (virt_addr_valid(buf) && (!object_is_on_stack(buf)))
		return buf;

	memcpy(rtwsdio->sdio_func->tmpbuf, buf, min_t(size_t, count, 4));

	return rtwsdio->sdio_func->tmpbuf;
}

static u8 rtw_sdio_readb(struct rtw_dev *rtwdev, u32 addr, int *err_ret)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool f0 = addr & REG_SDIO_ADDR_F0_BIT;
	u8 tmp;

	addr = rtw_sdio_mask_addr(addr, f0);

	if (f0)
		tmp = sdio_f0_readb(rtwsdio->sdio_func, addr, err_ret);
	else
		tmp = sdio_readb(rtwsdio->sdio_func, addr, err_ret);

	return tmp;
}

static void rtw_sdio_writeb(struct rtw_dev *rtwdev, u8 data, u32 addr, int *err_ret)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool f0 = addr & REG_SDIO_ADDR_F0_BIT;

	addr = rtw_sdio_mask_addr(addr, f0);

	if (f0)
		sdio_f0_writeb(rtwsdio->sdio_func, data, addr, err_ret);
	else
		sdio_writeb(rtwsdio->sdio_func, data, addr, err_ret);
}

static void rtw_sdio_read_direct(struct rtw_dev *rtwdev, u32 addr, void *buf,
				 size_t count, bool cmd52)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 *byte_buf;
	int i, ret;

	if (cmd52) {
		byte_buf = buf;

		for (i = 0; i < count; i++) {
			byte_buf[i] = rtw_sdio_readb(rtwdev, addr + i, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed read byte %d of %lu using CMD52 from 0x%08x: %d",
					 i + 1, count, addr + i, ret);
		}
	} else {
		ret = sdio_memcpy_fromio(rtwsdio->sdio_func,
					 rtw_sdio_cmd53_safe_buffer(rtwdev, buf, count),
					 rtw_sdio_mask_addr(addr, false), count);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed read %lu byte(s) using CMD53 from 0x%08x: %d",
				 count, addr, ret);
	}
}

static void rtw_sdio_read_indirect(struct rtw_dev *rtwdev, u32 addr, void *buf,
				   size_t count, bool cmd52)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 tmp, tmpbuf[8], *byte_buf;
	u32 reg_cfg, reg_data;
	int i, retry, ret;

	reg_cfg = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_CFG);
	reg_data = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_DATA);

	if (cmd52) {
		byte_buf = buf;

		for (i = 0; i < count; i++) {
			rtw_sdio_writeb(rtwdev, (addr + i) & 0xff, reg_cfg, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_cfg for indirect reading %d of %lu: %d",
					 i + 1, count, ret);

			rtw_sdio_writeb(rtwdev, ((addr + i) >> 8) & 0xff,
					reg_cfg + 1, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_cfg + 1 for indirect reading %d of %lu: %d",
					 i + 1, count, ret);

			rtw_sdio_writeb(rtwdev, BIT(3) | BIT(4), reg_cfg + 2, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_cfg + 2 for indirect reading %d of %lu: %d",
					 i + 1, count, ret);

			for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
				tmp = rtw_sdio_readb(rtwdev, reg_cfg + 2, &ret);
				if (!ret && tmp & BIT(4))
					break;
			}

			if (retry == RTW_SDIO_INDIRECT_RW_RETRIES)
				rtw_warn(rtwdev,
					 "Failed to wait for indirect read %d of %lu, last status = 0x%08x",
					 i + 1, count, tmp);

			byte_buf[i] = rtw_sdio_readb(rtwdev, reg_data, &ret);
			if (ret)
				rtw_warn(rtwdev,
					"Failed to read indirect byte %d of %lu at 0x%08x: %d",
					i + 1, count, addr + i, ret);
		}
	} else {
		sdio_writel(rtwsdio->sdio_func, addr | BIT(19) | BIT(20),
			    reg_cfg, &ret);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed to setup reg_cfg for indirect reading: %d",
				 ret);

		for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
			ret = sdio_readsb(rtwsdio->sdio_func, tmpbuf,
					  reg_cfg + 2, sizeof(tmpbuf));
			if (!ret && tmpbuf[0] & BIT(4))
				break;
		}

		if (retry == RTW_SDIO_INDIRECT_RW_RETRIES)
			rtw_warn(rtwdev,
				 "Failed to wait for indirect read, last status = 0x%08x",
				 tmpbuf[0]);

		memcpy(buf, &tmpbuf[2], count);
	}
}

static void rtw_sdio_read(struct rtw_dev *rtwdev, u32 addr, void *buf,
			  size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool cmd52, direct;

#if 0
	if ((offset & 0xFFFF0000) == 0) {
		if (adapter->pwr_off_flow_flag == 1 ||
		    adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF ||
		    cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_RW ||
		    cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_R) {
			value8 = (u8)r_indir_sdio_88xx(adapter, offset,
						       HALMAC_IO_BYTE);
		} else {
			offset |= WLAN_IOREG_OFFSET;
			status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
			if (status != HALMAC_RET_SUCCESS) {
				PLTFM_MSG_ERR("[ERR]convert offset\n");
				return status;
			}
			value8 = (u8)PLTFM_SDIO_CMD53_R8(offset);
		}
	} else {
		status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
		if (status != HALMAC_RET_SUCCESS) {
			PLTFM_MSG_ERR("[ERR]convert offset\n");
			return status;
		}
		value8 = PLTFM_SDIO_CMD52_R(offset);
	}
#endif

	switch (count) {
	case 1:
		if (rtw_sdio_is_bus_addr(addr)) {
			cmd52 = true;
			direct = true;

			addr = rtw_sdio_to_bus_offset(rtwdev, addr);
		} else {
			if (rtwsdio->is_powered_on) {
				addr = rtw_sdio_to_bus_offset(rtwdev,
							      addr | WLAN_IOREG_OFFSET);

				cmd52 = false;
				direct = true;
			} else {
				cmd52 = true;
				direct = false;
			}
		}
		break;

	case 2:
	case 4:
		if (rtwsdio->is_powered_on) {
			cmd52 = false;
			direct = true;

			if (!rtw_sdio_is_bus_addr(addr))
				addr |= WLAN_IOREG_OFFSET;

			rtw_sdio_to_bus_offset(rtwdev, addr);
		} else {
			if (!rtw_sdio_is_bus_addr(addr)) {
				cmd52 = true;
				direct = false;
			} else {
				cmd52 = true;
				direct = true;
			}
		}
		break;

	default:
		rtw_warn(rtwdev, "Invalid size %lu for reading", count);
		return;
	}

	sdio_claim_host(rtwsdio->sdio_func);

	if (direct)
		rtw_sdio_read_direct(rtwdev, addr, buf, count, cmd52);
	else
		rtw_sdio_read_indirect(rtwdev, addr, buf, count, cmd52);

	sdio_release_host(rtwsdio->sdio_func);
}

static void rtw_sdio_write_direct(struct rtw_dev *rtwdev, u32 addr, void *buf,
				  size_t count, bool cmd52)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 *byte_buf;
	int i, ret;

	if (cmd52) {
		byte_buf = buf;

		for (i = 0; i < count; i++) {
			rtw_sdio_writeb(rtwdev, byte_buf[i], addr + i, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed write byte %d of %lu using CMD52 from 0x%08x: %d",
					 i + 1, count, addr + i, ret);
			break;
		}
	} else {
		ret = sdio_memcpy_toio(rtwsdio->sdio_func,
				       rtw_sdio_mask_addr(addr, false),
				       rtw_sdio_cmd53_safe_buffer(rtwdev, buf, count),
				       count);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed write %lu byte(s) using CMD53 from 0x%08x: %d",
				 count, addr, ret);
	}
}

static void rtw_sdio_write_indirect(struct rtw_dev *rtwdev, u32 addr, void *buf,
				    size_t count, bool cmd52)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 tmp, *byte_buf = buf;
	u32 reg_cfg, reg_data;
	int i, retry, ret;

	if (count != 1 && count != 2 && count != 4) {
		rtw_warn(rtwdev,
			 "Invalid size %lu for indirect write", count);
		return;
	}

	reg_cfg = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_CFG);
	reg_data = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_DATA);

	if (cmd52) {
		byte_buf = buf;

		rtw_sdio_writeb(rtwdev, addr & 0xff, reg_cfg, &ret);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed to setup reg_cfg for indirect writing: %d",
				 ret);

		rtw_sdio_writeb(rtwdev, (addr >> 8) & 0xff, reg_cfg + 1, &ret);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed to setup reg_cfg + 1 for indirect writing: %d",
				 ret);

		for (i = 0; i < count; i++) {
			rtw_sdio_writeb(rtwdev, byte_buf[i], reg_data + i, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_data + %d for indirect writing: %d",
					 i, ret);
		}

		rtw_sdio_writeb(rtwdev, (count / 2) | BIT(2) | BIT(4),
				reg_cfg + 2, &ret);
		if (ret)
			rtw_warn(rtwdev,
				 "Failed to setup reg_cfg + 2 for indirect writing: %d",
				 ret);

		for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
			tmp = rtw_sdio_readb(rtwdev, reg_cfg + 2, &ret);
			if (!ret && tmp & BIT(4))
				break;
		}

		if (retry == RTW_SDIO_INDIRECT_RW_RETRIES)
			rtw_warn(rtwdev,
				 "Failed to wait for indirect write, last status = 0x%08x",
				 tmp);
	} else {
		for (i = 0; i < count; i++) {
			sdio_writel(rtwsdio->sdio_func, byte_buf[i], reg_data,
				    &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_data for indirect writing %d of %lu: %d",
					 i + 1, count, ret);

			sdio_writel(rtwsdio->sdio_func,
				    addr | ((count / 2) << 16) | BIT(18) | BIT(20),
				    reg_cfg, &ret);
			if (ret)
				rtw_warn(rtwdev,
					 "Failed to setup reg_cfg for indirect writing %d of %lu: %d",
					 i + 1, count, ret);

			for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
				tmp = rtw_sdio_readb(rtwdev, reg_cfg + 2, &ret);
				if (!ret && tmp & BIT(4))
					break;
			}

			if (retry == RTW_SDIO_INDIRECT_RW_RETRIES)
				rtw_warn(rtwdev,
					 "Failed to wait for indirect write %d of %lu, last status = 0x%08x",
					 i + 1, count, tmp);
			}
	}
}

static void rtw_sdio_write(struct rtw_dev *rtwdev, u32 addr, void *buf,
			   size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool cmd52, direct;

	switch (count) {
	case 1:
		if (!rtwsdio->is_powered_on && !rtw_sdio_is_bus_addr(addr)) {
			cmd52 = true;
			direct = false;
		} else {
			cmd52 = true;
			direct = true;

			if (!rtw_sdio_is_bus_addr(addr))
				addr |= WLAN_IOREG_OFFSET;

			addr = rtw_sdio_to_bus_offset(rtwdev, addr);
		}
		break;

	case 2:
	case 4:
		if (!rtwsdio->is_powered_on || (addr & (count - 1)) != 0) {
			if (!rtw_sdio_is_bus_addr(addr) && (addr & (count - 1)) == 0) {
				cmd52 = true;
				direct = false;
			} else {
				cmd52 = true;
				direct = true;

				if (!rtw_sdio_is_bus_addr(addr))
					addr |= WLAN_IOREG_OFFSET;

				addr = rtw_sdio_to_bus_offset(rtwdev, addr);
			}
		} else {
			cmd52 = false;
			direct = true;

			if (!rtw_sdio_is_bus_addr(addr))
				addr |= WLAN_IOREG_OFFSET;

			addr = rtw_sdio_to_bus_offset(rtwdev, addr);
		}
		break;

	default:
		rtw_warn(rtwdev, "Invalid size %lu for writing", count);
		return;
	}
#if 0
	enum halmac_ret_status status = HALMAC_RET_SUCCESS;
	enum halmac_sdio_cmd53_4byte_mode cmd53_4byte =
						adapter->sdio_cmd53_4byte;

	if ((adapter->pwr_off_flow_flag == 1 ||
	     adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF ||
	     cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_RW ||
	     cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_R) &&
	    (offset & 0xFFFF0000) == 0) {
		w_indir_sdio_88xx(adapter, offset, value, HALMAC_IO_BYTE);
	} else {
		if ((offset & 0xFFFF0000) == 0)
			offset |= WLAN_IOREG_OFFSET;
		status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
		if (status != HALMAC_RET_SUCCESS) {
			PLTFM_MSG_ERR("[ERR]convert offset\n");
			return status;
		}
		PLTFM_SDIO_CMD52_W(offset, value);
	}
	return HALMAC_RET_SUCCESS;
#endif
#if 0
	enum halmac_ret_status status = HALMAC_RET_SUCCESS;

	if (adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF ||
	    ((offset & (2 - 1)) != 0) ||
	    adapter->sdio_cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_RW ||
	    adapter->sdio_cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_W) {
		if ((offset & 0xFFFF0000) == 0 && ((offset & (2 - 1)) == 0)) {
			status = w_indir_sdio_88xx(adapter, offset, value,
						   HALMAC_IO_WORD);
		} else {
			if ((offset & 0xFFFF0000) == 0)
				offset |= WLAN_IOREG_OFFSET;

			status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
			if (status != HALMAC_RET_SUCCESS) {
				PLTFM_MSG_ERR("[ERR]convert offset\n");
				return status;
			}
			PLTFM_SDIO_CMD52_W(offset, (u8)(value & 0xFF));
			PLTFM_SDIO_CMD52_W(offset + 1,
					   (u8)((value & 0xFF00) >> 8));
		}
	} else {
		if ((offset & 0xFFFF0000) == 0)
			offset |= WLAN_IOREG_OFFSET;

		status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
		if (status != HALMAC_RET_SUCCESS) {
			PLTFM_MSG_ERR("[ERR]convert offset\n");
			return status;
		}

		PLTFM_SDIO_CMD53_W16(offset, value);
	}
	return status;
#endif
#if 0
	if (adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF ||
	    (offset & (4 - 1)) !=  0) {
		if ((offset & 0xFFFF0000) == 0 && ((offset & (4 - 1)) == 0)) {
			status = w_indir_sdio_88xx(adapter, offset, value,
						   HALMAC_IO_DWORD);
		} else {
			if ((offset & 0xFFFF0000) == 0)
				offset |= WLAN_IOREG_OFFSET;

			status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
			if (status != HALMAC_RET_SUCCESS) {
				PLTFM_MSG_ERR("[ERR]convert offset\n");
				return status;
			}
			PLTFM_SDIO_CMD52_W(offset, (u8)(value & 0xFF));
			PLTFM_SDIO_CMD52_W(offset + 1,
					   (u8)((value >> 8) & 0xFF));
			PLTFM_SDIO_CMD52_W(offset + 2,
					   (u8)((value >> 16) & 0xFF));
			PLTFM_SDIO_CMD52_W(offset + 3,
					   (u8)((value >> 24) & 0xFF));
		}
	} else {
		if ((offset & 0xFFFF0000) == 0)
			offset |= WLAN_IOREG_OFFSET;

		status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
		if (status != HALMAC_RET_SUCCESS) {
			PLTFM_MSG_ERR("[ERR]convert offset\n");
			return status;
		}
		PLTFM_SDIO_CMD53_W32(offset, value);
	}
#endif

	sdio_claim_host(rtwsdio->sdio_func);

	if (direct)
		rtw_sdio_write_direct(rtwdev, addr, buf, count, cmd52);
	else
		rtw_sdio_write_indirect(rtwdev, addr, buf, count, cmd52);

	sdio_release_host(rtwsdio->sdio_func);
}

static u8 rtw_sdio_read8(struct rtw_dev *rtwdev, u32 addr)
{
	u8 tmp;

	rtw_sdio_read(rtwdev, addr, &tmp, 1);

	return tmp;
}

static u16 rtw_sdio_read16(struct rtw_dev *rtwdev, u32 addr)
{
	__le16 tmp;
#if 0
	enum halmac_ret_status status = HALMAC_RET_SUCCESS;
	union {
		__le16 word;
		u8 byte[2];
	} value16 = { 0x0000 };

	if ((offset & 0xFFFF0000) == 0 &&
	    adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF) {
		value16.byte[0] = (u8)r_indir_sdio_88xx(adapter, offset,
							HALMAC_IO_BYTE);
		value16.byte[1] = (u8)r_indir_sdio_88xx(adapter, offset + 1,
							HALMAC_IO_BYTE);
		return rtk_le16_to_cpu(value16.word);
	} else if ((offset & 0xFFFF0000) != 0 &&
		   adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF) {
		value16.byte[0] = PLTFM_SDIO_CMD52_R(offset);
		value16.byte[1] = PLTFM_SDIO_CMD52_R(offset + 1);
		return rtk_le16_to_cpu(value16.word);
	}

	if ((offset & 0xFFFF0000) == 0)
		offset |= WLAN_IOREG_OFFSET;

	status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
	if (status != HALMAC_RET_SUCCESS) {
		PLTFM_MSG_ERR("[ERR]convert offset\n");
		return status;
	}

	if (((offset & (2 - 1)) != 0) ||
	    adapter->sdio_cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_RW ||
	    adapter->sdio_cmd53_4byte == HALMAC_SDIO_CMD53_4BYTE_MODE_R) {
		value16.byte[0] = (u8)PLTFM_SDIO_CMD53_R32(offset);
		value16.byte[1] = (u8)PLTFM_SDIO_CMD53_R32(offset + 1);
		return rtk_le16_to_cpu(value16.word);
	}

	return PLTFM_SDIO_CMD53_R16(offset);
#endif

	rtw_sdio_read(rtwdev, addr, &tmp, 2);

	return le16_to_cpu(tmp);
}

static u32 rtw_sdio_read32(struct rtw_dev *rtwdev, u32 addr)
{
	__le32 tmp;
#if 0
	enum halmac_ret_status status = HALMAC_RET_SUCCESS;
	union {
		__le32 dword;
		u8 byte[4];
	} value32 = { 0x00000000 };

	if (((offset & 0xFFFF0000) == 0) &&
	    adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF) {
		value32.byte[0] = (u8)r_indir_sdio_88xx(adapter, offset,
							HALMAC_IO_BYTE);
		value32.byte[1] = (u8)r_indir_sdio_88xx(adapter, offset + 1,
							HALMAC_IO_BYTE);
		value32.byte[2] = (u8)r_indir_sdio_88xx(adapter, offset + 2,
							HALMAC_IO_BYTE);
		value32.byte[3] = (u8)r_indir_sdio_88xx(adapter, offset + 3,
							HALMAC_IO_BYTE);
		return rtk_le32_to_cpu(value32.dword);
	} else if (((offset & 0xFFFF0000) != 0) &&
		   adapter->halmac_state.mac_pwr == HALMAC_MAC_POWER_OFF) {
		value32.byte[0] = PLTFM_SDIO_CMD52_R(offset);
		value32.byte[1] = PLTFM_SDIO_CMD52_R(offset + 1);
		value32.byte[2] = PLTFM_SDIO_CMD52_R(offset + 2);
		value32.byte[3] = PLTFM_SDIO_CMD52_R(offset + 3);
		return rtk_le32_to_cpu(value32.dword);
	}

	if (0 == (offset & 0xFFFF0000))
		offset |= WLAN_IOREG_OFFSET;

	status = cnv_to_sdio_bus_offset_88xx(adapter, &offset);
	if (status != HALMAC_RET_SUCCESS) {
		PLTFM_MSG_ERR("[ERR]convert offset\n");
		return status;
	}

	if ((offset & (4 - 1)) != 0) {
		value32.byte[0] = (u8)PLTFM_SDIO_CMD53_R32(offset);
		value32.byte[1] = (u8)PLTFM_SDIO_CMD53_R32(offset + 1);
		value32.byte[2] = (u8)PLTFM_SDIO_CMD53_R32(offset + 2);
		value32.byte[3] = (u8)PLTFM_SDIO_CMD53_R32(offset + 3);
		return rtk_le32_to_cpu(value32.dword);
	}

	return PLTFM_SDIO_CMD53_R32(offset);
#endif

	rtw_sdio_read(rtwdev, addr, &tmp, 4);

	return le32_to_cpu(tmp);
}

static void rtw_sdio_write8(struct rtw_dev *rtwdev, u32 addr, u8 val)
{
	rtw_sdio_write(rtwdev, addr, &val, 1);
}

static void rtw_sdio_write16(struct rtw_dev *rtwdev, u32 addr, u16 val)
{
	__le16 tmp = cpu_to_le16(val);

	rtw_sdio_write(rtwdev, addr, &tmp, 2);
}

static void rtw_sdio_write32(struct rtw_dev *rtwdev, u32 addr, u32 val)
{
	__le32 tmp = cpu_to_le32(val);

	rtw_sdio_write(rtwdev, addr, &tmp, 4);
}

static size_t rtw_sdio_cmd53_align_size(size_t len)
{
	if (len < RTW_SDIO_BLOCK_SIZE)
		return len;

	return ALIGN(len, RTW_SDIO_BLOCK_SIZE);
}

static u32 rtw_sdio_get_tx_addr(struct rtw_dev *rtwdev, u8 queue)
{
	switch (queue) {
	case RTW_TX_QUEUE_BCN:
	case RTW_TX_QUEUE_H2C:
	case RTW_TX_QUEUE_HI0:
	case RTW_TX_QUEUE_MGMT:
		return FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				  REG_SDIO_CMD_ADDR_TXFF_HIGH);
	case RTW_TX_QUEUE_VI:
	case RTW_TX_QUEUE_VO:
		return FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				  REG_SDIO_CMD_ADDR_TXFF_NORMAL);
	case RTW_TX_QUEUE_BE:
	case RTW_TX_QUEUE_BK:
		return FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				  REG_SDIO_CMD_ADDR_TXFF_LOW);
	default:
		rtw_warn(rtwdev, "Unsupported queue for TX addr: 0x%02x\n",
			 queue);
		return 0;
	}
};

static void rtw_sdio_read_port(struct rtw_dev *rtwdev, u8 *buf, size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	// TODO: CMD53 only so far...
	sdio_memcpy_fromio(rtwsdio->sdio_func, buf,
			   RTW_SDIO_ADDR_RX_RX0FF_GEN(rtwsdio->rx_addr++),
			   count);
}

static int rtw_sdio_write_port(struct rtw_dev *rtwdev, u8 *buf, size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	size_t txsize;
	u32 txaddr;
	int ret;

	txaddr = rtw_sdio_get_tx_addr(rtwdev, RTW_TX_QUEUE_BCN);
	if (!txaddr)
		return -EINVAL;

	txsize = rtw_sdio_cmd53_align_size(ALIGN(count, 4));

	sdio_claim_host(rtwsdio->sdio_func);
	ret = sdio_memcpy_toio(rtwsdio->sdio_func,
			       rtw_sdio_mask_addr(txaddr, false), buf, txsize);
	sdio_release_host(rtwsdio->sdio_func);

	if (ret)
		rtw_warn(rtwdev,
			 "Failed to write %lu byte(s) to SDIO port 0x%08x",
			 txsize, txaddr);

	return ret;
}

static void rtw_sdio_init(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	rtwsdio->irq_mask = REG_SDIO_HIMR_RX_REQUEST | REG_SDIO_HIMR_CPWM1;

	spin_lock_init(&rtwsdio->irq_lock);
}

static void rtw_sdio_enable_interrupt(struct rtw_dev *rtwdev,
				      struct rtw_sdio *rtwsdio)
{
	rtw_write32(rtwdev, REG_SDIO_HIMR, rtwsdio->irq_mask);
}

static void rtw_sdio_disable_interrupt(struct rtw_dev *rtwdev)
{
	rtw_write32(rtwdev, REG_SDIO_HIMR, 0x0);
}

static int rtw_sdio_tx_write(struct rtw_dev *rtwdev,
			     struct rtw_tx_pkt_info *pkt_info,
			     struct sk_buff *skb)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);

	return 0;
}

static void rtw_sdio_tx_kick_off(struct rtw_dev *rtwdev)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);
}

static int rtw_sdio_setup(struct rtw_dev *rtwdev)
{
	/* nothing to do */
	return 0;
}

static int rtw_sdio_start(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	spin_lock_bh(&rtwsdio->irq_lock);
	rtw_sdio_enable_interrupt(rtwdev, rtwsdio);
	spin_unlock_bh(&rtwsdio->irq_lock);

	return 0;
}

static void rtw_sdio_stop(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	spin_lock_bh(&rtwsdio->irq_lock);
	rtw_sdio_disable_interrupt(rtwdev);
	spin_unlock_bh(&rtwsdio->irq_lock);
}

static void rtw_sdio_deep_ps(struct rtw_dev *rtwdev, bool enter)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);
}

static void rtw_sdio_link_ps(struct rtw_dev *rtwdev, bool enter)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);
}

static void rtw_sdio_interface_cfg(struct rtw_dev *rtwdev)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);
}

static void rtw_sdio_power_switch(struct rtw_dev *rtwdev, bool on)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	rtwsdio->is_powered_on = true;
}

static int rtw_sdio_write_data_rsvd_page(struct rtw_dev *rtwdev, u8 *buf,
					 u32 size)
{
	struct rtw_tx_pkt_info pkt_info = {};
	struct sk_buff *skb;
	int ret;

	skb = rtw_tx_write_data_rsvd_page_get(rtwdev, &pkt_info, buf, size);
	if (!skb)
		return -ENOMEM;

	pkt_info.qsel = TX_DESC_QSEL_BEACON;
	rtw_tx_fill_tx_desc(&pkt_info, skb);

	ret = rtw_sdio_write_port(rtwdev, skb->data, skb->len);
	dev_kfree_skb_any(skb);

	return ret;
}

static int rtw_sdio_write_data_h2c(struct rtw_dev *rtwdev, u8 *buf, u32 size)
{
	rtw_dbg(rtwdev, RTW_DBG_SDIO, "%s: not implemented yet", __func__);

	return 0;
}

static void rtw_sdio_tx_err_isr(struct rtw_dev *rtwdev)
{
	u32 val = rtw_read32(rtwdev, REG_TXDMA_STATUS);

	rtw_write32(rtwdev, REG_TXDMA_STATUS, val);
}

static void rtw_sdio_rxfifo_recv(struct rtw_dev *rtwdev, u32 rx_len)
{
	size_t bufsz = rtw_sdio_cmd53_align_size(rx_len);
	struct sk_buff *new;

	new = dev_alloc_skb(bufsz);

	rtw_sdio_read_port(rtwdev, new->data, bufsz);

	// TODO: not complete
}

static void rtw_sdio_rx_isr(struct rtw_dev *rtwdev, u32 rx_len)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	if (!rx_len)
		return;

	sdio_claim_host(rtwsdio->sdio_func);

	do {
		rtw_sdio_rxfifo_recv(rtwdev, rx_len);

		rx_len = rtw_read32(rtwdev, REG_SDIO_RX0_REQ_LEN);
	} while (rx_len);

	sdio_release_host(rtwsdio->sdio_func);

	// TODO: not complete
}

static void rtw_sdio_handle_interrupt(struct sdio_func *sdio_func)
{
	struct ieee80211_hw *hw = sdio_get_drvdata(sdio_func);
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u32 hisr, rx_len;

	spin_lock_bh(&rtwsdio->irq_lock);

	hisr = rtw_read32(rtwdev, REG_SDIO_HISR);
	rx_len = rtw_read32(rtwdev, REG_SDIO_RX0_REQ_LEN);

	rtw_write32(rtwdev, REG_SDIO_HISR, hisr);

	if (hisr & REG_SDIO_HISR_TXERR)
		rtw_sdio_tx_err_isr(rtwdev);
	if (hisr & REG_SDIO_HISR_RX_REQUEST)
		rtw_sdio_rx_isr(rtwdev, rx_len);

	rtw_sdio_enable_interrupt(rtwdev, rtwsdio);

	spin_unlock_bh(&rtwsdio->irq_lock);
}

static int __maybe_unused rtw_sdio_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused rtw_sdio_resume(struct device *dev)
{
	return 0;
}

SIMPLE_DEV_PM_OPS(rtw_sdio_pm_ops, rtw_sdio_suspend, rtw_sdio_resume);

static int rtw_sdio_claim(struct rtw_dev *rtwdev, struct sdio_func *sdio_func)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	int ret;

	sdio_claim_host(sdio_func);

	ret = sdio_enable_func(sdio_func);
	if (ret) {
		rtw_err(rtwdev, "Failed to enable SDIO func");
		goto err_release_host;
	}

	ret = sdio_set_block_size(sdio_func, RTW_SDIO_BLOCK_SIZE);
	if (ret) {
		rtw_err(rtwdev, "Failed to set SDIO block size to 512");
		goto err_disable_func;
	}

	rtwsdio->sdio_func = sdio_func;

	if (sdio_func->card->host->ios.timing >= MMC_TIMING_UHS_SDR12 &&
	    sdio_func->card->host->ios.timing <= MMC_TIMING_UHS_DDR50)
		rtwsdio->sdio3_bus_mode = true;

	sdio_set_drvdata(sdio_func, rtwdev->hw);
	SET_IEEE80211_DEV(rtwdev->hw, &sdio_func->dev);

	sdio_release_host(sdio_func);

	return 0;

err_disable_func:
	sdio_disable_func(sdio_func);
err_release_host:
	sdio_release_host(sdio_func);
	return ret;
}

static void rtw_sdio_declaim(struct rtw_dev *rtwdev, struct sdio_func *sdio_func)
{
	sdio_disable_func(sdio_func);
}

static struct rtw_hci_ops rtw_sdio_ops = {
	.tx_write = rtw_sdio_tx_write,
	.tx_kick_off = rtw_sdio_tx_kick_off,
	.setup = rtw_sdio_setup,
	.start = rtw_sdio_start,
	.stop = rtw_sdio_stop,
	.deep_ps = rtw_sdio_deep_ps,
	.link_ps = rtw_sdio_link_ps,
	.interface_cfg = rtw_sdio_interface_cfg,

	.power_switch = rtw_sdio_power_switch,

	.read8 = rtw_sdio_read8,
	.read16 = rtw_sdio_read16,
	.read32 = rtw_sdio_read32,
	.write8 = rtw_sdio_write8,
	.write16 = rtw_sdio_write16,
	.write32 = rtw_sdio_write32,
	.write_data_rsvd_page = rtw_sdio_write_data_rsvd_page,
	.write_data_h2c = rtw_sdio_write_data_h2c,
};

static int rtw_sdio_request_irq(struct rtw_dev *rtwdev, struct sdio_func *sdio_func)
{
	int ret;

	sdio_claim_host(sdio_func);
	ret = sdio_claim_irq(sdio_func, &rtw_sdio_handle_interrupt);
	sdio_release_host(sdio_func);

	if (ret) {
		rtw_err(rtwdev, "failed to claim SDIO IRQ");
		return ret;
	}

	return 0;
}

static void rtw_sdio_free_irq(struct rtw_dev *rtwdev, struct sdio_func *sdio_func)
{
	sdio_release_irq(sdio_func);
}

static int rtw_sdio_probe(struct sdio_func *sdio_func,
			  const struct sdio_device_id *id)
{
	struct ieee80211_hw *hw;
	struct rtw_dev *rtwdev;
	int drv_data_size;
	int ret;

	drv_data_size = sizeof(struct rtw_dev) + sizeof(struct rtw_sdio);
	hw = ieee80211_alloc_hw(drv_data_size, &rtw_ops);
	if (!hw) {
		dev_err(&sdio_func->dev, "failed to allocate hw");
		return -ENOMEM;
	}

	rtwdev = hw->priv;
	rtwdev->hw = hw;
	rtwdev->dev = &sdio_func->dev;
	rtwdev->chip = (struct rtw_chip_info *)id->driver_data;
	rtwdev->hci.ops = &rtw_sdio_ops;
	rtwdev->hci.type = RTW_HCI_TYPE_SDIO;

	ret = rtw_core_init(rtwdev);
	if (ret)
		goto err_release_hw;

	rtw_dbg(rtwdev, RTW_DBG_SDIO,
		"rtw88 SDIO probe: vendor=0x%04x device=%04x class=%02x",
		id->vendor, id->device, id->class);

	ret = rtw_sdio_claim(rtwdev, sdio_func);
	if (ret) {
		rtw_err(rtwdev, "failed to claim SDIO device");
		goto err_deinit_core;
	}

	rtw_sdio_init(rtwdev);

	ret = rtw_chip_info_setup(rtwdev);
	if (ret) {
		rtw_err(rtwdev, "failed to setup chip information");
		goto err_pci_declaim;
	}

	ret = rtw_register_hw(rtwdev, hw);
	if (ret) {
		rtw_err(rtwdev, "failed to register hw");
		goto err_pci_declaim;
	}

	ret = rtw_sdio_request_irq(rtwdev, sdio_func);
	if (ret)
		goto err_unregister_hw;

	return 0;

err_unregister_hw:
	rtw_unregister_hw(rtwdev, hw);
err_pci_declaim:
	rtw_sdio_declaim(rtwdev, sdio_func);
err_deinit_core:
	rtw_core_deinit(rtwdev);
err_release_hw:
	ieee80211_free_hw(hw);

	return ret;
}

static void rtw_sdio_remove(struct sdio_func *sdio_func)
{
	struct ieee80211_hw *hw = sdio_get_drvdata(sdio_func);
	struct rtw_dev *rtwdev;

	if (!hw)
		return;

	rtwdev = hw->priv;

	rtw_unregister_hw(rtwdev, hw);
	rtw_sdio_disable_interrupt(rtwdev);
	rtw_sdio_declaim(rtwdev, sdio_func);
	rtw_sdio_free_irq(rtwdev, sdio_func);
	rtw_core_deinit(rtwdev);
	ieee80211_free_hw(hw);
}

static void rtw_sdio_shutdown(struct device *dev)
{
	struct sdio_func *sdio_func = dev_to_sdio_func(dev);
	struct ieee80211_hw *hw = sdio_get_drvdata(sdio_func);
	struct rtw_dev *rtwdev;
	struct rtw_chip_info *chip;

	if (!hw)
		return;

	rtwdev = hw->priv;
	chip = rtwdev->chip;

	if (chip->ops->shutdown)
		chip->ops->shutdown(rtwdev);
}

static const struct sdio_device_id rtw_sdio_dev_table[] =  {
#ifdef CONFIG_RTW88_8822C
	{
		.vendor = SDIO_VENDOR_ID_REALTEK,
		.device = SDIO_DEVICE_ID_REALTEK_RTW8822CS,
		.class = SDIO_CLASS_WLAN,
		.driver_data = (kernel_ulong_t)&rtw8822c_hw_spec,
	},
#endif
	{ /* sentinel */ }
};

static struct sdio_driver rtw_sdio_driver = {
	.name       = "RTW-SDIO WLAN",
	.probe      = rtw_sdio_probe,
	.remove     = rtw_sdio_remove,
	.id_table   = rtw_sdio_dev_table,
	.drv = {
		.pm = &rtw_sdio_pm_ops,
		.shutdown   = rtw_sdio_shutdown,
	}
};
module_sdio_driver(rtw_sdio_driver);

MODULE_AUTHOR("Martin Blumenstingl");
MODULE_AUTHOR("Realtek Corporation");
MODULE_DESCRIPTION("Realtek 802.11ac wireless SDIO driver");
MODULE_LICENSE("Dual BSD/GPL");
