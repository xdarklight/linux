// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on rtw88/pci.c:
 *   Copyright(c) 2018-2019  Realtek Corporation
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include "sdio.h"
#include "reg.h"
#include "tx.h"
#include "rx.h"
#include "fw.h"
#include "ps.h"
#include "debug.h"

#define RTW_SDIO_INDIRECT_RW_RETRIES		50

static bool rtw_sdio_is_bus_addr(u32 addr)
{
	return (addr & RTW_SDIO_BUS_MSK) != 0;
}

static bool rtw_sdio_bus_claim_needed(struct rtw_sdio *rtwsdio)
{
	return !rtwsdio->irq_thread ||
	       rtwsdio->irq_thread != current;
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

static u32 rtw_sdio_mask_addr(u32 addr)
{
	/* Mask addr to remove driver defined bit and
	 * make sure addr is in valid range
	 */
	return addr & 0x1ffff;
}

static bool rtw_sdio_is_buffer_dma_ready(void *buf)
{
	return virt_addr_valid(buf) && !object_is_on_stack(buf);
}

static u8 rtw_sdio_readb(struct rtw_dev *rtwdev, u32 addr, int *ret)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	addr = rtw_sdio_mask_addr(addr);

	return sdio_readb(rtwsdio->sdio_func, addr, ret);
}

static int rtw_sdio_writeb(struct rtw_dev *rtwdev, u32 addr, u8 data)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	int ret;

	addr = rtw_sdio_mask_addr(addr);

	sdio_writeb(rtwsdio->sdio_func, data, addr, &ret);

	return ret;
}

static int rtw_sdio_read_bytes(struct rtw_dev *rtwdev, u32 addr, void *buf,
			       size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 *byte_buf = buf;
	int i, ret = 0;

	addr = rtw_sdio_mask_addr(addr);

	for (i = 0; i < count; i++) {
		byte_buf[i] = sdio_readb(rtwsdio->sdio_func, addr, &ret);
		if (ret)
			break;
	}

	return ret;
}

static int rtw_sdio_read_block(struct rtw_dev *rtwdev, u32 addr, void *buf,
			       size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	void *ptr = buf;
	bool bounce;
	int ret;

	addr = rtw_sdio_mask_addr(addr);

	bounce = !rtw_sdio_is_buffer_dma_ready(buf);
	if (bounce) {
		ptr = kmalloc(count, GFP_KERNEL);
		if (!ptr)
			return -ENOMEM;
	}

	ret = sdio_memcpy_fromio(rtwsdio->sdio_func, ptr, addr, count);

	if (bounce) {
		memcpy(buf, ptr, count);
		kfree(ptr);
	}

	return ret;
}

static int rtw_sdio_write_bytes(struct rtw_dev *rtwdev, u32 addr, void *buf,
				size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u8 *byte_buf = buf;
	int i, ret = 0;

	addr = rtw_sdio_mask_addr(addr);

	for (i = 0; i < count; i++) {
		sdio_writeb(rtwsdio->sdio_func, byte_buf[i], addr + i, &ret);
		if (ret)
			break;
	}

	return ret;
}

static int rtw_sdio_write_block(struct rtw_dev *rtwdev, u32 addr, void *buf,
				size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	void *ptr = buf;
	bool bounce;
	int ret;

	addr = rtw_sdio_mask_addr(addr);

	bounce = !rtw_sdio_is_buffer_dma_ready(buf);
	if (bounce) {
		ptr = kmemdup(buf, count, GFP_KERNEL);
		if (!ptr)
			return -ENOMEM;
	}

	ret = sdio_memcpy_toio(rtwsdio->sdio_func, addr, ptr, count);

	if (bounce)
		kfree(ptr);

	return ret;
}

static u8 rtw_sdio_read_indirect8(struct rtw_dev *rtwdev, u32 addr, int *ret)
{
	u32 reg_cfg, reg_data;
	int retry;
	u8 tmp;

	reg_cfg = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_CFG);
	reg_data = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_DATA);

	*ret = rtw_sdio_writeb(rtwdev, reg_cfg, addr);
	if (*ret)
		return 0;

	*ret = rtw_sdio_writeb(rtwdev, reg_cfg + 1, addr >> 8);
	if (*ret)
		return 0;

	*ret = rtw_sdio_writeb(rtwdev, reg_cfg + 2, BIT(3));
	if (*ret)
		return 0;

	for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
		tmp = rtw_sdio_readb(rtwdev, reg_cfg + 2, ret);
		if (!ret && tmp & BIT(4))
			break;
	}

	if (*ret)
		return 0;

	return rtw_sdio_readb(rtwdev, reg_data, ret);
}

static u32 rtw_sdio_read_indirect32(struct rtw_dev *rtwdev, u32 addr, int *ret)
{
	u32 reg_cfg, reg_data;
	int retry;
	u32 data;
	u8 tmp;

	reg_cfg = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_CFG);
	reg_data = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_DATA);

	data = addr | BIT(19) | BIT(17);
	*ret = rtw_sdio_write_block(rtwdev, reg_cfg, &data, 4);
	if (*ret)
		return 0;

	for (retry = 0; retry < RTW_SDIO_INDIRECT_RW_RETRIES; retry++) {
		tmp = rtw_sdio_readb(rtwdev, reg_cfg + 2, ret);
		if (!ret && tmp & BIT(4))
			break;
	}

	if (*ret)
		return 0;

	*ret = rtw_sdio_read_block(rtwdev, reg_data, &data, 4);

	return data;
}

static int rtw_sdio_read_indirect(struct rtw_dev *rtwdev, u32 addr, u8 *buf,
				  size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool unaligned;
	size_t i;
	int ret;

	mutex_lock(&rtwsdio->indirect_mutex);

	switch (count) {
	case 1:
		if (!rtwsdio->is_powered_on)
			buf[0] = rtw_sdio_read_indirect8(rtwdev, addr, &ret);
		else
			buf[0] = rtw_sdio_read_indirect32(rtwdev, addr, &ret);
		break;

	case 2:
	case 4:
		unaligned = (addr & (count - 1));
		if (!rtwsdio->is_powered_on) {
			if (unaligned) {
				for (i = 0; i < count; i++) {
					buf[i] = rtw_sdio_read_indirect8(rtwdev, addr + i, &ret);
					if (ret)
						break;
				}
			} else {
				buf[0] = rtw_sdio_read_indirect8(rtwdev, addr, &ret);
				if (ret)
					break;
				addr = rtw_sdio_to_bus_offset(rtwdev, REG_SDIO_INDIRECT_REG_DATA);
				for (i = 1; i < count; i++) {
					buf[i] = rtw_sdio_readb(rtwdev, addr + i, &ret);
					if (ret)
						break;
				}
			}
		} else {
			if (unaligned) {
				for (i = 0; i < count; i++) {
					buf[i] = rtw_sdio_read_indirect32(rtwdev, addr + i, &ret);
					if (ret)
						break;
				}
			} else {
				u32 val;

				val = rtw_sdio_read_indirect32(rtwdev, addr + i,
							       &ret);
				memcpy(buf, &val, count);
			}
		}
		break;

	default:
		rtw_warn(rtwdev, "Invalid size %lu for indirect read", count);
	}

	mutex_unlock(&rtwsdio->indirect_mutex);

	return ret;
}

static int rtw_sdio_read(struct rtw_dev *rtwdev, u32 addr, void *buf,
			 size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool per_byte, direct, bus_claim;
	int ret;

	direct = rtw_sdio_is_bus_addr(addr);
	if (direct) {
		addr = rtw_sdio_to_bus_offset(rtwdev, addr);
		per_byte = (addr & 3) ||
			   (count & 3) ||
			   !rtwsdio->is_powered_on;
	}

	bus_claim = rtw_sdio_bus_claim_needed(rtwsdio);

	if (bus_claim)
		sdio_claim_host(rtwsdio->sdio_func);

	if (!direct)
		ret = rtw_sdio_read_indirect(rtwdev, addr, buf, count);
	else if (per_byte)
		ret = rtw_sdio_read_bytes(rtwdev, addr, buf, count);
	else
		ret = rtw_sdio_read_block(rtwdev, addr, buf, count);

	if (bus_claim)
		sdio_release_host(rtwsdio->sdio_func);

	return ret;
}

static int rtw_sdio_write(struct rtw_dev *rtwdev, u32 addr, void *buf,
			  size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool per_byte, bus_claim;
	int ret;

	if (!rtw_sdio_is_bus_addr(addr))
		addr |= WLAN_IOREG_OFFSET;

	addr = rtw_sdio_to_bus_offset(rtwdev, addr);

	per_byte = (addr & 3) ||
		   (count & 3) ||
		   !rtwsdio->is_powered_on;

	bus_claim = rtw_sdio_bus_claim_needed(rtwsdio);

	if (bus_claim)
		sdio_claim_host(rtwsdio->sdio_func);

	if (per_byte)
		ret = rtw_sdio_write_bytes(rtwdev, addr, buf, count);
	else
		ret = rtw_sdio_write_block(rtwdev, addr, buf, count);

	if (bus_claim)
		sdio_release_host(rtwsdio->sdio_func);

	return ret;
}

static u8 rtw_sdio_read8(struct rtw_dev *rtwdev, u32 addr)
{
	int ret;
	u8 tmp;

	ret = rtw_sdio_read(rtwdev, addr, &tmp, 1);
	if (ret)
		rtw_warn(rtwdev, "sdio read8 failed (0x%x): %d", addr, ret);

	return tmp;
}

static u16 rtw_sdio_read16(struct rtw_dev *rtwdev, u32 addr)
{
	__le16 tmp;
	int ret;

	ret = rtw_sdio_read(rtwdev, addr, &tmp, 2);
	if (ret)
		rtw_warn(rtwdev, "sdio read16 failed (0x%x): %d", addr, ret);

	return le16_to_cpu(tmp);
}

static u32 rtw_sdio_read32(struct rtw_dev *rtwdev, u32 addr)
{
	__le32 tmp;
	int ret;

	ret = rtw_sdio_read(rtwdev, addr, &tmp, 4);
	if (ret)
		rtw_warn(rtwdev, "sdio read32 failed (0x%x): %d", addr, ret);

	return le32_to_cpu(tmp);
}

static void rtw_sdio_write8(struct rtw_dev *rtwdev, u32 addr, u8 val)
{
	int ret;

	ret = rtw_sdio_write(rtwdev, addr, &val, 1);
	if (ret)
		rtw_warn(rtwdev, "sdio write8 failed (0x%x): %d", addr, ret);
}

static void rtw_sdio_write16(struct rtw_dev *rtwdev, u32 addr, u16 val)
{
	__le16 tmp = cpu_to_le16(val);
	int ret;

	ret = rtw_sdio_write(rtwdev, addr, &tmp, 2);
	if (ret)
		rtw_warn(rtwdev, "sdio write16 failed (0x%x): %d", addr, ret);
}

static void rtw_sdio_write32(struct rtw_dev *rtwdev, u32 addr, u32 val)
{
	__le32 tmp = cpu_to_le32(val);
	int ret;

	ret = rtw_sdio_write(rtwdev, addr, &tmp, 4);
	if (ret)
		rtw_warn(rtwdev, "sdio write32 failed (0x%x): %d", addr, ret);
}

static size_t rtw_sdio_cmd53_align_size(size_t len)
{
	len = ALIGN(len, 4);

	if (len < RTW_SDIO_BLOCK_SIZE)
		return len;

	return ALIGN(len, RTW_SDIO_BLOCK_SIZE);
}

static u32 rtw_sdio_get_tx_addr(struct rtw_dev *rtwdev, size_t size, u8 queue)
{
	u32 txaddr;

	switch (queue) {
	case RTW_TX_QUEUE_BCN:
	case RTW_TX_QUEUE_H2C:
	case RTW_TX_QUEUE_HI0:
		txaddr = FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				    REG_SDIO_CMD_ADDR_TXFF_HIGH);
		break;
	case RTW_TX_QUEUE_VI:
	case RTW_TX_QUEUE_VO:
		txaddr = FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				    REG_SDIO_CMD_ADDR_TXFF_NORMAL);
		break;
	case RTW_TX_QUEUE_BE:
	case RTW_TX_QUEUE_BK:
		txaddr = FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				    REG_SDIO_CMD_ADDR_TXFF_LOW);
		break;
	case RTW_TX_QUEUE_MGMT:
		txaddr = FIELD_PREP(REG_SDIO_CMD_ADDR_MSK,
				    REG_SDIO_CMD_ADDR_TXFF_EXTRA);
		break;
	default:
		rtw_warn(rtwdev, "Unsupported queue for TX addr: 0x%02x\n",
			 queue);
		return 0;
	}

	txaddr += DIV_ROUND_UP(size, 4);

	return txaddr;
};

static int rtw_sdio_read_port(struct rtw_dev *rtwdev, u8 *buf, size_t count)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u32 rxaddr = rtwsdio->rx_addr++;
	int ret;

	ret = sdio_memcpy_fromio(rtwsdio->sdio_func, buf,
				 RTW_SDIO_ADDR_RX_RX0FF_GEN(rxaddr), count);
	if (ret)
		rtw_warn(rtwdev,
			 "Failed to read %lu byte(s) from SDIO port 0x%08x",
			 count, rxaddr);

	return ret;
}

static int rtw_sdio_write_port(struct rtw_dev *rtwdev, u8 *buf,
			       size_t count, u8 queue)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	bool bus_claim;
	size_t txsize;
	u32 txaddr;
	void *ptr;
	int ret;

	txaddr = rtw_sdio_get_tx_addr(rtwdev, count, queue);
	if (!txaddr)
		return -EINVAL;

	txsize = rtw_sdio_cmd53_align_size(count);

	ptr = kmalloc(txsize, GFP_KERNEL | GFP_DMA);
	if (!ptr)
		return -ENOMEM;

	memcpy(ptr, buf, count);

	bus_claim = rtw_sdio_bus_claim_needed(rtwsdio);

	if (bus_claim)
		sdio_claim_host(rtwsdio->sdio_func);
	ret = sdio_memcpy_toio(rtwsdio->sdio_func,
			       rtw_sdio_mask_addr(txaddr), ptr, txsize);
	if (bus_claim)
		sdio_release_host(rtwsdio->sdio_func);

	if (ret)
		rtw_warn(rtwdev,
			 "Failed to write %lu byte(s) to SDIO port 0x%08x",
			 txsize, txaddr);

	kfree(ptr);

	return ret;
}

static void rtw_sdio_init(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	rtwsdio->irq_mask = REG_SDIO_HIMR_RX_REQUEST | REG_SDIO_HIMR_CPWM1;

	mutex_init(&rtwsdio->indirect_mutex);
}

static void rtw_sdio_enable_interrupt(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	rtw_write32(rtwdev, REG_SDIO_HIMR, rtwsdio->irq_mask);
}

static void rtw_sdio_disable_interrupt(struct rtw_dev *rtwdev)
{
	rtw_write32(rtwdev, REG_SDIO_HIMR, 0x0);
}

static u8 ac_to_hwq[] = {
	[IEEE80211_AC_VO] = RTW_TX_QUEUE_VO,
	[IEEE80211_AC_VI] = RTW_TX_QUEUE_VI,
	[IEEE80211_AC_BE] = RTW_TX_QUEUE_BE,
	[IEEE80211_AC_BK] = RTW_TX_QUEUE_BK,
};

static_assert(ARRAY_SIZE(ac_to_hwq) == IEEE80211_NUM_ACS);

static u8 rtw_hw_queue_mapping(struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	u8 q_mapping = skb_get_queue_mapping(skb);
	__le16 fc = hdr->frame_control;
	u8 queue;

	if (unlikely(ieee80211_is_beacon(fc)))
		queue = RTW_TX_QUEUE_BCN;
	else if (unlikely(ieee80211_is_mgmt(fc) || ieee80211_is_ctl(fc)))
		queue = RTW_TX_QUEUE_MGMT;
	else if (WARN_ON_ONCE(q_mapping >= ARRAY_SIZE(ac_to_hwq)))
		queue = ac_to_hwq[IEEE80211_AC_BE];
	else
		queue = ac_to_hwq[q_mapping];

	return queue;
}

static u8 rtw_sdio_get_tx_qsel(struct sk_buff *skb, u8 queue)
{
	switch (queue) {
	case RTW_TX_QUEUE_BCN:
		return TX_DESC_QSEL_BEACON;
	case RTW_TX_QUEUE_H2C:
		return TX_DESC_QSEL_H2C;
	case RTW_TX_QUEUE_MGMT:
		return TX_DESC_QSEL_MGMT;
	case RTW_TX_QUEUE_HI0:
		return TX_DESC_QSEL_HIGH;
	default:
		return skb->priority;
	}
};

static void rtw_sdio_tx_kick_off(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	queue_work(rtwsdio->txwq, &rtwsdio->tx_handler_data->work);
}

static int rtw_sdio_setup(struct rtw_dev *rtwdev)
{
	/* nothing to do */
	return 0;
}

static int rtw_sdio_start(struct rtw_dev *rtwdev)
{
	rtw_sdio_enable_interrupt(rtwdev);

	return 0;
}

static void rtw_sdio_stop(struct rtw_dev *rtwdev)
{
	rtw_sdio_disable_interrupt(rtwdev);
}

static void rtw_sdio_deep_ps(struct rtw_dev *rtwdev, bool enter)
{
	/* nothing to do */
}

static void rtw_sdio_link_ps(struct rtw_dev *rtwdev, bool enter)
{
	/* nothing to do */
}

static void rtw_sdio_interface_cfg(struct rtw_dev *rtwdev)
{
	u32 val;

	rtw_read32(rtwdev, REG_SDIO_FREE_TXPG);

	val = rtw_read32(rtwdev, REG_SDIO_FREE_TXPG);
	val &= 0xffff;
	val &= ~(REG_SDIO_TX_ERRSTPINTEN |
		 REG_SDIO_TX_ENMSKTMR |
		 REG_SDIO_TX_ENRXDMAMSKINT);
	rtw_write32(rtwdev, REG_SDIO_FREE_TXPG, val);
}

static void rtw_sdio_power_switch(struct rtw_dev *rtwdev, bool on)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;

	rtwsdio->is_powered_on = true;
}

static int rtw_sdio_write_data(struct rtw_dev *rtwdev,
			       struct rtw_tx_pkt_info *pkt_info,
			       struct sk_buff *skb, u8 queue)
{
	struct rtw_chip_info *chip = rtwdev->chip;
	u8 *pkt_desc;
	int ret;

	pkt_desc = skb_push(skb, chip->tx_pkt_desc_sz);
	memset(pkt_desc, 0, chip->tx_pkt_desc_sz);
	pkt_info->qsel = rtw_sdio_get_tx_qsel(skb, queue);
	rtw_tx_fill_tx_desc(pkt_info, skb);
	fill_txdesc_checksum_common(skb->data);

	ret = rtw_sdio_write_port(rtwdev, skb->data, skb->len, queue);
	dev_kfree_skb_any(skb);

	return ret;
}

static int rtw_sdio_write_data_rsvd_page(struct rtw_dev *rtwdev, u8 *buf,
					 u32 size)
{
	struct rtw_tx_pkt_info pkt_info = {};
	struct sk_buff *skb;

	skb = rtw_tx_write_data_rsvd_page_get(rtwdev, &pkt_info, buf, size);
	if (!skb)
		return -ENOMEM;

	return rtw_sdio_write_data(rtwdev, &pkt_info, skb, RTW_TX_QUEUE_BCN);
}

static int rtw_sdio_write_data_h2c(struct rtw_dev *rtwdev, u8 *buf, u32 size)
{
	struct rtw_tx_pkt_info pkt_info = {};
	struct sk_buff *skb;

	skb = rtw_tx_write_data_h2c_get(rtwdev, &pkt_info, buf, size);
	if (!skb)
		return -ENOMEM;

	return rtw_sdio_write_data(rtwdev, &pkt_info, skb, RTW_TX_QUEUE_H2C);
}

static int rtw_sdio_tx_write(struct rtw_dev *rtwdev,
			     struct rtw_tx_pkt_info *pkt_info,
			     struct sk_buff *skb)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	struct rtw_chip_info *chip = rtwdev->chip;
	u8 queue = rtw_hw_queue_mapping(skb);
	u8 *pkt_desc;

	pkt_desc = skb_push(skb, chip->tx_pkt_desc_sz);
	memset(pkt_desc, 0, chip->tx_pkt_desc_sz);
	pkt_info->qsel = rtw_sdio_get_tx_qsel(skb, queue);
	rtw_tx_fill_tx_desc(pkt_info, skb);
	fill_txdesc_checksum_common(skb->data);

	skb_queue_tail(&rtwsdio->tx_queue[queue], skb);

	return 0;
}

static void rtw_sdio_tx_err_isr(struct rtw_dev *rtwdev)
{
	u32 val = rtw_read32(rtwdev, REG_TXDMA_STATUS);

	rtw_write32(rtwdev, REG_TXDMA_STATUS, val);
}

static void rtw_sdio_rxfifo_recv(struct rtw_dev *rtwdev, u32 rx_len)
{
	struct rtw_chip_info *chip = rtwdev->chip;
	u32 pkt_desc_sz = chip->rx_pkt_desc_sz;
	struct rtw_rx_pkt_stat pkt_stat;
	struct sk_buff *skb;
	u32 pkt_offset;
	size_t bufsz;
	u8 *rx_desc;
	int ret;

	bufsz = rtw_sdio_cmd53_align_size(rx_len);

	skb = dev_alloc_skb(bufsz);
	if (!skb)
		return;

	ret = rtw_sdio_read_port(rtwdev, skb->data, bufsz);
	if (ret) {
		dev_kfree_skb_any(skb);
		return;
	}

	rx_desc = skb->data;
	chip->ops->query_rx_desc(rtwdev, rx_desc, &pkt_stat,
				 (struct ieee80211_rx_status *)skb->cb);
	pkt_offset = pkt_desc_sz + pkt_stat.drv_info_sz +
		     pkt_stat.shift;

	if (pkt_stat.is_c2h) {
		skb_put(skb, pkt_stat.pkt_len + pkt_offset);
		rtw_fw_c2h_cmd_rx_irqsafe(rtwdev, pkt_offset, skb);
		return;
	}

	skb_put(skb, pkt_stat.pkt_len);
	skb_reserve(skb, pkt_offset);
	ieee80211_rx_irqsafe(rtwdev->hw, skb);
}

static void rtw_sdio_rx_isr(struct rtw_dev *rtwdev)
{
	u32 rx_len;

	while (true) {
		if (rtw_chip_wcpu_11n(rtwdev))
			rx_len = rtw_read16(rtwdev, REG_SDIO_RX0_REQ_LEN);
		else
			rx_len = rtw_read32(rtwdev, REG_SDIO_RX0_REQ_LEN);

		if (!rx_len)
			break;

		rtw_sdio_rxfifo_recv(rtwdev, rx_len);
	}
}

static void rtw_sdio_handle_interrupt(struct sdio_func *sdio_func)
{
	struct ieee80211_hw *hw = sdio_get_drvdata(sdio_func);
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	u32 hisr;

	rtwsdio->irq_thread = current;

	hisr = rtw_read32(rtwdev, REG_SDIO_HISR);

	if (hisr & REG_SDIO_HISR_TXERR)
		rtw_sdio_tx_err_isr(rtwdev);
	if (hisr & REG_SDIO_HISR_RX_REQUEST) {
		hisr &= ~REG_SDIO_HISR_RX_REQUEST;
		rtw_sdio_rx_isr(rtwdev);
	}

	rtw_write32(rtwdev, REG_SDIO_HISR, hisr);

	rtwsdio->irq_thread = NULL;
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
EXPORT_SYMBOL(rtw_sdio_pm_ops);

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

static void rtw_sdio_declaim(struct rtw_dev *rtwdev,
			     struct sdio_func *sdio_func)
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

static int rtw_sdio_request_irq(struct rtw_dev *rtwdev,
				struct sdio_func *sdio_func)
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

static void rtw_sdio_tx_handler(struct work_struct *work)
{
	struct rtw_sdio_work_data *work_data =
		container_of(work, struct rtw_sdio_work_data, work);
	struct rtw_dev *rtwdev = work_data->rtwdev;
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	struct sk_buff *skb;
	int index, limit;

	for (index = RTK_MAX_TX_QUEUE_NUM - 1; index >= 0; index--) {
		for (limit = 0; limit < 200; limit++) {
			skb = skb_dequeue(&rtwsdio->tx_queue[index]);
			if (!skb)
				break;

			rtw_sdio_write_port(rtwdev, skb->data, skb->len, index);
			dev_kfree_skb_any(skb);
		}
	}
}

static void rtw_sdio_free_irq(struct rtw_dev *rtwdev,
			      struct sdio_func *sdio_func)
{
	sdio_release_irq(sdio_func);
}

static int rtw_sdio_init_tx(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	int i;

	rtwsdio->txwq = create_singlethread_workqueue("rtw88_sdio: tx wq");
	if (!rtwsdio->txwq) {
		rtw_err(rtwdev, "failed to create TX work queue\n");
		return -ENOMEM;
	}

	for (i = 0; i < RTK_MAX_TX_QUEUE_NUM; i++)
		skb_queue_head_init(&rtwsdio->tx_queue[i]);
	rtwsdio->tx_handler_data = kmalloc(sizeof(*rtwsdio->tx_handler_data),
					   GFP_KERNEL);
	if (!rtwsdio->tx_handler_data)
		goto err_destroy_wq;

	rtwsdio->tx_handler_data->rtwdev = rtwdev;
	INIT_WORK(&rtwsdio->tx_handler_data->work, rtw_sdio_tx_handler);

	return 0;

err_destroy_wq:
	destroy_workqueue(rtwsdio->txwq);
	return -ENOMEM;
}

static void rtw_sdio_deinit_tx(struct rtw_dev *rtwdev)
{
	struct rtw_sdio *rtwsdio = (struct rtw_sdio *)rtwdev->priv;
	int i;

	for (i = 0; i < RTK_MAX_TX_QUEUE_NUM; i++)
		skb_queue_purge(&rtwsdio->tx_queue[i]);

	flush_workqueue(rtwsdio->txwq);
	destroy_workqueue(rtwsdio->txwq);
	kfree(rtwsdio->tx_handler_data);
}

int rtw_sdio_probe(struct sdio_func *sdio_func,
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

	ret = rtw_sdio_init_tx(rtwdev);
	if (ret) {
		rtw_err(rtwdev, "failed to init SDIO TX queue\n");
		goto err_sdio_declaim;
	}

	ret = rtw_chip_info_setup(rtwdev);
	if (ret) {
		rtw_err(rtwdev, "failed to setup chip information");
		goto err_destroy_txwq;
	}

	ret = rtw_register_hw(rtwdev, hw);
	if (ret) {
		rtw_err(rtwdev, "failed to register hw");
		goto err_destroy_txwq;
	}

	ret = rtw_sdio_request_irq(rtwdev, sdio_func);
	if (ret)
		goto err_unregister_hw;

	return 0;

err_unregister_hw:
	rtw_unregister_hw(rtwdev, hw);
err_destroy_txwq:
	rtw_sdio_deinit_tx(rtwdev);
err_sdio_declaim:
	rtw_sdio_declaim(rtwdev, sdio_func);
err_deinit_core:
	rtw_core_deinit(rtwdev);
err_release_hw:
	ieee80211_free_hw(hw);

	return ret;
}
EXPORT_SYMBOL(rtw_sdio_probe);

void rtw_sdio_remove(struct sdio_func *sdio_func)
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
	rtw_sdio_deinit_tx(rtwdev);
	rtw_core_deinit(rtwdev);
	ieee80211_free_hw(hw);
}
EXPORT_SYMBOL(rtw_sdio_remove);

void rtw_sdio_shutdown(struct device *dev)
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
EXPORT_SYMBOL(rtw_sdio_shutdown);

MODULE_AUTHOR("Martin Blumenstingl");
MODULE_AUTHOR("Jernej Skrabec");
MODULE_DESCRIPTION("Realtek 802.11ac wireless SDIO driver");
MODULE_LICENSE("Dual BSD/GPL");
