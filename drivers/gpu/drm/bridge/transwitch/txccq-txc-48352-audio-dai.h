// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

struct txc_48352_audio_dai {
	struct regmap *regmap;
	void (*write_infoframe)(struct regmap *regmap,
				unsigned int tx_pkt_reg, u8 *buf,
				unsigned int len, bool enable);
};
