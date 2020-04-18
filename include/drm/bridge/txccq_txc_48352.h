/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#ifndef __DRM_BRIDGE_TXCCQ_TXC_48352__
#define __DRM_BRIDGE_TXCCQ_TXC_48352__

struct txc_48352;
struct drm_encoder;

struct txc_48352 *txc_48352_bind(struct drm_encoder *encoder,
				 struct device *dev);
void txc_48352_unbind(struct txc_48352 *priv);

#endif /* __DRM_BRIDGE_TXCCQ_TXC_48352__ */
