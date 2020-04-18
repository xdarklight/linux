/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#ifndef __DRM_BRIDGE_ANALOGIX_HDMI_14_TX__
#define __DRM_BRIDGE_ANALOGIX_HDMI_14_TX__

struct anx_hdmi_14_tx;

struct anx_hdmi_14_tx *anx_hdmi_14_tx_probe(struct device *dev);
void anx_hdmi_14_tx_remove(struct anx_hdmi_14_tx *priv);

#endif /* DRM_BRIDGE_ANALOGIX_HDMI_14_TX */
