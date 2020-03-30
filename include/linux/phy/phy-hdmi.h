/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

/**
 * struct phy_configure_opts_hdmi - HDMI PHY configuration set
 *
 * This structure is used to represent the configuration state of a
 * HDMI PHY.
 */
struct phy_configure_opts_hdmi {
	/**
	 * @pixel_clock:
	 *
	 * The pixel clock in kHz
	 */
	unsigned int pixel_clock;
};
