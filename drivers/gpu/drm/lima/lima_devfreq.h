/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com> */

#ifndef __LIMA_DEVFREQ_H__
#define __LIMA_DEVFREQ_H__

struct lima_device;

int lima_devfreq_init(struct lima_device *ldev);
void lima_devfreq_fini(struct lima_device *ldev);

void lima_devfreq_record_busy(struct lima_device *ldev);
void lima_devfreq_record_idle(struct lima_device *ldev);

#endif
