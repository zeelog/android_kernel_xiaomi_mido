/*
 * Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MSM_BCL_LEGACY_H
#define __MSM_BCL_LEGACY_H

enum bcl_legacy_dev_type {
	BCL_LEGACY_HIGH_IBAT,
	BCL_LEGACY_LOW_VBAT,
	BCL_LEGACY_SOC_MONITOR,
	BCL_LEGACY_TYPE_MAX,
};

int msm_bcl_legacy_read(enum bcl_legacy_dev_type, int *);

bool msm_bcl_is_legacy(void);

#endif /*__MSM_BCL_LEGACY_H*/
