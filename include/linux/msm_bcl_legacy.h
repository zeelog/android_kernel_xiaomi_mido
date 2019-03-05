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

enum bcl_dev_type {
	BCL_HIGH_IBAT,
	BCL_LOW_VBAT,
	BCL_SOC_MONITOR,
	BCL_TYPE_MAX,
};

int msm_bcl_read(enum bcl_dev_type, int *);

#endif /*__MSM_BCL_LEGACY_H*/
