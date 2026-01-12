/*
 * Copyright (C) 2010 - 2020 Novatek, Inc.
 *
 * $Revision: 63580 $
 * $Date: 2020-06-02 16:56:12 +0800 (週二, 02 六月 2020) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_S2716B_TOUCH_H
#define		_LINUX_S2716B_TOUCH_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define S2716B_DEBUG 0

//---I2C driver info.---
#define S2716B_NAME "s2716b-ts"

#define S2716B_I2C_ADDR   0x67

#define S2716B_SCREEN_MAX_X   720
#define S2716B_SCREEN_MAX_Y   720
#define S2716B_REVERT_X_FLAG   0
#define S2716B_REVERT_Y_FLAG   0
#define S2716B_EXCHANGE_X_Y_FLAG   1

#if S2716B_DEBUG
#define S2716B_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, S2716B_NAME, __func__, __LINE__, ##args)
#else
#define S2716B_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, S2716B_NAME, __func__, __LINE__, ##args)
#endif
#define S2716B_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, S2716B_NAME, __func__, __LINE__, ##args)

#endif /* _LINUX_S2716B_TOUCH_H */
