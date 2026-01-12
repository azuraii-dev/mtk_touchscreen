/******************************************************************************
 * mtk_tpd.c - MTK Android Linux Touch Panel Device Driver               *
 *                                                                            *
 * Copyright 2008-2009 MediaTek Co.,Ltd.                                      *
 *                                                                            *
 * DESCRIPTION:                                                               *
 *     this file provide basic touch panel event to input sub system          *
 *                                                                            *
 * AUTHOR:                                                                    *
 *     Kirby.Wu (mtk02247)                                                    *
 *                                                                            *
 * NOTE:                                                                      *
 * 1. Sensitivity for touch screen should be set to edge-sensitive.           *
 *    But in this driver it is assumed to be done by interrupt core,          *
 *    though not done yet. Interrupt core may provide interface to            *
 *    let drivers set the sensitivity in the future. In this case,            *
 *    this driver should set the sensitivity of the corresponding IRQ         *
 *    line itself.                                                            *
 ******************************************************************************/
#ifndef __HODAFONE_GESTURE_H
#define __HODAFONE_GESTURE_H
#include "tpd.h"
#include <linux/types.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define HODAFONE_GESTURE_FUNC

#define HODAFONE_GESTURE_DEBUG_ON                   0
#if HODAFONE_GESTURE_DEBUG_ON
#define HODAFONE_GESTURE_DEBUG(fmt,arg...)          do{\
					 printk(CRITICAL,"<HODAFONE-GESTURE-DBG>"fmt"\n", ##arg);\
				       }while(0)
#else
#define HODAFONE_GESTURE_DEBUG(fmt,arg...)
#endif
/************************************************************************************
 * 
 *hodafone gesture define
 * 
 * 
 * ***********************************************************************************/

#define KEY_HODAFONE_C 289
#define KEY_HODAFONE_E 290
#define KEY_HODAFONE_W 295
#define KEY_HODAFONE_O 292 
#define KEY_HODAFONE_M 291
#define KEY_HODAFONE_Z 296
#define KEY_HODAFONE_V 294
#define KEY_HODAFONE_STAR  290

#define KEY_HODAFONE_UP    297
#define KEY_HODAFONE_DOWN  298
#define KEY_HODAFONE_LEFT  299
#define KEY_HODAFONE_RIGHT 300
#define KEY_HODAFONE_TWO_LINE  302
#define KEY_HODAFONE_DOUBLE    301
#define KEY_HODAFONE_S 293

#define TPD_HODAFONE_KEY { KEY_HODAFONE_C, KEY_HODAFONE_E, KEY_HODAFONE_W, KEY_HODAFONE_O, KEY_HODAFONE_M, KEY_HODAFONE_Z, KEY_HODAFONE_V, KEY_HODAFONE_STAR, KEY_HODAFONE_UP, KEY_HODAFONE_DOWN, KEY_HODAFONE_LEFT, KEY_HODAFONE_RIGHT, KEY_HODAFONE_TWO_LINE, KEY_HODAFONE_DOUBLE } 

extern void hodafone_gesture_report_key(int report_key);
extern unsigned int hodafone_gesture_init(void);
extern int hodafone_gesture_flag;
#endif /*#define __HODAFONE_GESTURE_H*/
