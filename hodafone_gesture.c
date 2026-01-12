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
#include "hodafone_gesture.h"

/************************************************************************************
 * 
 * hodafone gesture define
 * 
 * 
 * ***********************************************************************************/
 int hodafone_gesture_flag = 0;
static struct input_dev *hodafone_power_idev;
static struct proc_dir_entry *gesture_proc_entry;

static unsigned int g_tpd_hodafone_keys[] = TPD_HODAFONE_KEY;
static unsigned int g_hodafone_key_size = sizeof(g_tpd_hodafone_keys) / sizeof(g_tpd_hodafone_keys[0]);
static int gesture_debug_read(struct seq_file *m, void *v);
static ssize_t gesture_debug_write(struct file *filp, const char  *buff, size_t len, loff_t *data);
static int hodafone_gesture_open(struct inode *inode, struct file *file);

static const struct proc_ops config_proc_ops = {
	.proc_open  = hodafone_gesture_open,
	.proc_read  = seq_read,
    .proc_write = gesture_debug_write,
    .proc_release = single_release,
};

static void hodafone_request_gesture_idev(void)
{
	struct input_dev *idev;
	int rc = 0;
    int i = 0;
	idev = input_allocate_device();
	if(!idev){
		return;
	}
	hodafone_power_idev = idev;
	idev->name = "hodafone_gesture";
	idev->id.bustype = BUS_I2C;
	input_set_capability(idev,EV_KEY,KEY_POWER);
    for(i=0; i<g_hodafone_key_size; i++)
    {
        input_set_capability(idev, EV_KEY, g_tpd_hodafone_keys[i]);
    }
	input_set_capability(idev,EV_KEY,KEY_END);

	rc = input_register_device(idev);
	if(rc){
		input_free_device(idev);
		hodafone_power_idev = NULL;
	}
}
unsigned int hodafone_gesture_init(void)
{
	hodafone_request_gesture_idev();
    
    gesture_proc_entry = proc_create("gesturectrl", 0777, NULL, &config_proc_ops);
    if (NULL == gesture_proc_entry) 
    {
        HODAFONE_GESTURE_DEBUG("proc_create gesture_proc_entry error:\n");
    }
    else
    {
        HODAFONE_GESTURE_DEBUG("proc_create gesture_proc_entry    Succesfull*****\n");
    }

    HODAFONE_GESTURE_DEBUG("[FTS_GESTRUE] hodafone_gesture_init success.\n");
    
	return 1;
}

static int hodafone_gesture_open(struct inode *inode, struct file *file)
{
	return single_open(file, gesture_debug_read, NULL);
};

static int gesture_debug_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", hodafone_gesture_flag);
	return 0;
}

static ssize_t gesture_debug_write(struct file *filp, const char  *buff, size_t len, loff_t *data)
{
	unsigned char writebuf[50];	
	if (copy_from_user(&writebuf, buff, len)) {
		return -EFAULT;
	}

    HODAFONE_GESTURE_DEBUG("HodafoneLog gesture_debug_write writebuf=%s",writebuf);
   	if(writebuf[0] == '0'){
		hodafone_gesture_flag = 0;
	}else if(writebuf[0] == '1'){
		hodafone_gesture_flag = 1;
	} 
	return len;
}

void hodafone_gesture_report_key(int report_key)
{
    input_report_key(hodafone_power_idev, report_key, 1);
    input_sync(hodafone_power_idev);
    input_report_key(hodafone_power_idev, report_key, 0);
    input_sync(hodafone_power_idev);
}

