/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>
#include "pmic.h"

#include <mach/msm_rpcrouter.h>
#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/delay.h>
#define VIBRATOR_DELAY 20
#define VIBRATOR_MIN 50
#endif
#define PM_LIBPROG      0x30000061
#ifndef CONFIG_HUAWEI_KERNEL
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif

#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#else//CONFIG_HUAWEI_KERNEL
#define PM_LIBVERS      0x60001
#define HTC_PROCEDURE_SET_VIB_ON_OFF	22
#endif

/*change PMIC_VIBRATOR_LEVEL to 2.8V */
#define PMIC_VIBRATOR_LEVEL	(2800)

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct hrtimer vibe_timer;

/* define the variable of shake time */
#ifdef CONFIG_HUAWEI_KERNEL
static int time_value = 0;
#endif
#ifndef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_PM8XXX_RPC_VIBRATOR
static void set_pmic_vibrator(int on)
{
	int rc;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

	if (on)
		rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
	else
		rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
}
#endif
#else

static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	int ret=0;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
/* add the variable of shake time */
#ifdef CONFIG_HUAWEI_KERNEL
		uint32_t vib_time;
#endif
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}
	
#ifndef CONFIG_HUAWEI_KERNEL
	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);
#else
/* setting shake time,send to modem*/
	if (on){
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
		req.vib_time = cpu_to_be32(time_value);
	}
	else{
		req.data = cpu_to_be32(0);
		req.vib_time = cpu_to_be32(0);
	}

#endif

	ret = msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);  
    if(ret)
	{
		printk("%s:msm_rpc_call fail,ret=%d\n",__func__,ret);
	}
}
#endif

static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}
#ifndef CONFIG_HUAWEI_KERNEL
static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}
#endif

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}

#ifndef CONFIG_HUAWEI_KERNEL
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);
	
#ifdef CONFIG_HUAWEI_KERNEL
	time_value = value;
#endif

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}
#else
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
    /* detele */

    time_value = value;

    if (value == 0)
    {
		/*solve the problem that we are getting multiple input responses (two) with only one press on the "back" button */
        //mdelay(VIBRATOR_DELAY);
        
        /* here calls the pmic_vibrator_off() directly 
        instead of calling it in timed_vibrator_off() */
        pmic_vibrator_off(NULL);

    }
	else 
    {
        /* make sure the vibrate time not too short or too long */
		value = (value > 15000 ? 15000 : value);
        value = (value < VIBRATOR_MIN ? VIBRATOR_MIN : value);

        /* here calls the pmic_vibrator_on() directly 
        instead of calling it in timed_vibrator_on() */
        pmic_vibrator_on(NULL);

       /* detele */
	}
}
#endif

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

