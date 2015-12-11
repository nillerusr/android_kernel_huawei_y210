
/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov5647.h"

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif


#ifdef CDBG
#undef CDBG
#endif
#define CDBG(fmt, args...) printk(KERN_ERR "ov5647.c: " fmt, ## args)

#define OV5647_REG_MODEL_ID           0x300A
#define OV5647_MODEL_ID               0x5647

#define REG_OV5647_GAIN_MSB           0x350A
#define REG_OV5647_GAIN_LSB           0x350B
#define REG_OV5647_LINE_HSB           0x3500
#define REG_OV5647_LINE_MSB           0x3501
#define REG_OV5647_LINE_LSB           0x3502
#define OV5647_MASTER_CLK_RATE        24000000




/* AF Total steps parameters */
#define OV5647_TOTAL_STEPS_NEAR_TO_FAR	32 


static uint16_t prev_line_length_pck;
static uint16_t prev_frame_length_lines;
static uint16_t snap_line_length_pck;
static uint16_t snap_frame_length_lines;




static int CSI_CONFIG = 0;

static uint8_t  mode_mask = 0x09;

static struct ov5647_ctrl *ov5647_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(ov5647_wait_queue);

DEFINE_MUTEX(ov5647_mut);

static DECLARE_WAIT_QUEUE_HEAD(ov5647_af_wait_queue);

enum ov5647_test_mode
{
    TEST_OFF,
    TEST_1,
    TEST_2,
    TEST_3
};

enum ov5647_resolution
{
    QTR_SIZE,
    FULL_SIZE,
    INVALID_SIZE
};

enum ov5647_reg_update
{
    /* Sensor egisters that need to be updated during initialization */
    REG_INIT,

    /* Sensor egisters that needs periodic I2C writes */
    UPDATE_PERIODIC,

    /* All the sensor Registers will be updated */
    UPDATE_ALL,

    /* Not valid update */
    UPDATE_INVALID
};

enum ov5647_setting
{
    RES_PREVIEW,
    RES_CAPTURE
};


struct ov5647_work
{
    struct work_struct work;
};
static struct ov5647_work *ov5647_sensorw;
static struct ov5647_work *ov5647_af_sensorw;
static struct i2c_client *ov5647_af_client;
static struct i2c_client *ov5647_client;



struct ov5647_ctrl
{
    const struct msm_camera_sensor_info *sensordata;

    int      sensormode;
    uint32_t fps_divider;   /* init to 1 * 0x00000400 */
    uint32_t pict_fps_divider;  /* init to 1 * 0x00000400 */

    uint16_t curr_lens_pos;
    uint16_t curr_step_pos;
    uint16_t init_curr_lens_pos;
    uint16_t my_reg_gain;
    uint32_t my_reg_line_count;

    enum ov5647_resolution prev_res;
    enum ov5647_resolution pict_res;
    enum ov5647_resolution curr_res;
    enum ov5647_test_mode  set_test;
};


struct register_address_value_pair const ov5647_init_reg_settings[] =
{
    {0x5000, 0x06},
    {0x5003, 0x08},
    {0x5a00, 0x08},
    {0x3000, 0xff},
    {0x3001, 0xff},
    {0x3002, 0xff},
    {0x301d, 0xf0},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3c01, 0x80},
    {0x3b07, 0x0c},

	/*analog control*/
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3634, 0x44},
    {0x3620, 0x64},
    {0x3621, 0xe0},
    {0x3600, 0x37},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x3731, 0x02},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3f05, 0x02},
    {0x3f06, 0x10},
    {0x3f01, 0x0a},
	/*AGAE target*/
    {0x3a0f, 0x58},
    {0x3a10, 0x50},
    {0x3a1b, 0x58},
    {0x3a1e, 0x50},
    {0x3a11, 0x60},
    {0x3a1f, 0x28},
    {0x4001, 0x02},
    {0x4000, 0x09},
    {0x3000, 0x00},
    {0x3001, 0x00},
    {0x3002, 0x00},
    {0x3017, 0xe0},
    {0x301c, 0xfc},
    {0x3636, 0x06},
    {0x3016, 0x08},
    {0x3827, 0xec},
		
    {0x3018, 0x44},//MIPI pad enable
    {0x3035, 0x21},//PLL
    {0x3106, 0xf5},//PLL
    {0x3034, 0x1a},//PLL
    {0x301c, 0xf8},
		
    /*lens setting*/
    {0x5000, 0x86},
	{0x5800, 0xc },
	{0x5801, 0x9 },
	{0x5802, 0x7 },
	{0x5803, 0x8 },
	{0x5804, 0x9 },
	{0x5805, 0x12},
	{0x5806, 0x6 },
	{0x5807, 0x4 },
	{0x5808, 0x3 },
	{0x5809, 0x3 },
	{0x580a, 0x4 },
	{0x580b, 0x7 },
	{0x580c, 0x4 },
	{0x580d, 0x1 },
	{0x580e, 0x0 },
	{0x580f, 0x0 },
	{0x5810, 0x2 },
	{0x5811, 0x5 },
	{0x5812, 0x4 },
	{0x5813, 0x1 },
	{0x5814, 0x0 },
	{0x5815, 0x0 },
	{0x5816, 0x2 },
	{0x5817, 0x5 },
	{0x5818, 0x6 },
	{0x5819, 0x4 },
	{0x581a, 0x3 },
	{0x581b, 0x3 },
	{0x581c, 0x4 },
	{0x581d, 0x7 },
	{0x581e, 0xd },
	{0x581f, 0x9 },
	{0x5820, 0x8 },
	{0x5821, 0x8 },
	{0x5822, 0x9 },
	{0x5823, 0x12},
	{0x5824, 0x28},
	{0x5825, 0x2c},
	{0x5826, 0xc },
	{0x5827, 0x2a},
	{0x5828, 0x26},
	{0x5829, 0x2a},
	{0x582a, 0x46},
	{0x582b, 0x44},
	{0x582c, 0x46},
	{0x582d, 0x28},
	{0x582e, 0x28},
	{0x582f, 0x62},
	{0x5830, 0x60},
	{0x5831, 0x42},
	{0x5832, 0x28},
	{0x5833, 0x2a},
	{0x5834, 0x46},
	{0x5835, 0x44},
	{0x5836, 0x46},
	{0x5837, 0x48},
	{0x5838, 0x26},
	{0x5839, 0x2a},
	{0x583a, 0x2a},
	{0x583b, 0x2a},
	{0x583c, 0x26},
	{0x583d, 0xbe},

	/* manual AWB,manual AE,close Lenc,open WBC*/
    {0x3503, 0x03},/*manual AE*/
    {0x3501, 0x10},
    {0x3502, 0x80},
    {0x350a, 0x00},
    {0x350b, 0x7f},
    {0x5001, 0x01},/*manual AWB*/
    {0x5180, 0x08},
    {0x5186, 0x04},
    {0x5187, 0x00},
    {0x5188, 0x04},
    {0x5189, 0x00},
    {0x518a, 0x04},
    {0x518b, 0x00},
    {0x5000, 0x06},/*No lenc,WBC on*/
};

struct register_address_value_pair const ov5647_preview[] =
{

	/*1296*972 Reference Setting 24M MCLK 2lane 280Mbps/lane 30fps
	for back to preview*/
	{0x0100, 0x00},
	{0x3035, 0x21},//PLL   0x21 30fps   0x41 15fps
	{0x3036, 0x66},//PLL,0x64
	{0x303c, 0x11},//PLL   

	{0x3821,0x00},
    {0x3820,0x47},
    {0x4510,0x03},
    {0x4520,0x0b},


    {0x3612, 0x59}, 
    {0x3618, 0x00}, 
    {0x380c, 0x09}, 
    {0x380d, 0x70}, 
    {0x380e, 0x04}, 
    {0x380f, 0x66}, 
    {0x3814, 0x31}, 
    {0x3815, 0x31}, 
    {0x3708, 0x64}, 
    {0x3709, 0x52}, 
    {0x3808, 0x05}, //X OUTPUT SIZE = 1296
    {0x3809, 0x10}, 
    {0x380a, 0x03}, //Y OUTPUT SIZE = 972
    {0x380b, 0xcc}, 
    {0x3800, 0x00}, 
    {0x3801, 0x08}, 
    {0x3802, 0x00}, 
    {0x3803, 0x00}, 
    {0x3804, 0x0a}, 
    {0x3805, 0x37}, 
    {0x3806, 0x07}, 
    {0x3807, 0xa7}, 
	/*banding filter*/	
    {0x3a08, 0x01}, 
    {0x3a09, 0x4b}, 
    {0x3a0a, 0x01}, 
    {0x3a0b, 0x13}, 
    {0x3a0d, 0x04}, 
    {0x3a0e, 0x03}, 
    {0x4004, 0x02},
    {0x4837, 0x19}, 
    {0x0100, 0x01}, 
};


struct register_address_value_pair const ov5647_snapshot[] =
{
	{0x0100, 0x00},
    {0x3035, 0x21},
    {0x3036, 0x66},
    {0x303c, 0x11},
		
   	{0x3821,0x00},
    {0x3820,0x47},
		
    {0x3612, 0x5b},
    {0x3618, 0x04},
    {0x380c, 0x0a},
    {0x380d, 0xc0},
    {0x380e, 0x07},
    {0x380f, 0xb6},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3708, 0x64},
    {0x3709, 0x12},

#if 1
    /*2608 * 1952 */
	{0x3808, 0x0a}, //X OUTPUT SIZE = 2608
    {0x3809, 0x30},
    {0x380a, 0x07}, //Y OUTPUT SIZE = 1952
    {0x380b, 0xa0},
    {0x3800, 0x00},
    {0x3801, 0x04},
    {0x3802, 0x00},
    {0x3803, 0x00},
    {0x3804, 0x0a},
    {0x3805, 0x3b},
    {0x3806, 0x07},
    {0x3807, 0xa3},
#else
   /*2592 * 1944 */
    {0x3808, 0x0a}, //2612
    {0x3809, 0x20},
    {0x380a, 0x07},
    {0x380b, 0x98},
    {0x3800, 0x00},
    {0x3801, 0x0c},
    {0x3802, 0x00},
    {0x3803, 0x02},
    {0x3804, 0x0a},
    {0x3805, 0x33},
    {0x3806, 0x07},
    {0x3807, 0xa1},
#endif
	/*banding filter*/	
    {0x3a08, 0x01},
    {0x3a09, 0x28},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0d, 0x07},
    {0x3a0e, 0x06},
    {0x4004, 0x04},
    {0x4837, 0x19},
    {0x0100, 0x01},
};


struct ov5647_reg ov5647_regs =
{
    .init_reg_settings      = &ov5647_init_reg_settings[0],
    .init_reg_settings_size = ARRAY_SIZE(ov5647_init_reg_settings),
    
    .prev_reg_settings		= &ov5647_preview[0],
    .prev_reg_settings_size = ARRAY_SIZE( ov5647_preview),
    
    .snap_reg_settings		= &ov5647_snapshot[0],
    .snap_reg_settings_size = ARRAY_SIZE(ov5647_snapshot),
};


/*=============================================================*/

static int ov5647_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
                                   int length)
{
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = 2,
            .buf = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    if (i2c_transfer(ov5647_client->adapter, msgs, 2) < 0)
    {
        CDBG("ov5647_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}

static int32_t ov5647_i2c_read_w(unsigned short saddr, unsigned short raddr,
                                       unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00) >> 8;
    buf[1] = (raddr & 0x00FF);

    rc = ov5647_i2c_rxdata(saddr, buf, 2);
    if (rc < 0)
    {
        return rc;
    }

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
    {
        CDBG("ov5647_i2c_read failed!\n");
    }

    return rc;
}

static int32_t ov5647_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length)
{
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    if (i2c_transfer(ov5647_client->adapter, msg, 1) < 0)
    {
        CDBG("ov5647_i2c_txdata failed\n");
        return -EIO;
    }

    return 0;
}

static int32_t ov5647_i2c_write(unsigned short saddr, unsigned short waddr, unsigned short wdata)
{
    int32_t rc = -EIO;
    unsigned char buf[3];

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = wdata;

    rc = ov5647_i2c_txdata(saddr, buf, 3);

    if (rc < 0)
    {
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
    }

    return rc;
}

static int32_t ov5647_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	return ov5647_i2c_write(ov5647_client->addr, waddr, bdata);
}

static int32_t ov5647_i2c_write_table(struct register_address_value_pair const *reg_conf_tbl, int num)
{
    int i;
    int32_t rc = 0;

    for (i = 0; i < num; i++)
    {
        rc = ov5647_i2c_write(ov5647_client->addr,
                                    reg_conf_tbl->register_address,
                                    reg_conf_tbl->register_value);
        if (rc < 0)
        {
            break;
        }

        reg_conf_tbl++;
    }

    return rc;
}



static int32_t ov5647_lens_shading_enable(uint8_t is_enable)
{
    int32_t rc = 0;

    return rc;
}

static int32_t ov5647_af_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(ov5647_af_client->adapter, msg, 1) < 0) {
		pr_err("ov5647_af_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}


static int32_t ov5647_af_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;
	CDBG("ov5647_af_i2c_write_b_sensor: addr = 0x%x, val = 0x%x, ov5647_af_client->addr = %0x\n",
		waddr, bdata, ov5647_af_client->addr);
	rc = ov5647_af_i2c_txdata(ov5647_af_client->addr >> 1, buf, 2);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
				waddr, bdata);
	}
	return rc;
}

static void ov5647_start_stream(void)
{
	ov5647_i2c_write(ov5647_client->addr, 0x4202, 0x00);/* streaming on */
}

static void ov5647_stop_stream(void)
{
	ov5647_i2c_write(ov5647_client->addr, 0x4202, 0x0f);/* streaming off */
}

static void ov5647_group_hold_on(void)
{
	ov5647_i2c_write_b_sensor(0x0104, 0x01);
}

static void ov5647_group_hold_off(void)
{
	ov5647_i2c_write_b_sensor(0x0104, 0x0);
}

static void ov5647_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
    /* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;
	uint32_t preview_pclk = 0x37, snapshot_pclk = 0x4f;

	d1 = (prev_frame_length_lines * 0x00000400) / snap_frame_length_lines;
	d2 = (prev_line_length_pck * 0x00000400) / snap_line_length_pck;
	divider = (d1 * d2*preview_pclk/snapshot_pclk) / 0x400;
	CDBG(KERN_ERR "ov5647_get_pict_fps divider = %d", divider);
	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
}

static uint16_t ov5647_get_prev_lines_pf(void)
{
    if (ov5647_ctrl->prev_res == QTR_SIZE)
		return prev_frame_length_lines;
	else
		return snap_frame_length_lines;
}

static uint16_t ov5647_get_prev_pixels_pl(void)
{
   if (ov5647_ctrl->prev_res == QTR_SIZE)
		return prev_line_length_pck;
	else
		return snap_line_length_pck;
}

static uint16_t ov5647_get_pict_lines_pf(void)
{
    if (ov5647_ctrl->pict_res == QTR_SIZE)
		return prev_frame_length_lines;
	else
		return snap_frame_length_lines;
}

static uint16_t ov5647_get_pict_pixels_pl(void)
{
    if (ov5647_ctrl->pict_res == QTR_SIZE)
		return prev_line_length_pck;
	else
		return snap_line_length_pck;
}

static uint32_t ov5647_get_pict_max_exp_lc(void)
{
	return snap_frame_length_lines * 24;
}

static int32_t ov5647_set_fps(struct fps_cfg *fps)
{
    uint16_t total_lines_per_frame = 0;
	int32_t  rc = 0;

	CDBG("%s: fps_divider = 0x%x, pict_fps_divider = 0x%x\n",__func__, fps->fps_div, fps->pict_fps_div);
	
	ov5647_ctrl->fps_divider = fps->fps_div;
	ov5647_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (ov5647_ctrl->sensormode == SENSOR_PREVIEW_MODE) 
	{
		total_lines_per_frame = (uint16_t)
		((prev_frame_length_lines * ov5647_ctrl->fps_divider) / 0x400);
	} 
	else 
	{
		total_lines_per_frame = (uint16_t)
		((snap_frame_length_lines * ov5647_ctrl->fps_divider) / 0x400);
	}

	ov5647_group_hold_on();
	
	rc = ov5647_i2c_write_b_sensor(0x0340, ((total_lines_per_frame & 0xFF00) >> 8));
	
	rc = ov5647_i2c_write_b_sensor(0x0341, (total_lines_per_frame & 0x00FF));
	
	ov5647_group_hold_off();

	return rc;;
}

static int32_t ov5647_write_exp_gain(uint16_t gain, uint32_t line)
{


	int rc = 0;
	uint16_t max_line = 0;
	uint8_t intg_time_hsb, intg_time_msb, intg_time_lsb;
	uint8_t gain_lsb, gain_hsb;
	ov5647_ctrl->my_reg_gain = gain;
	ov5647_ctrl->my_reg_line_count = (uint16_t)line;

	//CDBG(KERN_ERR "preview exposure setting 0x%x, 0x%x, %d",
	//	 gain, line, line);

	gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
	gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain & 0x300)>>8);
	/* adjust frame rate */
	if (line > prev_frame_length_lines - 4) {
		rc = ov5647_i2c_write_b_sensor(0x380E,
			 (uint8_t)((line+4) >> 8)) ;
		rc = ov5647_i2c_write_b_sensor(0x380F,
			 (uint8_t)((line+4) & 0x00FF)) ;
		max_line = line + 4;
	} else if (max_line > prev_frame_length_lines) {
		rc = ov5647_i2c_write_b_sensor(0x380E,
			 (uint8_t)(prev_frame_length_lines >> 8)) ;
		rc = ov5647_i2c_write_b_sensor(0x380F,
			 (uint8_t)(prev_frame_length_lines & 0x00FF)) ;
		max_line = 1104;
	}

	line = line<<4;
	/* ov5647 need this operation */
	intg_time_hsb = (u8)(line>>16);
	intg_time_msb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line & 0x00FF);

	ov5647_group_hold_on();
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_HSB, intg_time_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_MSB, intg_time_msb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_LSB, intg_time_lsb) ;

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_MSB, gain_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, gain_lsb) ;
	ov5647_group_hold_off();

    return 0;
}

static int32_t ov5647_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int rc = 0;


    uint16_t max_line;
	uint8_t gain_lsb, gain_hsb;
	u8 intg_time_hsb, intg_time_msb, intg_time_lsb;

	ov5647_ctrl->my_reg_gain = gain;
	ov5647_ctrl->my_reg_line_count = (uint16_t)line;

	gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
	gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain & 0x300)>>8);

	//CDBG(KERN_ERR "snapshot exposure seting 0x%x, 0x%x, %d", gain, line, line);

	if (line > snap_frame_length_lines -4) {
		rc = ov5647_i2c_write_b_sensor(0x380E,
			 (uint8_t)((line+4) >> 8)) ;
		rc = ov5647_i2c_write_b_sensor(0x380F,
			 (uint8_t)((line+4) & 0x00FF)) ;
		max_line = line + 4;
	} else if (max_line > snap_frame_length_lines) {
		rc = ov5647_i2c_write_b_sensor(0x380E,
			 (uint8_t)(snap_frame_length_lines >> 8)) ;
		rc = ov5647_i2c_write_b_sensor(0x380F,
			 (uint8_t)(snap_frame_length_lines & 0x00FF)) ;
		max_line = snap_frame_length_lines;
	}
	line = line<<4;
	/* ov5647 need this operation */
	intg_time_hsb = (u8)(line>>16);
	intg_time_msb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line & 0x00FF);

	/* FIXME for BLC trigger */
	ov5647_group_hold_on();
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_HSB, intg_time_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_MSB, intg_time_msb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_LSB, intg_time_lsb) ;

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_MSB, gain_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, gain_lsb - 1) ;

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_HSB, intg_time_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_MSB, intg_time_msb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_LSB, intg_time_lsb) ;

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_MSB, gain_hsb) ;
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, gain_lsb) ;
	ov5647_group_hold_off();

    return rc;
}

static int32_t ov5647_move_focus(int direction, int32_t num_steps)
{
	uint8_t   code_val_msb = 0;
	uint8_t   code_val_lsb = 0;
	int16_t   step_direction, actual_step, next_position;
	int rc;

	if (num_steps == 0)
	{
		return 0;
	}

	if (direction == MOVE_NEAR)
		step_direction = 20;
	else if (direction == MOVE_FAR)
		step_direction = -20;
	else
		return -EINVAL;

	actual_step = (int16_t)(step_direction * num_steps);
	next_position = (int16_t)ov5647_ctrl->curr_lens_pos + actual_step;
	if (next_position < 0) {
		CDBG(KERN_ERR "%s: OV5647 position(=%d) out of range",
			__func__, next_position);
		next_position = 0;
	}
	if (next_position > 0x3FF) {
		CDBG(KERN_ERR "%s: OV5647 position(=%d) out of range",
			__func__, next_position);
		next_position = 0x3FF;
	}
	ov5647_ctrl->curr_lens_pos = next_position;

	code_val_msb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4);
	code_val_lsb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x000F) << 4);
	code_val_lsb |= mode_mask;

	rc = ov5647_af_i2c_write_b_sensor(code_val_msb, code_val_lsb);
	/* DAC Setting */
	if (rc != 0) 
	{
		CDBG(KERN_ERR "%s: WRITE ERROR lsb = 0x%x, msb = 0x%x",
			__func__, code_val_lsb, code_val_msb);
	} 
	else
	{
		CDBG(KERN_ERR "%s: Successful lsb = 0x%x, msb = 0x%x",
			__func__, code_val_lsb, code_val_msb);
		/* delay may set based on the steps moved
		when I2C write successful */
		msleep(10);
	}
	
	return 0;
}

static int32_t ov5647_set_default_focus(void)
{

	uint8_t  code_val_msb = 0;
	uint8_t  code_val_lsb = 0;
	int rc = 0;

	ov5647_ctrl->curr_lens_pos = 200;

	code_val_msb = (ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4;
	code_val_lsb = (ov5647_ctrl->curr_lens_pos & 0x000F) << 4;
	code_val_lsb |= mode_mask;

	CDBG(KERN_ERR "ov5647_set_default_focus:lens pos = %d",
		 ov5647_ctrl->curr_lens_pos);
	rc = ov5647_af_i2c_write_b_sensor(code_val_msb, code_val_lsb);
	/* DAC Setting */
	if (rc != 0)
	{
		CDBG(KERN_ERR "%s: WRITE ERROR lsb = 0x%x, msb = 0x%x",
			__func__, code_val_lsb, code_val_msb);
	}
	else
	{
		CDBG(KERN_ERR "%s: WRITE successful lsb = 0x%x, msb = 0x%x",
			__func__, code_val_lsb, code_val_msb);
	}

	msleep(100);
	
	return rc;
}

static int32_t ov5647_setting(enum ov5647_reg_update rupdate,
                                    enum ov5647_setting    rt)
{
    int32_t rc = 0;
    struct msm_camera_csi_params ov5647_csi_params;
    msleep(30);
	
    CDBG("%s: rupdate=%d , rt=%d\n", __func__, rupdate, rt);

	ov5647_stop_stream();
	
    switch (rupdate)
    {
        case UPDATE_PERIODIC:
		{
            if (rt == RES_PREVIEW)
            {
                ov5647_i2c_write_table(ov5647_preview,
                                                  ARRAY_SIZE(ov5647_preview));
                msleep(20);
		        if (!CSI_CONFIG) 
                {
			        msm_camio_vfe_clk_rate_set(192000000);
        			ov5647_csi_params.data_format = CSI_10BIT;
        			ov5647_csi_params.lane_cnt = 2;
        			ov5647_csi_params.lane_assign = 0xe4;
        			ov5647_csi_params.dpcm_scheme = 0;
        			ov5647_csi_params.settle_cnt = 0x18; 
        			rc = msm_camio_csi_config(&ov5647_csi_params);
        			msleep(20);
        			CSI_CONFIG = 1;
		        }
        
                msleep(20);
            }
            else
            {   
                rc = ov5647_i2c_write_table(ov5647_regs.snap_reg_settings,
                                                  ov5647_regs.snap_reg_settings_size);
                mdelay(10);

            }
			ov5647_start_stream();
			
			msleep(30);

            break;
        }
        case REG_INIT:
		{
			
			CSI_CONFIG = 0;
			
            rc = ov5647_i2c_write(ov5647_client->addr, 0x0100, 0x00);
            rc = ov5647_i2c_write(ov5647_client->addr, 0x0103, 0x01);
            mdelay(5);

            rc = ov5647_i2c_write_table(ov5647_regs.init_reg_settings,
                                                  ov5647_regs.init_reg_settings_size);
            mdelay(10);
            break;
         }

        default:
            rc = -EINVAL;
            break;
    }

    return rc;
}


static int32_t ov5647_video_config(int mode, int res)
{
    int32_t rc = 0;


    switch (res)
    {
        case QTR_SIZE:
            rc = ov5647_setting(UPDATE_PERIODIC, RES_PREVIEW);
            if (rc < 0)
            {
                return rc;
            }
            break;

        case FULL_SIZE:
            rc = ov5647_setting(UPDATE_PERIODIC, RES_CAPTURE);
            if (rc < 0)
            {
                return rc;
            }
            break;

        default:
            return 0;
    }           /* switch */

    ov5647_ctrl->prev_res   = res;
    ov5647_ctrl->curr_res   = res;
    ov5647_ctrl->sensormode = mode;

    return rc;
}

static int32_t ov5647_snapshot_config(int mode)
{
    int32_t rc = 0;

    rc = ov5647_setting(UPDATE_PERIODIC, RES_CAPTURE);
    if (rc < 0)
    {
        return rc;
    }

    ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;

    ov5647_ctrl->sensormode = mode;

	CDBG("%s:  rc = %d\n", __func__, rc );

    return rc;
}

static int32_t ov5647_raw_snapshot_config(int mode)
{
    int32_t rc = 0;

    rc = ov5647_setting(UPDATE_PERIODIC, RES_CAPTURE);
    if (rc < 0)
    {
        return rc;
    }

    ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;

    ov5647_ctrl->sensormode = mode;

    return rc;
}

static int32_t ov5647_set_sensor_mode(int mode, int res)
{
    int32_t rc = 0;

    switch (mode)
    {
        case SENSOR_PREVIEW_MODE:
            rc = ov5647_video_config(mode, res);
            break;

        case SENSOR_SNAPSHOT_MODE:
            rc = ov5647_snapshot_config(mode);
            break;

        case SENSOR_RAW_SNAPSHOT_MODE:
            rc = ov5647_raw_snapshot_config(mode);
            break;

        default:
            rc = -EINVAL;
            break;
    }

    return rc;
}

static void ov5647_power_down(void)
{
    if(ov5647_ctrl->sensordata->vreg_disable_func)
    {
        ov5647_ctrl->sensordata->vreg_disable_func(0);
    }
}

static int ov5647_probe_init_done(const struct msm_camera_sensor_info *data)
{
    CDBG("ov5647_probe_init_done start\n");

    gpio_direction_output ( data->sensor_pwd, 0);
    gpio_direction_output(data->sensor_reset, 0);
    gpio_free ( data->sensor_pwd );
    gpio_free ( data->sensor_reset );
    if (data->vreg_enable_func)
    {
       data->vreg_enable_func(0);
    } 
	
    return 0;
}

static int ov5647_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
    int32_t rc = 0;
    uint16_t chipid = 0;

    CDBG("ov5647_probe_init_sensor\n");

    rc = gpio_request(data->sensor_reset, "ov5647");
    if (!rc)
    {
        gpio_direction_output(data->sensor_reset, 0);
    }
    else
    {
        goto init_probe_fail;
    }
   
    mdelay(1);

    rc = gpio_request(data->sensor_pwd, "ov5647");
    if (!rc)
    {
        gpio_direction_output(data->sensor_pwd, 0);
    }
    else
    {
       goto init_probe_fail;
    }
    
	mdelay(1);
    
    if (data->vreg_enable_func)
    {
       data->vreg_enable_func(1);
    } 
	
    mdelay(10);

    gpio_direction_output(data->sensor_pwd, 1);
	
    mdelay(1);
	
    gpio_direction_output(data->sensor_reset, 1);

    mdelay(30);

    /*  Read sensor Model ID: */
    rc = ov5647_i2c_read_w(ov5647_client->addr, OV5647_REG_MODEL_ID, &chipid);
    if (rc < 0)
    {
        goto init_probe_fail;
    }

    CDBG("model_id = 0x%x\n", chipid);

    /* Compare sensor ID to OV5647_MODEL_ID */
    if (OV5647_MODEL_ID != chipid)
    {
        CDBG("wrong model_id = 0x%x\n", chipid);
        rc = -ENODEV;
        goto init_probe_fail;
    }

    goto init_probe_done;

    CDBG("init_probe_done start\n");

init_probe_fail:
    ov5647_probe_init_done(data);
	
init_probe_done:
	
    return rc;
}

static int ov5647_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int32_t rc = 0;

    ov5647_ctrl = kzalloc(sizeof(struct ov5647_ctrl), GFP_KERNEL);
    if (!ov5647_ctrl)
    {
        CDBG("ov5647_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }

    ov5647_ctrl->fps_divider = 1 * 0x00000400;
    ov5647_ctrl->pict_fps_divider = 1 * 0x00000400;
    ov5647_ctrl->set_test = TEST_OFF;
    ov5647_ctrl->prev_res = QTR_SIZE;
    ov5647_ctrl->pict_res = FULL_SIZE;

    if (data)
    {
        ov5647_ctrl->sensordata = data;
    }

    mdelay( 20 );

	prev_frame_length_lines = 0x0466; 

	prev_line_length_pck = 0x768*2;

	snap_frame_length_lines = 0x07b6;

	snap_line_length_pck = 0xa8c;

    /* enable mclk first */
	msm_camio_clk_rate_set(OV5647_MASTER_CLK_RATE);

    rc = ov5647_probe_init_sensor(data);
    if (rc < 0)
    {
        goto init_fail1;
    }

    if (ov5647_ctrl->prev_res == QTR_SIZE)
    {
        rc = ov5647_setting(REG_INIT, RES_PREVIEW);
    }
    else
    {
        rc = ov5647_setting(REG_INIT, RES_CAPTURE);
    }

    if (rc < 0)
    {
        CDBG("ov5647_setting failed. rc = %d\n", rc);
        goto init_fail1;
    }

    /* enable AF actuator */
	if (ov5647_ctrl->sensordata->vcm_enable) 
	{
		rc = gpio_request(ov5647_ctrl->sensordata->vcm_pwd, "ov5647_af");
		if (!rc)
		{
			gpio_direction_output(ov5647_ctrl->sensordata->vcm_pwd, 1);
		}
		else 
		{
			CDBG("af gpio request failed!\n");
			goto init_fail2;
		}
		msleep(20);
		rc = ov5647_set_default_focus();
		if (rc < 0)
		{
			gpio_direction_output(ov5647_ctrl->sensordata->vcm_pwd, 0);
			gpio_free(ov5647_ctrl->sensordata->vcm_pwd);
		}
	}

    goto init_done;


init_fail2:
	if (data->vcm_enable) {
		int ret = gpio_request(data->vcm_pwd, "s5k4e1_af");
		if (!ret) {
			gpio_direction_output(data->vcm_pwd, 0);
			msleep(20);
			gpio_free(data->vcm_pwd);
		}
	}
	
init_fail1:
    ov5647_probe_init_done(data);

init_done:
    return rc;
}

static int ov5647_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&ov5647_wait_queue);
    return 0;
}
static int ov5647_af_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5647_af_wait_queue);
	return 0;
}

static const struct i2c_device_id ov5647_af_i2c_id[] = {
	{"ov5647_af", 0},
	{ }
};


static int ov5647_af_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	
	CDBG("ov5647_af_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov5647_af_sensorw = kzalloc(sizeof(struct ov5647_work), GFP_KERNEL);
	if (!ov5647_af_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov5647_af_sensorw);
	ov5647_af_init_client(client);
	ov5647_af_client = client;

	msleep(50);

	CDBG("ov5647_af_probe successed! rc = %d\n", rc);
	
	return 0;

probe_failure:
	CDBG("ov5647_af_probe failed! rc = %d\n", rc);
	return rc;
}


static const struct i2c_device_id ov5647_i2c_id[] =
{
    {"ov5647", 0},
    {}
};
static int ov5647_i2c_probe(struct i2c_client *         client,
                                  const struct i2c_device_id *id)
{
    int rc = 0;

    CDBG("ov5647_probe called!\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        CDBG("i2c_check_functionality failed\n");
        goto probe_failure;
    }

    ov5647_sensorw = kzalloc(sizeof(struct ov5647_work), GFP_KERNEL);
    if (!ov5647_sensorw)
    {
        CDBG("kzalloc failed.\n");
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, ov5647_sensorw);
    ov5647_init_client(client);
    ov5647_client = client;

    mdelay(50);

    CDBG("ov5647_probe successed! rc = %d\n", rc);
	
    return 0;

probe_failure:
    CDBG("ov5647_probe failed! rc = %d\n", rc);
    return rc;
}
static struct i2c_driver ov5647_i2c_driver =
{
    .id_table = ov5647_i2c_id,
    .probe    = ov5647_i2c_probe,
    .remove   = __exit_p(ov5647_i2c_remove),
    .driver   = {
        .name = "ov5647",
    },
};

static struct i2c_driver ov5647_af_i2c_driver = {
	.id_table = ov5647_af_i2c_id,
	.probe  = ov5647_af_i2c_probe,
	.remove = __exit_p(ov5647_af_i2c_remove),
	.driver = {
		.name = "ov5647_af",
	},
};


int ov5647_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    int rc = 0;

    if (copy_from_user(&cdata,
                       (void *)argp, sizeof(struct sensor_cfg_data)))
    {
        return -EFAULT;
    }

    mutex_lock(&ov5647_mut);

    CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
    switch (cdata.cfgtype)
    {
        case CFG_GET_PICT_FPS:
            ov5647_get_pict_fps(cdata.cfg.gfps.prevfps,
                                      &(cdata.cfg.gfps.pictfps));

            if (copy_to_user((void *)argp, &cdata,
                             sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_GET_PREV_L_PF:
            cdata.cfg.prevl_pf = ov5647_get_prev_lines_pf();

            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_GET_PREV_P_PL:
            cdata.cfg.prevp_pl = ov5647_get_prev_pixels_pl();

            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_GET_PICT_L_PF:
            cdata.cfg.pictl_pf = ov5647_get_pict_lines_pf();

            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_GET_PICT_P_PL:
            cdata.cfg.pictp_pl = ov5647_get_pict_pixels_pl();

            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_GET_PICT_MAX_EXP_LC:
            cdata.cfg.pict_max_exp_lc = ov5647_get_pict_max_exp_lc();

            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_SET_FPS:
        case CFG_SET_PICT_FPS:
            rc = ov5647_set_fps(&(cdata.cfg.fps));
            break;

        case CFG_SET_EXP_GAIN:
            rc = ov5647_write_exp_gain(cdata.cfg.exp_gain.gain,
                                             cdata.cfg.exp_gain.line);
            break;

        case CFG_SET_PICT_EXP_GAIN:
            CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
            rc = ov5647_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
                                                cdata.cfg.exp_gain.line);
            break;

        case CFG_SET_MODE:
            rc = ov5647_set_sensor_mode(cdata.mode, cdata.rs);
            break;

        case CFG_PWR_DOWN:
            ov5647_power_down();
            break;

        case CFG_MOVE_FOCUS:
            CDBG("ov5647_ioctl: CFG_MOVE_FOCUS: cdata.cfg.focus.dir=%d cdata.cfg.focus.steps=%d\n"                                                                   ,
                 cdata.cfg.focus.dir, cdata.cfg.focus.steps);
            rc = ov5647_move_focus(cdata.cfg.focus.dir,
                                         cdata.cfg.focus.steps);
            break;

        case CFG_SET_DEFAULT_FOCUS:
            rc = ov5647_set_default_focus();
            break;

        case CFG_SET_LENS_SHADING:
            CDBG("%s: CFG_SET_LENS_SHADING\n", __func__);
            rc = ov5647_lens_shading_enable(cdata.cfg.lens_shading);
            break;

        case CFG_GET_AF_MAX_STEPS:
            cdata.max_steps = OV5647_TOTAL_STEPS_NEAR_TO_FAR;
            if (copy_to_user((void *)argp,
                             &cdata, sizeof(struct sensor_cfg_data)))
            {
                rc = -EFAULT;
            }

            break;

        case CFG_SET_EFFECT:
        default:
            rc = -EINVAL;
            break;
    }

    mutex_unlock(&ov5647_mut);

    return rc;
}

int ov5647_sensor_release(void)
{
    int rc = -EBADF;

    mutex_lock(&ov5647_mut);

    gpio_direction_output(ov5647_ctrl->sensordata->sensor_pwd, 0);
    gpio_direction_output(ov5647_ctrl->sensordata->sensor_reset, 0);

    gpio_free(ov5647_ctrl->sensordata->sensor_reset);

    gpio_free(ov5647_ctrl->sensordata->sensor_pwd);


	if (ov5647_ctrl->sensordata->vcm_enable) 
	{
		gpio_direction_output(ov5647_ctrl->sensordata->vcm_pwd, 0);
		gpio_free(ov5647_ctrl->sensordata->vcm_pwd);
	}

   /*quit camera application ,power down camera*/
    ov5647_power_down();

    kfree(ov5647_ctrl);
    ov5647_ctrl = NULL;

    CDBG("ov5647_release completed\n");

    mutex_unlock(&ov5647_mut);
    return rc;
}

static int ov5647_sensor_probe(const struct msm_camera_sensor_info *info,
                                     struct msm_sensor_ctrl *             s)
{
    int rc = i2c_add_driver(&ov5647_i2c_driver);

    CDBG("ov5647_sensor_probe \n" );
    if ((rc < 0) || (ov5647_client == NULL))
    {
        rc = -ENOTSUPP;
        CDBG("ov5647:   i2c_add_driver  failed \n" );
        goto probe_done;
    }

	if (info->vcm_enable)
	{
		rc = i2c_add_driver(&ov5647_af_i2c_driver);
		if (rc < 0 || ov5647_af_client == NULL) {
			rc = -ENOTSUPP;
			CDBG("I2C add driver ov5647 af failed");
			goto probe_fail_1;
		}
	}
	else
	{
		CDBG("ov5647_af_i2c_driver ok \n");
	}


	
    msm_camio_clk_rate_set(OV5647_MASTER_CLK_RATE);
	
    rc = ov5647_probe_init_sensor(info);
    if (rc < 0)
    {
        CDBG("ov5647_probe_init_sensor failed!!\n");
        goto probe_done;
    }

	strncpy((char *)info->sensor_name, "23060049SA-OV-S", strlen("23060049SA-OV-S"));

    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    set_hw_dev_flag(DEV_I2C_CAMERA_MAIN);
    #endif
	
    s->s_init = ov5647_sensor_open_init;
    s->s_release = ov5647_sensor_release;
    s->s_config = ov5647_sensor_config;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
    ov5647_probe_init_done(info);

probe_fail_1:
	i2c_del_driver(&ov5647_af_i2c_driver);
probe_done:
    CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
    return rc;
}

static int __ov5647_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, ov5647_sensor_probe);
}

static struct platform_driver msm_camera_driver =
{
    .probe     = __ov5647_probe,
    .driver    = {
        .name  = "msm_camera_ov5647",
        .owner = THIS_MODULE,
    },
};

static int __init ov5647_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(ov5647_init);

