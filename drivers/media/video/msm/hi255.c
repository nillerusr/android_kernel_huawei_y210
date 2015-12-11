/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "hi255.h"
#include "linux/hardware_self_adapt.h"
#include <asm/mach-types.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
 #include <linux/hw_dev_dec.h>
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HI255
 #undef CDBG
 #define CDBG(fmt, args...) printk(KERN_INFO "hi255.c: " fmt, ## args)
#endif

/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define HI255_CHIP_ID 0xf4

enum hi255_test_mode_t
{
    TEST_OFF,
    TEST_1,
    TEST_2,
    TEST_3
};

enum hi255_resolution_t
{
    QTR_SIZE,
    FULL_SIZE,
    INVALID_SIZE
};

enum hi255_reg_update_t
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

enum hi255_setting_t
{
    RES_PREVIEW,
    RES_CAPTURE
};

/*
 * Time in milisecs for waiting for the sensor to reset.
 */
#define HI255_RESET_DELAY_MSECS 66

/* for 30 fps preview */
#define HI255_DEFAULT_CLOCK_RATE 24000000

#define HI255_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/* FIXME: Changes from here */
struct hi255_work_t
{
    struct work_struct work;
};

struct hi255_ctrl_t
{
    const struct  msm_camera_sensor_info *sensordata;

    int      sensormode;
    uint32_t fps_divider; /* init to 1 * 0x00000400 */
    uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

    uint16_t curr_lens_pos;
    uint16_t init_curr_lens_pos;
    uint16_t my_reg_gain;
    uint32_t my_reg_line_count;

    enum hi255_resolution_t prev_res;
    enum hi255_resolution_t pict_res;
    enum hi255_resolution_t curr_res;
    enum hi255_test_mode_t  set_test;

    unsigned short imgaddr;
};

const static char hi255_supported_effect[] = "none,mono,negative,sepia,aqua";
static bool CSI_CONFIG;
#define MODEL_TRULY 0
#define MODEL_SUNNY 1

#define REGISTER_WRITE_WAIT_FW 1
#define REGISTER_WRITE_WAIT_PATCH 2
#define REGISTER_WRITE_WAIT_PREVIEW 3
#define REGISTER_WRITE_WAIT_SNAPSHOT 4
#define EFFECT_OFF_VALUE 0
#define EFFECT_MONO_VALUE 1
#define EFFECT_SEPIA_VALUE 2
#define EFFECT_AQUA_VALUE 2
#define EFFECT_NEGATIVE_VALUE 3
#define WB_AUTO_VALUE 0x003F
#define WB_NOT_AUTO_VALUE 0
static int current_effect  = CAMERA_EFFECT_OFF;
static struct  hi255_work_t *hi255sensorw = NULL;

static struct  i2c_client *hi255_client = NULL;
static struct hi255_ctrl_t *hi255_ctrl = NULL;
static enum hi255_reg_update_t last_rupdate = -1;
static enum hi255_setting_t last_rt = -1;
static DECLARE_WAIT_QUEUE_HEAD(hi255_wait_queue);
DEFINE_MUTEX(hi255_sem);

static int32_t hi255_i2c_read_w(unsigned char raddr, unsigned char *rdata)
{
    int32_t rc = 0;
    unsigned char buf;
 struct i2c_msg msgs[] =
    {
        {
            .addr  = hi255_client->addr,
            .flags = 0,
            .len = 1,
            .buf = &buf,
        },
        {
            .addr  = hi255_client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = &buf,
        },
    };

    buf = raddr;

    if (i2c_transfer(hi255_client->adapter, msgs, 2) < 0)
    {
        CDBG("hi255_i2c_rxdata failed!\n");
        return -EIO;
    }

    *rdata = buf;

    if (rc < 0)
    {
        CDBG("hi255_i2c_read failed!\n");
    }

    return rc;
}

static int32_t hi255_i2c_txdata(unsigned char saddr,
                                 unsigned char *txdata, int length)
{
    int32_t i  = 0;
    int32_t rc = -EFAULT;
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    for (i = 0; i < 3; i++)
    {
        rc = i2c_transfer(hi255_client->adapter, msg, 1);
        if (0 <= rc)
        {
            return 0;
        }
    }

    if (3 == i)
    {
        CDBG("hi255_i2c_txdata faild\n");
        return -EIO;
    }

    return 0;
}

static int32_t hi255_i2c_write_w(unsigned char waddr, unsigned char wdata)
{
    unsigned char buf[2];
    int32_t rc = 0;
    buf[0] = waddr;
    buf[1] = wdata;

    rc = hi255_i2c_txdata(hi255_client->addr, buf, 2);

    if (rc < 0)
    {
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
    }

    return rc;
}

static int32_t hi255_i2c_write_w_table(struct hi255_i2c_reg_conf const *reg_conf_tbl,
                                        int                               num_of_items_in_table)
{
    int i;
    int32_t rc = -EFAULT;

    for (i = 0; i < num_of_items_in_table; i++)
    {
        rc = hi255_i2c_write_w(reg_conf_tbl->waddr, reg_conf_tbl->wdata);
        if (rc < 0)
        {
            break;
        }

        reg_conf_tbl++;
    }

    return rc;
}

int32_t hi255_set_default_focus(uint8_t af_step)
{
    CDBG("s5k4cdgx_set_default_focus:\n");

    return 0;
}

int32_t hi255_set_fps(struct fps_cfg    *fps)
{
    /* input is new fps in Q8 format */
    int32_t rc = 0;

    CDBG("hi255_set_fps\n");
    return rc;
}

int32_t hi255_write_exp_gain(uint16_t gain, uint32_t line)
{
    CDBG("hi255_write_exp_gain\n");
    return 0;
}

int32_t hi255_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
    int32_t rc = 0;

    CDBG("hi255_set_pict_exp_gain\n");

    mdelay(10);

    /* camera_timed_wait(snapshot_wait*exposure_ratio); */
    return rc;
}
/* Mirror from kernel space */ 
int32_t hi255_setting(enum hi255_reg_update_t rupdate,
                       enum hi255_setting_t    rt)
{
    struct msm_camera_csi_params hi255_csi_params;
    int32_t rc = 0;
  
    if ((rupdate == last_rupdate) && (rt == last_rt))
    {
        CDBG("hi255_setting exit\n");
        return rc;
    }

    CDBG("hi255_setting in rupdate=%d,rt=%d\n", rupdate, rt);
    switch (rupdate)
    {
    case UPDATE_PERIODIC:

        /*preview setting*/
        if (rt == RES_PREVIEW)
        {

                CDBG("hi255:  sensor: init preview reg.\n");
                rc = hi255_i2c_write_w_table(hi255_regs.hi255_preview_reg_config,
                                          hi255_regs.hi255_preview_reg_config_size);
                if (rc)
                {
                   CDBG("       write hi255_preview_reg_config error!!!!!!");
                }

            if(!CSI_CONFIG)
            {
                CDBG("hi255: init CSI  config!\n");
             //   msm_camio_vfe_clk_rate_set(192000000);
                hi255_csi_params.data_format = CSI_8BIT;
                hi255_csi_params.lane_cnt = 1;
                hi255_csi_params.lane_assign = 0xe4;
                hi255_csi_params.dpcm_scheme = 0;
                hi255_csi_params.settle_cnt = 0x18;
                rc = msm_camio_csi_config(&hi255_csi_params);
                CSI_CONFIG = 1;
            }
        }
        /*snapshot setting*/
        else
        {
            CDBG("hi255:  sensor: init snapshot reg.\n");
            rc = hi255_i2c_write_w_table(hi255_regs.hi255_snapshot_reg_config,
                                          hi255_regs.hi255_snapshot_reg_config_size);
        }
        mdelay(5);
        break;

    case REG_INIT:

        /* Write init sensor register */

        rc = hi255_i2c_write_w_table(hi255_regs.hi255_init_reg_sensor_start,
                                      hi255_regs.hi255_init_reg_sensor_start_size);

        if (rc)
        {
            CDBG("       write hi255_init_reg_sensor_start error!!!!!!");
        }

        break;
    default:
        rc = -EFAULT;
        break;
    } /* switch (rupdate) */
    if (rc == 0)
    {
        last_rupdate = rupdate;
        last_rt = rt;
    }

    return rc;
}

int32_t hi255_video_config(int mode, int res)
{
    int32_t rc;

    switch (res)
    {
    case QTR_SIZE:
        rc = hi255_setting(UPDATE_PERIODIC, RES_PREVIEW);
        if (rc < 0)
        {
            return rc;
        }

        CDBG("sensor configuration done!\n");
        break;

    case FULL_SIZE:
        rc = hi255_setting(UPDATE_PERIODIC, RES_CAPTURE);
        if (rc < 0)
        {
            return rc;
        }

        break;

    default:
        return 0;
    } /* switch */

    hi255_ctrl->prev_res   = res;
    hi255_ctrl->curr_res   = res;
    hi255_ctrl->sensormode = mode;

    return rc;
}

int32_t hi255_snapshot_config(int mode)
{
    int32_t rc = 0;

    CDBG("hi255_snapshot_config in\n");
    rc = hi255_setting(UPDATE_PERIODIC, RES_CAPTURE);
    if (rc < 0)
    {
        return rc;
    }

    hi255_ctrl->curr_res = hi255_ctrl->pict_res;

    hi255_ctrl->sensormode = mode;

    return rc;
}

int32_t hi255_power_down(void)
{
    int32_t rc = 0;

    mdelay(1);

    return rc;
}

int32_t hi255_move_focus(int direction, int32_t num_steps)
{
    return 0;
}

static int hi255_sensor_init_done(const struct msm_camera_sensor_info *data)
{

    gpio_direction_output(data->sensor_reset, 0);
    gpio_free(data->sensor_reset);

    gpio_direction_output(data->sensor_pwd, 1);
    gpio_free(data->sensor_pwd);

    if (data->vreg_disable_func)
    {
        data->vreg_disable_func(0);
    }


    last_rupdate = -1;
    last_rt = -1;
    return 0;
}

static int hi255_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
    int rc;
    unsigned char chipid =0x00;

    /* pull down power down */
    rc = gpio_request(data->sensor_pwd, "hi255");
    if (!rc || (rc == -EBUSY))
    {
        gpio_direction_output(data->sensor_pwd, 0);
    }
    else
    {
        goto init_probe_fail;
    }

    mdelay(1);
    /* Set the sensor reset when camera is not initialization. */
    {
        rc = gpio_request(data->sensor_reset, "hi255");
        if (!rc || (rc == -EBUSY))
        {
            rc = gpio_direction_output(data->sensor_reset, 0);
        }
        else
        {
            goto init_probe_fail;
        }
        mdelay(2);

       if (data->vreg_enable_func)
       {
           data->vreg_enable_func(1);
       }
        mdelay(60);

        /*hardware reset*/
        /* Set the sensor reset when camera is not initialization. */
        rc = gpio_direction_output(data->sensor_reset, 1);
        if (rc < 0)
        {
            goto init_probe_fail;
        }
        mdelay(5);
        /* Set the soft reset to reset the chip and read the chip ID when camera is not initialization. */
        /* 3. Read sensor Model ID: */
        rc = hi255_i2c_read_w(0x04, &chipid);
        if (rc < 0)
        {
            CDBG("hi255_i2c_read_w Model_ID failed!! rc=%d", rc);
            goto init_probe_fail;
        }

        CDBG("hi255 chipid = 0x%x\n", chipid);

        /* 4. Compare sensor ID to HI255 ID: */
        if (chipid != HI255_CHIP_ID)
        {
            CDBG("hi255 Model_ID error!!");
            rc = -ENODEV;
            goto init_probe_fail;
        }
        CDBG("sensor name is %s.", data->sensor_name);
    }

    goto init_probe_done;

init_probe_fail:
    hi255_sensor_init_done(data);
init_probe_done:
    return rc;
}

int hi255_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int32_t rc;

    hi255_ctrl = kzalloc(sizeof(struct hi255_ctrl_t), GFP_KERNEL);
    if (!hi255_ctrl)
    {
        CDBG("hi255_sensor_open_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }
	 current_effect  = CAMERA_EFFECT_OFF;
    CDBG("hi255_sensor_open_*****************!\n");
    hi255_ctrl->fps_divider = 1 * 0x00000400;
    hi255_ctrl->pict_fps_divider = 1 * 0x00000400;
    hi255_ctrl->set_test = TEST_OFF;
    hi255_ctrl->prev_res = QTR_SIZE;
    hi255_ctrl->pict_res = FULL_SIZE;

    if (data)
    {
        hi255_ctrl->sensordata = data;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(HI255_DEFAULT_CLOCK_RATE);
    mdelay(1);

    rc = hi255_probe_init_sensor(data);
    if (rc < 0)
    {
        CDBG("hi255 init failed!!!!!\n");
        goto init_fail;
    }
    else
    {
        CSI_CONFIG = 0;
        CDBG("hi255 init succeed!!!!! rc = %d \n", rc);
        rc = hi255_setting(REG_INIT, RES_PREVIEW);
        if (rc < 0)
        {
            CDBG("hi255 init failed!!!!!\n");
            goto init_fail;
        }
        goto init_done;
    }

    /* Don't write sensor init register at open camera. */

init_fail:
    kfree(hi255_ctrl);
init_done:
    return rc;
}

int hi255_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&hi255_wait_queue);
    return 0;
}

int32_t hi255_set_sensor_mode(int mode, int res)
{
    int32_t rc = 0;

    switch (mode)
    {
    case SENSOR_PREVIEW_MODE:
        CDBG("SENSOR_PREVIEW_MODE,res=%d\n", res);
        rc = hi255_video_config(mode, res);
        break;

    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:
        CDBG("SENSOR_SNAPSHOT_MODE\n");
        rc = hi255_snapshot_config(mode);
        break;

    default:
        rc = -EINVAL;
        break;
    }

    return rc;
}

static long hi255_set_effect(int mode, int effect)
{
    struct hi255_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;
    printk("hi255_set_effect =%d current_effect  = %d;\n ",effect,current_effect);
    /* variable 0xE887 is the key of effect
     * if the value of register is already the value
     * we are to set, return from the func
     */
    switch (effect)
    {
    case CAMERA_EFFECT_OFF:
        reg_conf_tbl = hi255_regs.hi255_effect_off_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_off_reg_config_size;
        break;

    case CAMERA_EFFECT_MONO:
        reg_conf_tbl = hi255_regs.hi255_effect_mono_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_mono_reg_config_size;
        break;

    case CAMERA_EFFECT_NEGATIVE:
        reg_conf_tbl = hi255_regs.hi255_effect_negative_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_negative_reg_config_size;
        break;

    case CAMERA_EFFECT_SEPIA:
        reg_conf_tbl = hi255_regs.hi255_effect_sepia_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_sepia_reg_config_size;
        break;

    case CAMERA_EFFECT_AQUA:
        reg_conf_tbl = hi255_regs.hi255_effect_aqua_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_aqua_reg_config_size;
        break;

    /*the effect we need is solarize and posterize*/
    case CAMERA_EFFECT_SOLARIZE:
        reg_conf_tbl = hi255_regs.hi255_effect_solarize_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_solarize_reg_config_size;
        break;

    case CAMERA_EFFECT_POSTERIZE:
        reg_conf_tbl = hi255_regs.hi255_effect_posterize_reg_config;
        num_of_items_in_table = hi255_regs.hi255_effect_posterize_reg_config_size;
        break;

    default:
        return 0;
    }
	 if(current_effect != effect)
    {
       rc = hi255_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
       current_effect = effect;
    }
    //rc = hi255_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    mdelay(50);
    return rc;
}

static long hi255_set_wb(int wb)
{
    struct hi255_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;
    /* variable 0x6848 is the key of whitebalance
     * if the value of register is already the value
     * we are to set, return from the func
     */
    CDBG("hi255_set_wb FF wb:%d\n", wb);
    switch (wb)
    {
    case CAMERA_WB_AUTO:
        reg_conf_tbl = hi255_regs.hi255_wb_auto_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_auto_reg_config_size;
        break;

    case CAMERA_WB_INCANDESCENT:
        reg_conf_tbl = hi255_regs.hi255_wb_a_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_a_reg_config_size;
        break;

    case CAMERA_WB_CUSTOM:
        reg_conf_tbl = hi255_regs.hi255_wb_f_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_f_reg_config_size;
        break;
    case CAMERA_WB_FLUORESCENT:
        reg_conf_tbl = hi255_regs.hi255_wb_tl84_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_tl84_reg_config_size;
        break;

    case CAMERA_WB_DAYLIGHT:
        /* Daylight should be D50 */
        reg_conf_tbl = hi255_regs.hi255_wb_d50_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_d50_reg_config_size;
        break;

    case CAMERA_WB_CLOUDY_DAYLIGHT:
        /* Cloudy should be D65 */
        reg_conf_tbl = hi255_regs.hi255_wb_d65_reg_config;
        num_of_items_in_table = hi255_regs.hi255_wb_d65_reg_config_size;
        break;

    case CAMERA_WB_TWILIGHT:
        return 0;
        break;

    case CAMERA_WB_SHADE:
        return 0;
        break;

    default:
        return 0;
    }
    rc = hi255_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    mdelay(50);
    return rc;
}

int hi255_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    long rc = 0;
    if (copy_from_user(&cdata,
                       (void *)argp,
                       sizeof(struct sensor_cfg_data)))
    {
        return -EFAULT;
    }
    mutex_lock(&hi255_sem);
    CDBG("hi255_sensor_config: cfgtype = %d\n",
         cdata.cfgtype);
    switch (cdata.cfgtype)
    {
    case CFG_GET_PICT_FPS:
        break;

    case CFG_GET_PREV_L_PF:
        break;

    case CFG_GET_PREV_P_PL:
        break;

    case CFG_GET_PICT_L_PF:
        break;

    case CFG_GET_PICT_P_PL:
        break;

    case CFG_GET_PICT_MAX_EXP_LC:
        break;

    case CFG_SET_FPS:
    case CFG_SET_PICT_FPS:
        rc = hi255_set_fps(&(cdata.cfg.fps));
        break;

    case CFG_SET_EXP_GAIN:
        rc =
            hi255_write_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_PICT_EXP_GAIN:
        rc =
            hi255_set_pict_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_MODE:
        rc = hi255_set_sensor_mode(cdata.mode,
                                    cdata.rs);
        break;

    case CFG_PWR_DOWN:
        rc = hi255_power_down();
        break;

    case CFG_MOVE_FOCUS:
        rc =
            hi255_move_focus(
            cdata.cfg.focus.dir,
            cdata.cfg.focus.steps);
        break;

    case CFG_SET_DEFAULT_FOCUS:
        rc =
            hi255_set_default_focus(
            cdata.cfg.focus.steps);
        break;
    case CFG_SET_EFFECT:
        rc = hi255_set_effect(cdata.mode,
                               cdata.cfg.effect);
        break;

    case CFG_SET_WB:
        rc = hi255_set_wb(cdata.cfg.effect);
        break;

    case CFG_SET_ANTIBANDING:
        break;

    case CFG_MAX:

        break;

    default:
        rc = -EFAULT;
        break;
    }
    mutex_unlock(&hi255_sem);

    return rc;
}

int hi255_sensor_release(void)
{
    int rc = -EBADF;

    mutex_lock(&hi255_sem);

    hi255_power_down();

    hi255_sensor_init_done(hi255_ctrl->sensordata);

    kfree(hi255_ctrl);

    mutex_unlock(&hi255_sem);
    CDBG("hi255_release completed!\n");
    return rc;
}

static int hi255_i2c_probe(struct i2c_client *         client,
                            const struct i2c_device_id *id)
{
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    hi255sensorw =
        kzalloc(sizeof(struct hi255_work_t), GFP_KERNEL);

    if (!hi255sensorw)
    {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, hi255sensorw);
    hi255_init_client(client);
    hi255_client = client;

    //hi255_client->addr = hi255_client->addr >> 1;
    mdelay(50);

    CDBG("i2c probe ok\n");
    return 0;

probe_failure:
    kfree(hi255sensorw);
    hi255sensorw = NULL;
    pr_err("i2c probe failure %d\n", rc);
    return rc;
}

static const struct i2c_device_id hi255_i2c_id[] =
{
    { "hi255", 0},
    { }
};

static struct i2c_driver hi255_i2c_driver =
{
    .id_table = hi255_i2c_id,
    .probe    = hi255_i2c_probe,
    .remove   = __exit_p(hi255_i2c_remove),
    .driver   = {
        .name = "hi255",
    },
};

static int hi255_sensor_probe(const struct msm_camera_sensor_info *info,
                               struct msm_sensor_ctrl *             s)
{
    /* We expect this driver to match with the i2c device registered
     * in the board file immediately. */
    int rc = i2c_add_driver(&hi255_i2c_driver);

    if ((rc < 0) || (hi255_client == NULL))
    {
        rc = -ENOTSUPP;
        goto probe_done;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(HI255_DEFAULT_CLOCK_RATE);
    mdelay(20);

    rc = hi255_probe_init_sensor(info);
    if (rc < 0)
    {
        CDBG("hi255 probe failed!!!!\n");
        i2c_del_driver(&hi255_i2c_driver);
        goto probe_done;
    }
    else
    {
        /*camera name for project menu to display*/
        strncpy((char *)info->sensor_name, "23060077FF-Hy-F", strlen("23060077FF-Hy-F"));

        CDBG("hi255 probe succeed!!!!\n");
    }
    /*initialize the registers to save the time of open camera*/
    /*don't use standby mode anymore, delete the initiation when probe*/

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_CAMERA_MAIN);
#endif

    s->s_init = hi255_sensor_open_init;
    s->s_release = hi255_sensor_release;
    s->s_config = hi255_sensor_config;
    /*set the s_mount_angle value of sensor*/
    s->s_mount_angle = info->sensor_platform_info->mount_angle;
    hi255_sensor_init_done(info);

    /* For go to sleep mode, follow the datasheet */
    msleep(150);
    set_camera_support(true);
probe_done:
    return rc;
}

static int __hi255_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, hi255_sensor_probe);
}

static struct platform_driver msm_camera_driver =
{
    .probe     = __hi255_probe,
    .driver    = {
        .name  = "msm_camera_hi255",
        .owner = THIS_MODULE,
    },
};

static int __init hi255_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(hi255_init);

