// SPDX-License-Identifier: GPL-2.0
/*
 * sc2232 driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version,adjust sc2232.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIPI_FREQ_186M				186000000
#define SC2232_MAX_PIXEL_RATE		(MIPI_FREQ_186M * 2 / 10 * 2)

#define SC2232_XVCLK_FREQ		27000000

#define CHIP_ID					0x2238
#define SC2232_REG_CHIP_ID		0x3107

#define SC2232_REG_CTRL_MODE		0x0100
#define SC2232_MODE_SW_STANDBY		0x0
#define SC2232_MODE_STREAMING		BIT(0)

#define	SC2232_EXPOSURE_MIN		3
#define	SC2232_EXPOSURE_STEP	1
#define SC2232_VTS_MAX			0xffff

#define SC2232_REG_EXP_LONG_H		0x3e00    //[3:0]
#define SC2232_REG_EXP_LONG_M		0x3e01    //[7:0]
#define SC2232_REG_EXP_LONG_L		0x3e02    //[7:4]

#define SC2232_REG_AGAIN			0x3e08
#define SC2232_REG_AGAIN_FINE		0x3e09

#define SC2232_REG_DGAIN			0x3e06
#define SC2232_REG_DGAIN_FINE		0x3e07

#define SC2232_GAIN_MIN				0x40
#define SC2232_GAIN_MAX				0x4000
#define SC2232_GAIN_STEP			1
#define SC2232_GAIN_DEFAULT			0x40

#define SC2232_SOFTWARE_RESET_REG	0x0103

#define SC2232_REG_VTS			0x320e
#define SC2232_REG_HTS			0x320c

#define SC2232_FLIP_REG			0x3221
#define SC2232_FLIP_MASK		0x60
#define SC2232_MIRROR_MASK		0x06
#define REG_NULL			0xFFFF

#define SC2232_REG_VALUE_08BIT		1
#define SC2232_REG_VALUE_16BIT		2
#define SC2232_REG_VALUE_24BIT		3

#define SC2232_LANES			2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define SC2232_NAME			"sc2232"

static const char * const sc2232_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC2232_NUM_SUPPLIES ARRAY_SIZE(sc2232_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc2232_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mipi_freq_idx;
	u32 bpp;
	u32 vc[PAD_MAX];
};

struct sc2232 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC2232_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	struct v4l2_fract	cur_fps;
	bool			streaming;
	bool			power_on;
	const struct sc2232_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_sc2232(sd) container_of(sd, struct sc2232, subdev)

static const struct regval sc2232_linear10bit_1920x1080_regs[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x3034,0x81},//pll2 bypass
	{0x3039,0xa2},//pll1 bypass
	{0x3624,0x08},
	{0x337f,0x03},
	{0x3368,0x04},
	{0x3369,0x00},
	{0x336a,0x00},
	{0x336b,0x00},
	{0x3367,0x08},
	{0x330e,0x30},
	{0x3366,0x7c},
	{0x3302,0x1f},
	{0x3907,0x00},
	{0x3902,0x45},
	{0x3908,0x11},
	{0x335e,0x01},
	{0x335f,0x03},
	{0x337c,0x04},
	{0x337d,0x06},
	{0x33a0,0x05},
	{0x3633,0x4f},
	{0x3622,0x06},
	{0x3631,0x84},
	{0x366e,0x08},
	{0x3326,0x00},
	{0x3303,0x20},
	{0x3638,0x1f},
	{0x3636,0x25},
	{0x3625,0x02},
	{0x331b,0x83},
	{0x3333,0x30},
	{0x3635,0xa0},
	{0x363c,0x05},
	{0x3038,0xff},
	{0x3639,0x09},
	{0x3621,0x28},
	{0x3211,0x0c},
	{0x3320,0x01},
	{0x331e,0x19},
	{0x3620,0x28},
	{0x3309,0x60},
	{0x331f,0x59},
	{0x3308,0x10},
	{0x3f00,0x07},
	{0x3802,0x01},  //0x01 for over 2fps update
	{0x33aa,0x10},
	{0x3677,0x86},
	{0x3678,0x88},
	{0x3679,0x88},
	{0x367e,0x08},
	{0x367f,0x28},
	{0x3670,0x0c},
	{0x3690,0x33},
	{0x3691,0x11},
	{0x3692,0x43},
	{0x369c,0x08},
	{0x369d,0x28},
	{0x360f,0x01},
	{0x3671,0xc6},
	{0x3672,0x06},
	{0x3673,0x16},
	{0x367a,0x28},
	{0x367b,0x3f},
	{0x320c,0x08},
	{0x320d,0x98},
	{0x320e,0x04},
	{0x320f,0x65},
	{0x3f04,0x04},
	{0x3f05,0x28},
	{0x3235,0x08},
	{0x3236,0xc8},
	{0x3222,0x29},
	{0x3901,0x02},
	{0x3905,0x98},
	{0x3e1e,0x34},
	{0x3900,0x19},
	{0x391d,0x04},
	{0x391e,0x00},
	{0x3641,0x01},
	{0x3213,0x04},
	{0x3614,0x80},
	{0x363a,0x9f},
	{0x3630,0x9c},
	{0x3306,0x48},
	{0x330b,0xcd},
	{0x3018,0x33},
	{0x3031,0x0a},
	{0x3037,0x20},
	{0x3001,0xfe},
	{0x4603,0x00},
	{0x4827,0x48},
	{0x301c,0x78},
	{0x4809,0x01},
	{0x3314,0x04},
	{0x303c,0x0e},
	{0x4837,0x35},
	{0x3933,0x0a},
	{0x3934,0x10},
	{0x3940,0x60},
	{0x3942,0x02},
	{0x3943,0x1f},
	{0x3960,0xba},
	{0x3961,0xae},
	{0x3966,0xba},
	{0x3980,0xa0},
	{0x3981,0x40},
	{0x3982,0x18},
	{0x3903,0x08},
	{0x3984,0x08},
	{0x3985,0x20},
	{0x3986,0x50},
	{0x3987,0xb0},
	{0x3988,0x08},
	{0x3989,0x10},
	{0x398a,0x20},
	{0x398b,0x30},
	{0x398c,0x60},
	{0x398d,0x20},
	{0x398e,0x10},
	{0x398f,0x08},
	{0x3990,0x60},
	{0x3991,0x24},
	{0x3992,0x15},
	{0x3993,0x08},
	{0x3994,0x0a},
	{0x3995,0x20},
	{0x3996,0x38},
	{0x3997,0xa0},
	{0x3998,0x08},
	{0x3999,0x10},
	{0x399a,0x18},
	{0x399b,0x30},
	{0x399c,0x30},
	{0x399d,0x18},
	{0x399e,0x10},
	{0x399f,0x08},
	{0x3637,0x55},
	{0x363b,0x06},
	{0x366f,0x2c},
	{0x5000,0x06},
	{0x5780,0x7f},
	{0x5781,0x04},
	{0x5782,0x03},
	{0x5783,0x02},
	{0x5784,0x01},
	{0x5785,0x18},
	{0x5786,0x10},
	{0x5787,0x08},
	{0x5788,0x02},
	{0x57a0,0x00},
	{0x57a1,0x71},
	{0x57a2,0x01},
	{0x57a3,0xf1},
	{0x395e,0xc0},
	{0x3962,0x89},
	{0x3e00,0x00},
	{0x3e01,0x8c},
	{0x3e02,0x60},
	{0x3e03,0x0b},
	{0x3e06,0x00},
	{0x3e07,0x80},
	{0x3e08,0x03},
	{0x3e09,0x10},
	{0x3301,0x0f},
	{0x3632,0x08},
	{0x3034,0x01},
	{0x3039,0x22},
	{0x0100,0x01},
	{REG_NULL, 0x00},
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
static const struct sc2232_mode supported_modes[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0463,
		.hts_def = 0x0898 * 2,
		.vts_def = 0x0465,
		.reg_list = sc2232_linear10bit_1920x1080_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_186M,
};

/* Write registers up to 4 at a time */
static int sc2232_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int sc2232_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= sc2232_write_reg(client, regs[i].addr,
			SC2232_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc2232_read_reg(struct i2c_client *client,
			    u16 reg,
			    unsigned int len,
			    u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int sc2232_get_reso_dist(const struct sc2232_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc2232_mode *
sc2232_find_best_fit(struct sc2232 *sc2232, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc2232->cfg_num; i++) {
		dist = sc2232_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}

	return &supported_modes[cur_best_fit];
}

static void sc2232_change_mode(struct sc2232 *sc2232, const struct sc2232_mode *mode)
{
	sc2232->cur_mode = mode;
	sc2232->cur_vts = sc2232->cur_mode->vts_def;
	dev_info(&sc2232->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
		mode->width, mode->height, mode->hdr_mode);
}

static int sc2232_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	const struct sc2232_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc2232->mutex);

	mode = sc2232_find_best_fit(sc2232, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc2232->mutex);
		return -ENOTTY;
#endif
	} else {
		sc2232_change_mode(sc2232, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc2232->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc2232->vblank, vblank_def,
					 SC2232_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(sc2232->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
			mode->bpp * 2 * SC2232_LANES;
		__v4l2_ctrl_s_ctrl_int64(sc2232->pixel_rate, pixel_rate);
		sc2232->cur_fps = mode->max_fps;
		sc2232->cur_vts = mode->vts_def;
	}

	mutex_unlock(&sc2232->mutex);

	return 0;
}

static int sc2232_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	const struct sc2232_mode *mode = sc2232->cur_mode;

	mutex_lock(&sc2232->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc2232->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc2232->mutex);

	return 0;
}

static int sc2232_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int sc2232_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc2232 *sc2232 = to_sc2232(sd);

	if (fse->index >= sc2232->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc2232_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	const struct sc2232_mode *mode = sc2232->cur_mode;

	if (sc2232->streaming)
		fi->interval = sc2232->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct sc2232_mode *sc2232_find_mode(struct sc2232 *sc2232, int fps)
{
	const struct sc2232_mode *mode = NULL;
	const struct sc2232_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < sc2232->cfg_num; i++) {
		mode = &supported_modes[i];
		if (mode->width == sc2232->cur_mode->width &&
		    mode->height == sc2232->cur_mode->height &&
		    mode->hdr_mode == sc2232->cur_mode->hdr_mode &&
		    mode->bus_fmt == sc2232->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int sc2232_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	const struct sc2232_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	int fps;

	if (sc2232->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = sc2232_find_mode(sc2232, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	sc2232->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(sc2232->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(sc2232->vblank, vblank_def,
				 SC2232_VTS_MAX - mode->height,
				 1, vblank_def);
	__v4l2_ctrl_s_ctrl(sc2232->link_freq, mode->mipi_freq_idx);
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
		mode->bpp * 2 * SC2232_LANES;
	__v4l2_ctrl_s_ctrl_int64(sc2232->pixel_rate, pixel_rate);
	sc2232->cur_fps = mode->max_fps;
	return 0;
}

static int sc2232_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	const struct sc2232_mode *mode = sc2232->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (SC2232_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (SC2232_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc2232_get_module_inf(struct sc2232 *sc2232,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SC2232_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, sc2232->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, sc2232->len_name, sizeof(inf->base.lens));
}

static int sc2232_set_gain(struct sc2232 *sc2232, u32 total_gain)
{
	u32 again = 0, again_fine = 0;
	u32 dgain = 0, dgain_fine = 0;
	u32 step = 0;
	u32 val = 0;
	int ret = 0;

	if (total_gain < 0x80) {/* 1x gain ~  2x gain */
		step = (total_gain - 0x40) >> 2;

		again = 0x0;
		again_fine = step + 0x10;
		dgain = 0x0;
		dgain_fine = 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x0f);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x08);

	} else if (total_gain < 0x100) {/* 2x gain ~  4x gain */
		step = (total_gain - 0x80) >> 3;

		again = 0x1;
		again_fine = step + 0x10;
		dgain = 0x0;
		dgain_fine = 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x20);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x08);

	} else if (total_gain < 0x200) {/* 4x gain ~  8x gain */
		step = (total_gain - 0x100) >> 4;

		again = 0x3;
		again_fine = step + 0x10;
		dgain = 0x0;
		dgain_fine = 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x28);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x08);

	} else if (total_gain < 0x400) {/* 8x gain ~  16x gain */
		step = (total_gain - 0x200) >> 5;

		again = 0x7;
		again_fine = step + 0x10;
		dgain = 0x0;
		dgain_fine = 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x80);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x08);

	} else if (total_gain < 0x800) { /* 16x gain ~  32x gain */
		step = (total_gain - 0x400) >> 6;

		again = 0x7;
		again_fine = 0x1f;
		dgain = 0x0;
		dgain_fine = step * 8 + 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x80);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x48);
	} else if (total_gain < 0x1000) { /* 32x gain ~  64x gain */
		step = (total_gain - 0x800) >> 7;

		again = 0x7;
		again_fine = 0x1f;
		dgain = 0x1;
		dgain_fine = step * 8 + 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x80);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x48);
	} else if (total_gain < 0x2000) { /* 64x gain ~  128x gain */
		step = (total_gain - 0x1000) >> 8;

		again = 0x7;
		again_fine = 0x1f;
		dgain = 0x3;
		dgain_fine = step * 8 + 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x80);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x48);
	} else if (total_gain <= 0x4000) { /* 128x gain ~  256x gain */
		step = (total_gain - 0x2000) >> 9;
		step = (step >= 16) ? 0xf : step;

		again = 0x7;
		again_fine = 0x1f;
		dgain = 0x7;
		dgain_fine = step * 8 + 0x80;

		ret |= sc2232_write_reg(sc2232->client, 0x3301, SC2232_REG_VALUE_08BIT, 0x80);
		ret |= sc2232_write_reg(sc2232->client, 0x3632, SC2232_REG_VALUE_08BIT, 0x48);
	}

	ret |= sc2232_write_reg(sc2232->client, 0x3812, SC2232_REG_VALUE_08BIT, 0x30);

	dev_dbg(&sc2232->client->dev, "total_gain:%d again 0x%x, again_fine 0x%x, dgain 0x%x, dgain_fine 0x%x\n",
			 total_gain, again, again_fine, dgain, dgain_fine);

	ret |= sc2232_read_reg(sc2232->client, SC2232_REG_AGAIN,SC2232_REG_VALUE_08BIT, &val);
	ret |= sc2232_write_reg(sc2232->client, SC2232_REG_AGAIN,SC2232_REG_VALUE_08BIT, (val & 0xE3) | (again << 2));
	ret |= sc2232_read_reg(sc2232->client, SC2232_REG_DGAIN,SC2232_REG_VALUE_08BIT, &val);
	ret |= sc2232_write_reg(sc2232->client, SC2232_REG_DGAIN,SC2232_REG_VALUE_08BIT,(val & 0xF0) | dgain);

	ret |= sc2232_write_reg(sc2232->client, SC2232_REG_AGAIN_FINE, SC2232_REG_VALUE_08BIT, again_fine);
	ret |= sc2232_write_reg(sc2232->client, SC2232_REG_DGAIN_FINE, SC2232_REG_VALUE_08BIT, dgain_fine);
	return ret;
}

static long sc2232_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_HDR_CFG:
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc2232_get_module_inf(sc2232, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc2232->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc2232_write_reg(sc2232->client, SC2232_REG_CTRL_MODE,
				SC2232_REG_VALUE_08BIT, SC2232_MODE_STREAMING);
		else
			ret = sc2232_write_reg(sc2232->client, SC2232_REG_CTRL_MODE,
				SC2232_REG_VALUE_08BIT, SC2232_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc2232_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret = 0;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc2232_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = sc2232_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc2232_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = sc2232_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = sc2232_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = sc2232_ioctl(sd, cmd, &cg);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sc2232_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc2232_start_stream(struct sc2232 *sc2232)
{
	int ret;

	ret = sc2232_write_array(sc2232->client, sc2232->cur_mode->reg_list);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&sc2232->ctrl_handler);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	if (sc2232->has_init_exp && sc2232->cur_mode->hdr_mode != NO_HDR) {
		ret = sc2232_ioctl(&sc2232->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&sc2232->init_hdrae_exp);
		if (ret) {
			dev_err(&sc2232->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	return sc2232_write_reg(sc2232->client, SC2232_REG_CTRL_MODE,
		SC2232_REG_VALUE_08BIT, SC2232_MODE_STREAMING);
}

static int __sc2232_stop_stream(struct sc2232 *sc2232)
{
	sc2232->has_init_exp = false;
	return sc2232_write_reg(sc2232->client, SC2232_REG_CTRL_MODE,
		SC2232_REG_VALUE_08BIT, SC2232_MODE_SW_STANDBY);
}

static int sc2232_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	struct i2c_client *client = sc2232->client;
	int ret = 0;

	mutex_lock(&sc2232->mutex);
	on = !!on;
	if (on == sc2232->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc2232_start_stream(sc2232);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc2232_stop_stream(sc2232);
		pm_runtime_put(&client->dev);
	}

	sc2232->streaming = on;

unlock_and_return:
	mutex_unlock(&sc2232->mutex);

	return ret;
}

static int sc2232_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	struct i2c_client *client = sc2232->client;
	int ret = 0;

	mutex_lock(&sc2232->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc2232->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= sc2232_write_reg(sc2232->client,
			SC2232_SOFTWARE_RESET_REG,
			SC2232_REG_VALUE_08BIT,
			0x01);
		usleep_range(100, 200);

		sc2232->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc2232->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc2232->mutex);

	return ret;
}

static int __sc2232_power_on(struct sc2232 *sc2232)
{
	int ret;
	struct device *dev = &sc2232->client->dev;

	if (!IS_ERR_OR_NULL(sc2232->pins_default)) {
		ret = pinctrl_select_state(sc2232->pinctrl,
					   sc2232->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc2232->xvclk, SC2232_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (27MHz)\n");
	if (clk_get_rate(sc2232->xvclk) != SC2232_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(sc2232->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sc2232->reset_gpio))
		gpiod_set_value_cansleep(sc2232->reset_gpio, 1);

	ret = regulator_bulk_enable(SC2232_NUM_SUPPLIES, sc2232->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc2232->reset_gpio))
		gpiod_set_value_cansleep(sc2232->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(sc2232->pwdn_gpio))
		gpiod_set_value_cansleep(sc2232->pwdn_gpio, 1);
	usleep_range(2000, 4000);

	return 0;

disable_clk:
	clk_disable_unprepare(sc2232->xvclk);

	return ret;
}

static void __sc2232_power_off(struct sc2232 *sc2232)
{
	int ret;
	struct device *dev = &sc2232->client->dev;

	if (!IS_ERR(sc2232->pwdn_gpio))
		gpiod_set_value_cansleep(sc2232->pwdn_gpio, 0);
	clk_disable_unprepare(sc2232->xvclk);
	if (!IS_ERR(sc2232->reset_gpio))
		gpiod_set_value_cansleep(sc2232->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(sc2232->pins_sleep)) {
		ret = pinctrl_select_state(sc2232->pinctrl,
					   sc2232->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC2232_NUM_SUPPLIES, sc2232->supplies);
}

static int sc2232_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2232 *sc2232 = to_sc2232(sd);

	return __sc2232_power_on(sc2232);
}

static int sc2232_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2232 *sc2232 = to_sc2232(sd);

	__sc2232_power_off(sc2232);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc2232_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc2232 *sc2232 = to_sc2232(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc2232_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc2232->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc2232->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc2232_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc2232 *sc2232 = to_sc2232(sd);

	if (fie->index >= sc2232->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sc2232_pm_ops = {
	SET_RUNTIME_PM_OPS(sc2232_runtime_suspend,
			   sc2232_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc2232_internal_ops = {
	.open = sc2232_open,
};
#endif

static const struct v4l2_subdev_core_ops sc2232_core_ops = {
	.s_power = sc2232_s_power,
	.ioctl = sc2232_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc2232_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc2232_video_ops = {
	.s_stream = sc2232_s_stream,
	.g_frame_interval = sc2232_g_frame_interval,
	.s_frame_interval = sc2232_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc2232_pad_ops = {
	.enum_mbus_code = sc2232_enum_mbus_code,
	.enum_frame_size = sc2232_enum_frame_sizes,
	.enum_frame_interval = sc2232_enum_frame_interval,
	.get_fmt = sc2232_get_fmt,
	.set_fmt = sc2232_set_fmt,
	.get_mbus_config = sc2232_g_mbus_config,
};

static const struct v4l2_subdev_ops sc2232_subdev_ops = {
	.core	= &sc2232_core_ops,   /* v4l2_subdev_core_ops sc2232_core_ops */
	.video	= &sc2232_video_ops,  /* */
	.pad	= &sc2232_pad_ops,    /* */
};

static void sc2232_modify_fps_info(struct sc2232 *sc2232)
{
	const struct sc2232_mode *mode = sc2232->cur_mode;

	sc2232->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				      sc2232->cur_vts;
}

static int sc2232_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc2232 *sc2232 = container_of(ctrl->handler,
					     struct sc2232, ctrl_handler);
	struct i2c_client *client = sc2232->client;
	s64 max;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc2232->cur_mode->height + ctrl->val - 2;
		__v4l2_ctrl_modify_range(sc2232->exposure,
					 sc2232->exposure->minimum, max,
					 sc2232->exposure->step,
					 sc2232->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc2232->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end;
		val = ctrl->val << 1;
		ret = sc2232_write_reg(sc2232->client,
					SC2232_REG_EXP_LONG_L,
					SC2232_REG_VALUE_08BIT,
					(val << 4 & 0XF0));
		ret |= sc2232_write_reg(sc2232->client,
					SC2232_REG_EXP_LONG_M,
					SC2232_REG_VALUE_08BIT,
					(val >> 4 & 0XFF));
		ret |= sc2232_write_reg(sc2232->client,
					SC2232_REG_EXP_LONG_H,
					SC2232_REG_VALUE_08BIT,
					(val >> 12 & 0X0F));
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (sc2232->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end;
		ret = sc2232_set_gain(sc2232, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = sc2232_write_reg(sc2232->client, SC2232_REG_VTS,
					SC2232_REG_VALUE_16BIT,
					ctrl->val + sc2232->cur_mode->height);
		if (!ret)
			sc2232->cur_vts = ctrl->val + sc2232->cur_mode->height;
		sc2232_modify_fps_info(sc2232);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		break;
	case V4L2_CID_HFLIP:
		ret = sc2232_read_reg(sc2232->client, SC2232_FLIP_REG,
				      SC2232_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2232_MIRROR_MASK;
		else
			val &= ~SC2232_MIRROR_MASK;
		ret |= sc2232_write_reg(sc2232->client, SC2232_FLIP_REG,
					SC2232_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc2232_read_reg(sc2232->client, SC2232_FLIP_REG,
				      SC2232_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2232_FLIP_MASK;
		else
			val &= ~SC2232_FLIP_MASK;
		ret |= sc2232_write_reg(sc2232->client, SC2232_FLIP_REG,
					SC2232_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

ctrl_end:
	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops sc2232_ctrl_ops = {
	.s_ctrl = sc2232_set_ctrl,
};

static int sc2232_initialize_controls(struct sc2232 *sc2232)
{
	const struct sc2232_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 pixel_rate = 0;

	handler = &sc2232->ctrl_handler;
	mode = sc2232->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc2232->mutex;

	sc2232->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			ARRAY_SIZE(link_freq_items) - 1, 0,
			link_freq_items);
	__v4l2_ctrl_s_ctrl(sc2232->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * SC2232_LANES;
	sc2232->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_PIXEL_RATE, 0, SC2232_MAX_PIXEL_RATE,
		1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc2232->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (sc2232->hblank)
		sc2232->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sc2232->vblank = v4l2_ctrl_new_std(handler, &sc2232_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				SC2232_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max =  mode->vts_def - 2;
	sc2232->exposure = v4l2_ctrl_new_std(handler, &sc2232_ctrl_ops,
				V4L2_CID_EXPOSURE, SC2232_EXPOSURE_MIN,
				exposure_max, SC2232_EXPOSURE_STEP,
				mode->exp_def);

	sc2232->anal_gain = v4l2_ctrl_new_std(handler, &sc2232_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, SC2232_GAIN_MIN,
				SC2232_GAIN_MAX, SC2232_GAIN_STEP,
				SC2232_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &sc2232_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc2232_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc2232->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc2232->subdev.ctrl_handler = handler;
	sc2232->has_init_exp = false;
	sc2232->cur_vts = mode->vts_def;
	sc2232->cur_fps = mode->max_fps;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc2232_check_sensor_id(struct sc2232 *sc2232,
				  struct i2c_client *client)
{
	struct device *dev = &sc2232->client->dev;
	u32 id = 0;
	int ret;

	ret = sc2232_read_reg(client, SC2232_REG_CHIP_ID,
		SC2232_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC%04x sensor\n", CHIP_ID);
	return 0;
}

static int sc2232_configure_regulators(struct sc2232 *sc2232)
{
	unsigned int i;

	for (i = 0; i < SC2232_NUM_SUPPLIES; i++)
		sc2232->supplies[i].supply = sc2232_supply_names[i];

	return devm_regulator_bulk_get(&sc2232->client->dev,
				       SC2232_NUM_SUPPLIES,
				       sc2232->supplies);
}

static int sc2232_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc2232 *sc2232;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sc2232 = devm_kzalloc(dev, sizeof(*sc2232), GFP_KERNEL);
	if (!sc2232)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc2232->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc2232->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc2232->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc2232->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE,
			&hdr_mode);

	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	sc2232->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < sc2232->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sc2232->cur_mode = &supported_modes[i];
			break;
		}
	}
	sc2232->client = client;

	sc2232->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc2232->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc2232->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc2232->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc2232->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc2232->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc2232->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc2232->pinctrl)) {
		sc2232->pins_default =
			pinctrl_lookup_state(sc2232->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc2232->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc2232->pins_sleep =
			pinctrl_lookup_state(sc2232->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc2232->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc2232_configure_regulators(sc2232);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc2232->mutex);

	sd = &sc2232->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc2232_subdev_ops);
	ret = sc2232_initialize_controls(sc2232);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc2232_power_on(sc2232);
	if (ret)
		goto err_free_handler;

	ret = sc2232_check_sensor_id(sc2232, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc2232_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc2232->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc2232->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc2232->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc2232->module_index, facing,
		 SC2232_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
#ifdef USED_SYS_DEBUG
	add_sysfs_interfaces(dev);
#endif
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc2232_power_off(sc2232);
err_free_handler:
	v4l2_ctrl_handler_free(&sc2232->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc2232->mutex);

	return ret;
}

static int sc2232_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2232 *sc2232 = to_sc2232(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc2232->ctrl_handler);
	mutex_destroy(&sc2232->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc2232_power_off(sc2232);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc2232_of_match[] = {
	{ .compatible = "smartsens,sc2232" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc2232_of_match);
#endif

static const struct i2c_device_id sc2232_match_id[] = {
	{ "smartsens,sc2232", 0 },
	{ },
};

static struct i2c_driver sc2232_i2c_driver = {
	.driver = {
		.name = SC2232_NAME,
		.pm = &sc2232_pm_ops,
		.of_match_table = of_match_ptr(sc2232_of_match),
	},
	.probe		= &sc2232_probe,
	.remove		= &sc2232_remove,
	.id_table	= sc2232_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(sc2232_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc2232_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc2232_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("Smartsens sc2232 sensor driver");
MODULE_LICENSE("GPL v2");
