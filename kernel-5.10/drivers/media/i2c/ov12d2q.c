// SPDX-License-Identifier: GPL-2.0
/*
 * ov12d2q driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
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
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV12D2Q_LANES			4
#define OV12D2Q_BITS_PER_SAMPLE		10
#define MIPI_FREQ_625M			625000000LL
#define PIXEL_RATE_WITH_625M		(MIPI_FREQ_625M * 2 * \
					OV12D2Q_LANES / OV12D2Q_BITS_PER_SAMPLE)

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define OV12D2Q_XVCLK_FREQ		24000000

#define CHIP_ID				0x561441
#define OV12D2Q_REG_CHIP_ID		0x300a

#define OV12D2Q_REG_CTRL_MODE		0x0100
#define OV12D2Q_MODE_SW_STANDBY		0x0
#define OV12D2Q_MODE_STREAMING		BIT(0)

#define	OV12D2Q_EXPOSURE_MIN		8
#define	OV12D2Q_EXPOSURE_STEP		1
#define OV12D2Q_VTS_MAX			0xffff

#define OV12D2Q_GAIN_MIN		0x80
#define OV12D2Q_GAIN_MAX		0x3df61
#define OV12D2Q_GAIN_STEP		1
#define OV12D2Q_GAIN_DEFAULT		0x80

#define OV12D2Q_REG_AGAIN_L		0x3508
#define OV12D2Q_REG_AGAIN_M		0x3548
#define OV12D2Q_REG_AGAIN_S		0x3588
#define OV12D2Q_REG_DGAIN_L_H_B		0x350A
#define OV12D2Q_REG_DGAIN_L_H_GB	0x3510
#define OV12D2Q_REG_DGAIN_L_H_GR	0x3513
#define OV12D2Q_REG_DGAIN_L_H_R		0x3516

#define OV12D2Q_REG_DGAIN_M_H_B		0x354A

#define OV12D2Q_REG_DGAIN_S_H_B		0x358A
#define OV12D2Q_REG_DGAIN_S_H_GB	0x3590
#define OV12D2Q_REG_DGAIN_S_H_GR	0x3593
#define OV12D2Q_REG_DGAIN_S_H_R		0x3596

#define OV12D2Q_REG_EXP_L_H		0x3501
#define OV12D2Q_REG_EXP_M_H		0x3541
#define OV12D2Q_REG_EXP_S_H		0x3581

#define OV12D2Q_SOFTWARE_RESET_REG	0x0103
#define OV12D2Q_REG_ISP_X_WIN		0x3810
#define OV12D2Q_REG_ISP_Y_WIN		0x3812

#define OV12D2Q_GROUP_UPDATE_ADDRESS	0x3208
#define OV12D2Q_GROUP_UPDATE_START_DATA	0x00
#define OV12D2Q_GROUP_UPDATE_END_DATA	0x10
#define OV12D2Q_GROUP_UPDATE_LAUNCH	0xA0

#define OV12D2Q_REG_TEST_PATTERN	0x5081
#define OV12D2Q_TEST_PATTERN_ENABLE	0x01
#define OV12D2Q_TEST_PATTERN_DISABLE	0x0

#define OV12D2Q_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define OV12D2Q_REG_VALUE_08BIT		1
#define OV12D2Q_REG_VALUE_16BIT		2
#define OV12D2Q_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OV12D2Q_NAME			"ov12d2q"

static const char * const ov12d2q_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV12D2Q_NUM_SUPPLIES ARRAY_SIZE(ov12d2q_supply_names)

#define OV12D2Q_FLIP_REG		0x3820
#define OV12D2Q_MIRROR_REG		0x3821
#define MIRROR_BIT_MASK			BIT(2)
#define FLIP_BIT_MASK			BIT(2)

struct regval {
	u16 addr;
	u8 val;
};

struct ov12d2q_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
	u32 link_freq_idx;
	u32 bpp;
};

struct ov12d2q {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV12D2Q_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov12d2q_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			is_thunderboot;
	bool			is_thunderboot_ng;
	bool			is_first_streamoff;
	u8			flip;
	struct v4l2_fract	cur_fps;
	u32			cur_vts;
};

#define to_ov12d2q(sd) container_of(sd, struct ov12d2q, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval ov12d2q_global_regs[] = {
	{REG_NULL, 0x00},
};


/*
 * Xclk 24Mhz
 * max_framerate 60fps
 */
static const struct regval ov12d2q_2256x1256_regs[] = {
	{0x0301, 0xc0},
	{0x0302, 0x01},
	{0x0303, 0x06},
	{0x0304, 0x02},
	{0x0305, 0x71},
	{0x0306, 0x04},
	{0x0307, 0x00},
	{0x0309, 0x01},
	{0x0320, 0x20},
	{0x0323, 0x06},
	{0x0324, 0x01},
	{0x0325, 0xc2},
	{0x0326, 0xd3},
	{0x032b, 0x06},
	{0x0343, 0x06},
	{0x0344, 0x01},
	{0x0345, 0xb8},
	{0x0346, 0xcb},
	{0x0350, 0x02},
	{0x0360, 0x09},
	{0x3002, 0x80},
	{0x300d, 0x11},
	{0x300e, 0x11},
	{0x3012, 0x41},
	{0x3016, 0xf0},
	{0x3017, 0xd0},
	{0x3018, 0xf0},
	{0x3019, 0xc2},
	{0x301a, 0xf0},
	{0x301b, 0x34},
	{0x301c, 0x91},
	{0x301d, 0x02},
	{0x301e, 0x98},
	{0x301f, 0x21},
	{0x3022, 0xf0},
	{0x3027, 0x2e},
	{0x302c, 0x01},
	{0x302d, 0x00},
	{0x302e, 0x00},
	{0x302f, 0x00},
	{0x3030, 0x03},
	{0x3044, 0xc2},
	{0x304b, 0x00},
	{0x30d4, 0x00},
	{0x3209, 0x00},
	{0x320a, 0x00},
	{0x320b, 0x00},
	{0x320c, 0x00},
	{0x320d, 0x01},
	{0x3216, 0x01},
	{0x3218, 0x80},
	{0x33c0, 0x00},
	{0x33c3, 0x00},
	{0x33c4, 0x00},
	{0x3400, 0x04},
	{0x3408, 0x05},
	{0x340c, 0x10},
	{0x340e, 0x30},
	{0x3421, 0x08},
	{0x3422, 0x00},
	{0x3423, 0x15},
	{0x3424, 0x40},
	{0x3425, 0x10},
	{0x3426, 0x20},
	{0x3500, 0x00},
	{0x3501, 0x05},
	{0x3502, 0x60},
	{0x3504, 0x08},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x350a, 0x01},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350e, 0x00},
	{0x3510, 0x01},
	{0x3511, 0x00},
	{0x3512, 0x00},
	{0x3513, 0x01},
	{0x3514, 0x00},
	{0x3515, 0x00},
	{0x3516, 0x01},
	{0x3517, 0x00},
	{0x3518, 0x00},
	{0x352d, 0x00},
	{0x352e, 0x00},
	{0x352f, 0x00},
	{0x3541, 0x00},
	{0x3542, 0x40},
	{0x3548, 0x01},
	{0x3549, 0x00},
	{0x354a, 0x01},
	{0x354b, 0x00},
	{0x354c, 0x00},
	{0x354e, 0x80},
	{0x3550, 0x01},
	{0x3551, 0x00},
	{0x3552, 0x00},
	{0x3581, 0x00},
	{0x3582, 0x40},
	{0x3588, 0x01},
	{0x3589, 0x00},
	{0x358a, 0x01},
	{0x358b, 0x00},
	{0x358c, 0x00},
	{0x3590, 0x01},
	{0x3591, 0x00},
	{0x3592, 0x00},
	{0x3610, 0x80},
	{0x3615, 0x27},
	{0x3617, 0x5a},
	{0x3624, 0x88},
	{0x3628, 0x77},
	{0x3644, 0x20},
	{0x3652, 0x00},
	{0x3653, 0x00},
	{0x3663, 0x6b},
	{0x3660, 0x4f},
	{0x3661, 0xd0},
	{0x3662, 0x09},
	{0x3680, 0xc1},
	{0x3683, 0x80},
	{0x3684, 0x03},
	{0x3685, 0x52},
	{0x3687, 0xd2},
	{0x3689, 0x27},
	{0x368a, 0x38},
	{0x368b, 0x08},
	{0x368c, 0x06},
	{0x368e, 0x00},
	{0x3692, 0x00},
	{0x3693, 0x00},
	{0x3696, 0x26},
	{0x3697, 0x1f},
	{0x3698, 0x1d},
	{0x3699, 0x59},
	{0x369a, 0x01},
	{0x369b, 0x20},
	{0x3700, 0x2e},
	{0x3701, 0x06},
	{0x3702, 0x4f},
	{0x3703, 0x28},
	{0x3704, 0x07},
	{0x3705, 0x00},
	{0x3706, 0x2f},
	{0x3707, 0x08},
	{0x3708, 0x2d},
	{0x3709, 0x5d},
	{0x370a, 0x00},
	{0x370b, 0x69},
	{0x370c, 0x0c},
	{0x3711, 0x30},
	{0x3712, 0x00},
	{0x3713, 0x00},
	{0x3714, 0x63},
	{0x371a, 0x1c},
	{0x371b, 0xd0},
	{0x371c, 0x04},
	{0x371d, 0x24},
	{0x371e, 0x13},
	{0x371f, 0x0c},
	{0x3720, 0x08},
	{0x3721, 0x15},
	{0x3724, 0x08},
	{0x3725, 0x32},
	{0x3727, 0x22},
	{0x3728, 0x11},
	{0x3729, 0x00},
	{0x372a, 0x00},
	{0x372b, 0x00},
	{0x3752, 0x02},
	{0x3753, 0x03},
	{0x3754, 0xee},
	{0x3760, 0x04},
	{0x3761, 0x14},
	{0x3762, 0x04},
	{0x3765, 0x08},
	{0x3766, 0x0c},
	{0x3767, 0x00},
	{0x376a, 0x00},
	{0x376b, 0x00},
	{0x376d, 0x1b},
	{0x376f, 0x02},
	{0x37d9, 0x08},
	{0x37f6, 0x07},
	{0x37f7, 0x04},
	{0x37f8, 0x2d},
	{0x37f9, 0x02},
	{0x37fa, 0x02},
	{0x37fb, 0x02},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x12},
	{0x3805, 0x1f},
	{0x3806, 0x0a},
	{0x3807, 0x1f},
	{0x3808, 0x08},
	{0x3809, 0xd0},
	{0x380a, 0x04},
	{0x380b, 0xe8},
	{0x380c, 0x02},
	{0x380d, 0x1b},
	{0x380e, 0x05},
	{0x380f, 0x70},
	{0x3810, 0x00},
	{0x3811, 0x21},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x22},
	{0x3815, 0x22},
	{0x381a, 0x00},
	{0x381b, 0x01},
	{0x381e, 0x00},
	{0x381f, 0x02},
	{0x3820, 0x01},
	{0x3821, 0x0d},
	{0x3822, 0x00},
	{0x3823, 0x04},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x64},
	{0x3828, 0xf7},
	{0x382a, 0x83},
	{0x382c, 0x00},
	{0x382d, 0x00},
	{0x3835, 0x00},
	{0x3836, 0x00},
	{0x3837, 0x08},
	{0x3839, 0x00},
	{0x383b, 0x00},
	{0x383c, 0x00},
	{0x383d, 0x08},
	{0x383e, 0x00},
	{0x383f, 0x33},
	{0x3842, 0x00},
	{0x3856, 0x00},
	{0x3857, 0x08},
	{0x3858, 0x00},
	{0x3859, 0x10},
	{0x3865, 0x70},
	{0x3867, 0x08},
	{0x3868, 0x00},
	{0x3904, 0x33},
	{0x3907, 0x33},
	{0x390a, 0x9a},
	{0x3914, 0x34},
	{0x3938, 0x4b},
	{0x3939, 0x0c},
	{0x393b, 0x4b},
	{0x393c, 0x0c},
	{0x393e, 0x40},
	{0x393f, 0x0c},
	{0x3975, 0x05},
	{0x3979, 0x32},
	{0x397d, 0x69},
	{0x3981, 0x15},
	{0x3983, 0x33},
	{0x3985, 0x1a},
	{0x3986, 0x08},
	{0x398a, 0x09},
	{0x39cd, 0x00},
	{0x39ce, 0x24},
	{0x39cf, 0x40},
	{0x39d0, 0x0a},
	{0x39d1, 0x50},
	{0x39d2, 0x05},
	{0x39d3, 0x94},
	{0x39d4, 0x01},
	{0x39d5, 0x79},
	{0x3a12, 0x00},
	{0x3a13, 0x00},
	{0x3a14, 0x00},
	{0x3a15, 0x00},
	{0x3a16, 0x00},
	{0x3a18, 0x04},
	{0x3a1a, 0x05},
	{0x3a1c, 0x0a},
	{0x3a1e, 0x03},
	{0x3a1f, 0x34},
	{0x3a22, 0x12},
	{0x3a24, 0x00},
	{0x3a25, 0xfe},
	{0x3a26, 0x01},
	{0x3a27, 0x01},
	{0x3a2a, 0xa8},
	{0x3a2b, 0xa8},
	{0x3a36, 0x00},
	{0x3d84, 0x00},
	{0x3d85, 0x1b},
	{0x3d88, 0x00},
	{0x3d89, 0x00},
	{0x3d8a, 0x03},
	{0x3d8b, 0xff},
	{0x3d8c, 0xa3},
	{0x3d8d, 0xc4},
	{0x3da4, 0x04},
	{0x3daa, 0xa0},
	{0x3dab, 0x10},
	{0x3dac, 0xa1},
	{0x3dad, 0x8c},
	{0x3dae, 0xa1},
	{0x3daf, 0xb3},
	{0x3e00, 0x0e},
	{0x3e01, 0x0e},
	{0x3e02, 0x0e},
	{0x3e03, 0x0e},
	{0x3e04, 0x0e},
	{0x3e05, 0x0e},
	{0x3e06, 0x0e},
	{0x3e07, 0x0e},
	{0x3e09, 0x47},
	{0x3e0b, 0x25},
	{0x3e0d, 0x13},
	{0x3e0f, 0x09},
	{0x3e11, 0x07},
	{0x3e13, 0x06},
	{0x3e15, 0x05},
	{0x3e17, 0x04},
	{0x3e18, 0x38},
	{0x3e19, 0x38},
	{0x3e1a, 0x13},
	{0x3e1b, 0x30},
	{0x3e1c, 0x07},
	{0x3e1d, 0x06},
	{0x3e1e, 0x05},
	{0x3e1f, 0x04},
	{0x3e20, 0x0f},
	{0x3e21, 0x0f},
	{0x3e22, 0x0f},
	{0x3e23, 0x0f},
	{0x3e24, 0x0f},
	{0x3e25, 0x0f},
	{0x3e26, 0x0f},
	{0x3e27, 0x0f},
	{0x3e28, 0x07},
	{0x3e29, 0x07},
	{0x3e2a, 0x07},
	{0x3e2b, 0x02},
	{0x3e2c, 0x07},
	{0x3e2d, 0x07},
	{0x3e30, 0x07},
	{0x3e3a, 0x02},
	{0x3e3b, 0xdf},
	{0x3e3c, 0xff},
	{0x3e3d, 0x44},
	{0x3e3e, 0x00},
	{0x3e3f, 0x00},
	{0x3e40, 0xc1},
	{0x3e42, 0x54},
	{0x3e43, 0x54},
	{0x3e44, 0x54},
	{0x3e45, 0x54},
	{0x3f00, 0x10},
	{0x3f01, 0x26},
	{0x3f03, 0x40},
	{0x4002, 0xf3},
	{0x4009, 0x02},
	{0x400e, 0xc6},
	{0x400f, 0x00},
	{0x4010, 0x28},
	{0x4011, 0x01},
	{0x4012, 0x0d},
	{0x4015, 0x02},
	{0x4016, 0x11},
	{0x4017, 0x00},
	{0x4018, 0x03},
	{0x401a, 0x40},
	{0x401e, 0x00},
	{0x401f, 0xcc},
	{0x4020, 0x04},
	{0x4021, 0x00},
	{0x4022, 0x04},
	{0x4023, 0x00},
	{0x4024, 0x04},
	{0x4025, 0x00},
	{0x4026, 0x04},
	{0x4027, 0x00},
	{0x4028, 0x01},
	{0x4030, 0x00},
	{0x4031, 0x10},
	{0x4032, 0x00},
	{0x4033, 0x10},
	{0x4034, 0x08},
	{0x4035, 0x10},
	{0x4036, 0x08},
	{0x4037, 0x10},
	{0x4040, 0x08},
	{0x4041, 0x10},
	{0x4042, 0x08},
	{0x4043, 0x10},
	{0x4044, 0x00},
	{0x4045, 0x10},
	{0x4046, 0x00},
	{0x4047, 0x10},
	{0x4050, 0x00},
	{0x4051, 0x00},
	{0x4056, 0x25},
	{0x4102, 0xf3},
	{0x4109, 0x02},
	{0x410e, 0xc6},
	{0x410f, 0x00},
	{0x4110, 0x28},
	{0x4111, 0x01},
	{0x4112, 0x0d},
	{0x4115, 0x04},
	{0x4116, 0x1b},
	{0x4117, 0x00},
	{0x4118, 0x07},
	{0x411a, 0x40},
	{0x411e, 0x00},
	{0x411f, 0xcc},
	{0x4128, 0x01},
	{0x4156, 0x25},
	{0x4702, 0xf3},
	{0x4709, 0x02},
	{0x470e, 0xc6},
	{0x470f, 0x00},
	{0x4710, 0x28},
	{0x4711, 0x01},
	{0x4712, 0x0d},
	{0x4715, 0x04},
	{0x4716, 0x1b},
	{0x4717, 0x00},
	{0x4718, 0x07},
	{0x471a, 0x40},
	{0x471e, 0x00},
	{0x471f, 0xcc},
	{0x4728, 0x01},
	{0x4756, 0x25},
	{0x4301, 0x00},
	{0x4303, 0x00},
	{0x4305, 0x00},
	{0x4307, 0x00},
	{0x4308, 0x00},
	{0x430b, 0xff},
	{0x430d, 0x00},
	{0x430e, 0x00},
	{0x4503, 0x0f},
	{0x4504, 0x82},
	{0x4508, 0x00},
	{0x451d, 0x00},
	{0x451e, 0x00},
	{0x451f, 0x00},
	{0x4523, 0x00},
	{0x4526, 0x00},
	{0x4527, 0x00},
	{0x4530, 0x00},
	{0x4547, 0x06},
	{0x4640, 0x00},
	{0x4641, 0x30},
	{0x4643, 0x00},
	{0x4645, 0x13},
	{0x464a, 0x00},
	{0x464b, 0x30},
	{0x4680, 0x00},
	{0x4681, 0x24},
	{0x4683, 0x0c},
	{0x4800, 0x64},
	{0x480b, 0x10},
	{0x480c, 0x80},
	{0x480e, 0x04},
	{0x480f, 0x32},
	{0x4826, 0x32},
	{0x4833, 0x18},
	{0x4837, 0x06},
	{0x484b, 0x27},
	{0x4850, 0x47},
	{0x4860, 0x00},
	{0x4861, 0xec},
	{0x4862, 0x04},
	{0x4883, 0x00},
	{0x4885, 0x10},
	{0x4888, 0x10},
	{0x4889, 0x03},
	{0x4d00, 0x04},
	{0x4d01, 0x8f},
	{0x4d02, 0xb9},
	{0x4d03, 0xc1},
	{0x4d04, 0xb6},
	{0x4d05, 0x7e},
	{0x5000, 0xeb},
	{0x5001, 0xcb},
	{0x5002, 0x15},
	{0x5003, 0x01},
	{0x5007, 0x1e},
	{0x5008, 0x00},
	{0x5009, 0x00},
	{0x500a, 0x00},
	{0x500b, 0x30},
	{0x500c, 0x12},
	{0x500d, 0x1f},
	{0x500e, 0x0a},
	{0x500f, 0x0f},
	{0x504b, 0x40},
	{0x5081, 0x00},
	{0x50c4, 0xaa},
	{0x50d0, 0x00},
	{0x50d1, 0x10},
	{0x50d2, 0x01},
	{0x50d3, 0xb3},
	{0x515a, 0x06},
	{0x515b, 0x06},
	{0x515c, 0x02},
	{0x515d, 0x02},
	{0x515e, 0x02},
	{0x515f, 0x06},
	{0x5160, 0x0a},
	{0x5161, 0x0e},
	{0x5180, 0x09},
	{0x5181, 0x10},
	{0x5182, 0x05},
	{0x5183, 0x20},
	{0x5184, 0x00},
	{0x5185, 0x10},
	{0x5186, 0x00},
	{0x5187, 0x10},
	{0x5188, 0x12},
	{0x5189, 0x00},
	{0x518a, 0x0a},
	{0x518b, 0x20},
	{0x518d, 0x09},
	{0x5192, 0x00},
	{0x5193, 0x18},
	{0x51d2, 0x10},
	{0x51da, 0x00},
	{0x51db, 0x30},
	{0x5250, 0x8e},
	{0x5251, 0x00},
	{0x5252, 0x10},
	{0x5254, 0x00},
	{0x5255, 0x70},
	{0x5256, 0x00},
	{0x5257, 0xc7},
	{0x5258, 0x12},
	{0x5259, 0x10},
	{0x525a, 0x0a},
	{0x525b, 0x30},
	{0x525e, 0x00},
	{0x525f, 0x18},
	{0x5260, 0x00},
	{0x5261, 0x10},
	{0x5262, 0x00},
	{0x5263, 0x10},
	{0x5264, 0x12},
	{0x5265, 0x00},
	{0x5266, 0x0a},
	{0x5267, 0x20},
	{0x5268, 0x00},
	{0x5269, 0x00},
	{0x526a, 0x00},
	{0x526b, 0x00},
	{0x526c, 0x12},
	{0x526d, 0x10},
	{0x526e, 0x0a},
	{0x526f, 0x30},
	{0x5278, 0x08},
	{0x5279, 0x10},
	{0x527a, 0x00},
	{0x527b, 0x00},
	{0x527c, 0x06},
	{0x527d, 0x06},
	{0x527e, 0x02},
	{0x527f, 0x02},
	{0x5280, 0x02},
	{0x5281, 0x06},
	{0x5282, 0x0a},
	{0x5283, 0x0e},
	{0x5381, 0x00},
	{0x53c4, 0xaa},
	{0x545a, 0x06},
	{0x545b, 0x06},
	{0x545c, 0x02},
	{0x545d, 0x02},
	{0x545e, 0x02},
	{0x545f, 0x06},
	{0x5460, 0x0a},
	{0x5461, 0x0e},
	{0x5480, 0x09},
	{0x5481, 0x10},
	{0x5482, 0x05},
	{0x5483, 0x20},
	{0x5484, 0x00},
	{0x5485, 0x08},
	{0x5486, 0x00},
	{0x5487, 0x00},
	{0x5488, 0x09},
	{0x5489, 0x00},
	{0x548a, 0x05},
	{0x548b, 0x20},
	{0x548d, 0x09},
	{0x54d2, 0x10},
	{0x54da, 0x00},
	{0x54da, 0x00},
	{0x54db, 0x30},
	{0x54db, 0x30},
	{0x5550, 0xec},
	{0x5551, 0x00},
	{0x5552, 0x10},
	{0x5554, 0x00},
	{0x5555, 0x72},
	{0x5556, 0x00},
	{0x5557, 0xca},
	{0x5558, 0x12},
	{0x5559, 0x10},
	{0x555a, 0x0a},
	{0x555b, 0x30},
	{0x555e, 0x00},
	{0x555f, 0x18},
	{0x5560, 0x00},
	{0x5561, 0x00},
	{0x5562, 0x00},
	{0x5563, 0x08},
	{0x5564, 0x12},
	{0x5565, 0x10},
	{0x5566, 0x0a},
	{0x5567, 0x30},
	{0x5568, 0x00},
	{0x5569, 0x00},
	{0x556a, 0x00},
	{0x556b, 0x00},
	{0x556c, 0x12},
	{0x556d, 0x10},
	{0x556e, 0x0a},
	{0x556f, 0x30},
	{0x557c, 0x06},
	{0x557d, 0x06},
	{0x557e, 0x02},
	{0x557f, 0x02},
	{0x5580, 0x02},
	{0x5581, 0x06},
	{0x5582, 0x0a},
	{0x5583, 0x0e},
	{0x5681, 0x00},
	{0x56c4, 0xaa},
	{0x575a, 0x06},
	{0x575b, 0x06},
	{0x575c, 0x02},
	{0x575d, 0x02},
	{0x575e, 0x02},
	{0x575f, 0x06},
	{0x5760, 0x0a},
	{0x5761, 0x0e},
	{0x5780, 0x09},
	{0x5781, 0x10},
	{0x5782, 0x05},
	{0x5783, 0x20},
	{0x5784, 0x00},
	{0x5785, 0x08},
	{0x5786, 0x00},
	{0x5787, 0x00},
	{0x5788, 0x09},
	{0x5789, 0x00},
	{0x578a, 0x05},
	{0x578b, 0x20},
	{0x578d, 0x09},
	{0x5792, 0x00},
	{0x5793, 0x18},
	{0x57d2, 0x10},
	{0x57da, 0x00},
	{0x57db, 0x30},
	{0x5850, 0xec},
	{0x5851, 0x00},
	{0x5852, 0x10},
	{0x5854, 0x00},
	{0x5855, 0x72},
	{0x5856, 0x00},
	{0x5857, 0xca},
	{0x5858, 0x12},
	{0x5859, 0x10},
	{0x585a, 0x0a},
	{0x585b, 0x30},
	{0x585e, 0x00},
	{0x585f, 0x18},
	{0x5860, 0x00},
	{0x5861, 0x00},
	{0x5862, 0x00},
	{0x5863, 0x08},
	{0x5864, 0x12},
	{0x5865, 0x10},
	{0x5866, 0x0a},
	{0x5867, 0x30},
	{0x5868, 0x00},
	{0x5869, 0x00},
	{0x586a, 0x00},
	{0x586b, 0x00},
	{0x586c, 0x12},
	{0x586d, 0x10},
	{0x586e, 0x0a},
	{0x586f, 0x30},
	{0x587c, 0x06},
	{0x587d, 0x06},
	{0x587e, 0x02},
	{0x587f, 0x02},
	{0x5880, 0x02},
	{0x5881, 0x06},
	{0x5882, 0x0a},
	{0x5883, 0x0e},
	{0x5f06, 0x10},
	{0x5f07, 0x20},
	{0x5f08, 0x0c},
	{0x5f09, 0x0c},
	{0x5f0a, 0x04},
	{0x5f0b, 0x04},
	{0x5f0c, 0x04},
	{0x5f0d, 0x0c},
	{0x5f0e, 0x14},
	{0x5f0f, 0x1c},
	{0x5f10, 0x01},
	{0x5f11, 0x01},
	{0x5f18, 0x12},
	{0x5f19, 0x20},
	{0x5f1a, 0x0a},
	{0x5f1b, 0x40},
	{0x5f1d, 0x10},
	{0x5f1f, 0x00},
	{0x5f20, 0x12},
	{0x5f21, 0x00},
	{0x5f22, 0x0a},
	{0x5f23, 0x40},
	{0x5f25, 0x09},
	{0x5f2a, 0x00},
	{0x5f2b, 0x18},
	{0x6600, 0x00},
	{0x6601, 0x00},
	{0x6602, 0x00},
	{0x6603, 0x83},
	{0x6960, 0x0f},
	{0x69a2, 0x09},
	{0x69a3, 0x00},
	{0x69a6, 0x05},
	{0x69a7, 0x10},
	{0x69aa, 0x09},
	{0x69ab, 0x00},
	{0x69ae, 0x05},
	{0x69af, 0x10},
	{0x69b2, 0x09},
	{0x69b3, 0x00},
	{0x69b6, 0x05},
	{0x69b7, 0x10},
	{0x69ba, 0x09},
	{0x69bb, 0x00},
	{0x69be, 0x05},
	{0x69bf, 0x10},
	{0x6a24, 0x09},
	{0x6a25, 0x00},
	{0x6a2a, 0x05},
	{0x6a2b, 0x10},
	{0x6a61, 0x40},
	{0x6a64, 0x09},
	{0x6a65, 0x00},
	{0x6a6a, 0x05},
	{0x6a6b, 0x10},
	{0x6a23, 0x00},
	{0x6a27, 0x00},
	{0x6a63, 0x00},
	{0x6a67, 0x00},
	{0x69a1, 0x00},
	{0x69a5, 0x00},
	{0x69a9, 0x00},
	{0x69ad, 0x00},
	{0x69b1, 0x00},
	{0x69b5, 0x00},
	{0x69b9, 0x00},
	{0x69bd, 0x00},
	{0xfff4, 0x01},
	{0xfff6, 0x00},
	{0x0361, 0x07},
	{0x3644, 0x20},
	{0x5000, 0x2b},
	{0x5001, 0x0b},
	{0x50d4, 0x00},
	{0x5171, 0xbe},
	{0x3222, 0x03},
	{0x3208, 0x06},
	{0x3938, 0x41},
	{0x393b, 0x41},
	{0x3208, 0x16},
	{0x3208, 0x07},
	{0x3938, 0x43},
	{0x393b, 0x44},
	{0x3208, 0x17},
	{0x3208, 0x08},
	{0x3938, 0x45},
	{0x393b, 0x46},
	{0x3208, 0x18},
	{0x3208, 0x09},
	{0x3938, 0x4b},
	{0x393b, 0x4b},
	{0x3208, 0x19},
	{0x5000, 0x29},
	{0x5001, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 */
static const struct regval ov12d2q_4512x2512_regs[] = {
	{0x0301, 0xc0},
	{0x0302, 0x01},
	{0x0303, 0x06},
	{0x0304, 0x02},
	{0x0305, 0x71},
	{0x0306, 0x04},
	{0x0307, 0x00},
	{0x0309, 0x01},
	{0x0320, 0x20},
	{0x0323, 0x06},
	{0x0324, 0x01},
	{0x0325, 0xc2},
	{0x0326, 0xd3},
	{0x032b, 0x06},
	{0x0343, 0x06},
	{0x0344, 0x01},
	{0x0345, 0xb8},
	{0x0346, 0xcb},
	{0x0350, 0x02},
	{0x0360, 0x09},
	{0x3002, 0x80},
	{0x300d, 0x11},
	{0x300e, 0x11},
	{0x3012, 0x41},
	{0x3016, 0xf0},
	{0x3017, 0xd0},
	{0x3018, 0xf0},
	{0x3019, 0xc2},
	{0x301a, 0xf0},
	{0x301b, 0x34},
	{0x301c, 0x91},
	{0x301d, 0x02},
	{0x301e, 0x98},
	{0x301f, 0x21},
	{0x3022, 0xf0},
	{0x3027, 0x2e},
	{0x302c, 0x01},
	{0x302d, 0x00},
	{0x302e, 0x00},
	{0x302f, 0x00},
	{0x3030, 0x03},
	{0x3044, 0xc2},
	{0x304b, 0x00},
	{0x30d4, 0x00},
	{0x3209, 0x00},
	{0x320a, 0x00},
	{0x320b, 0x00},
	{0x320c, 0x00},
	{0x320d, 0x01},
	{0x3216, 0x01},
	{0x3218, 0x80},
	{0x33c0, 0x00},
	{0x33c3, 0x00},
	{0x33c4, 0x00},
	{0x3400, 0x04},
	{0x3408, 0x05},
	{0x340c, 0x10},
	{0x340e, 0x30},
	{0x3421, 0x08},
	{0x3422, 0x00},
	{0x3423, 0x15},
	{0x3424, 0x40},
	{0x3425, 0x10},
	{0x3426, 0x20},
	{0x3500, 0x00},
	{0x3501, 0x0a},
	{0x3502, 0xd0},
	{0x3504, 0x08},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x350a, 0x01},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350e, 0x00},
	{0x3510, 0x01},
	{0x3511, 0x00},
	{0x3512, 0x00},
	{0x3513, 0x01},
	{0x3514, 0x00},
	{0x3515, 0x00},
	{0x3516, 0x01},
	{0x3517, 0x00},
	{0x3518, 0x00},
	{0x352d, 0x00},
	{0x352e, 0x00},
	{0x352f, 0x00},
	{0x3541, 0x00},
	{0x3542, 0x40},
	{0x3548, 0x01},
	{0x3549, 0x00},
	{0x354a, 0x01},
	{0x354b, 0x00},
	{0x354c, 0x00},
	{0x354e, 0x80},
	{0x3550, 0x01},
	{0x3551, 0x00},
	{0x3552, 0x00},
	{0x3581, 0x00},
	{0x3582, 0x40},
	{0x3588, 0x01},
	{0x3589, 0x00},
	{0x358a, 0x01},
	{0x358b, 0x00},
	{0x358c, 0x00},
	{0x3590, 0x01},
	{0x3591, 0x00},
	{0x3592, 0x00},
	{0x3610, 0x80},
	{0x3615, 0x27},
	{0x3617, 0x5a},
	{0x3624, 0x88},
	{0x3628, 0x77},
	{0x3644, 0x20},
	{0x3652, 0x00},
	{0x3653, 0x00},
	{0x3663, 0x6b},
	{0x3660, 0x4f},
	{0x3661, 0xd0},
	{0x3662, 0x09},
	{0x3680, 0xc1},
	{0x3683, 0x80},
	{0x3684, 0x03},
	{0x3685, 0x52},
	{0x3687, 0xc2},
	{0x3689, 0x27},
	{0x368a, 0x38},
	{0x368b, 0x08},
	{0x368c, 0x06},
	{0x368e, 0x00},
	{0x3692, 0x00},
	{0x3693, 0x00},
	{0x3696, 0x26},
	{0x3697, 0x1f},
	{0x3698, 0x1d},
	{0x3699, 0x59},
	{0x369a, 0x02},
	{0x369b, 0x34},
	{0x3700, 0x2e},
	{0x3701, 0x06},
	{0x3702, 0x4f},
	{0x3703, 0x28},
	{0x3704, 0x07},
	{0x3705, 0x00},
	{0x3706, 0x2f},
	{0x3707, 0x08},
	{0x3708, 0x2d},
	{0x3709, 0x5d},
	{0x370a, 0x00},
	{0x370b, 0x69},
	{0x370c, 0x0c},
	{0x3711, 0x00},
	{0x3712, 0x01},
	{0x3713, 0x00},
	{0x3714, 0x69},
	{0x371a, 0x1c},
	{0x371b, 0xd0},
	{0x371c, 0x04},
	{0x371d, 0x24},
	{0x371e, 0x13},
	{0x371f, 0x0c},
	{0x3720, 0x08},
	{0x3721, 0x15},
	{0x3724, 0x08},
	{0x3725, 0x32},
	{0x3727, 0x22},
	{0x3728, 0x11},
	{0x3729, 0x00},
	{0x372a, 0x00},
	{0x372b, 0x00},
	{0x3752, 0x02},
	{0x3753, 0x03},
	{0x3754, 0xee},
	{0x3760, 0x04},
	{0x3761, 0x14},
	{0x3762, 0x04},
	{0x3765, 0x08},
	{0x3766, 0x0c},
	{0x3767, 0x00},
	{0x376a, 0x00},
	{0x376b, 0x00},
	{0x376d, 0x1b},
	{0x376f, 0x02},
	{0x37d9, 0x08},
	{0x37f6, 0x07},
	{0x37f7, 0x04},
	{0x37f8, 0x2d},
	{0x37f9, 0x02},
	{0x37fa, 0x02},
	{0x37fb, 0x02},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x12},
	{0x3805, 0x1f},
	{0x3806, 0x0a},
	{0x3807, 0x1f},
	{0x3808, 0x11},
	{0x3809, 0xa0},
	{0x380a, 0x09},
	{0x380b, 0xd0},
	{0x380c, 0x02},
	{0x380d, 0x1c},
	{0x380e, 0x0a},
	{0x380f, 0xe0},
	{0x3810, 0x00},
	{0x3811, 0x21},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x381a, 0x00},
	{0x381b, 0x01},
	{0x381e, 0x00},
	{0x381f, 0x02},
	{0x3820, 0x00},
	{0x3821, 0x04},
	{0x3822, 0x00},
	{0x3823, 0x04},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x3828, 0xf7},
	{0x382a, 0x83},
	{0x382c, 0x00},
	{0x382d, 0x00},
	{0x3835, 0x00},
	{0x3836, 0x00},
	{0x3837, 0x10},
	{0x3839, 0x00},
	{0x383b, 0x00},
	{0x383c, 0x00},
	{0x383d, 0x10},
	{0x383e, 0x00},
	{0x383f, 0x33},
	{0x3842, 0x00},
	{0x3856, 0x00},
	{0x3857, 0x10},
	{0x3858, 0x00},
	{0x3859, 0x20},
	{0x3865, 0x00},
	{0x3867, 0x08},
	{0x3868, 0x00},
	{0x3904, 0x33},
	{0x3907, 0x33},
	{0x390a, 0x9a},
	{0x3914, 0x34},
	{0x3938, 0x4b},
	{0x3939, 0x0c},
	{0x393b, 0x4b},
	{0x393c, 0x0c},
	{0x393e, 0x40},
	{0x393f, 0x0c},
	{0x3975, 0x05},
	{0x3979, 0x32},
	{0x397d, 0x69},
	{0x3981, 0x15},
	{0x3983, 0x33},
	{0x3985, 0x1a},
	{0x3986, 0x08},
	{0x398a, 0x09},
	{0x39cd, 0x00},
	{0x39ce, 0x24},
	{0x39cf, 0x40},
	{0x39d0, 0x0a},
	{0x39d1, 0x50},
	{0x39d2, 0x05},
	{0x39d3, 0x94},
	{0x39d4, 0x01},
	{0x39d5, 0x79},
	{0x3a12, 0x00},
	{0x3a13, 0x00},
	{0x3a14, 0x00},
	{0x3a15, 0x00},
	{0x3a16, 0x00},
	{0x3a18, 0x04},
	{0x3a1a, 0x05},
	{0x3a1c, 0x0a},
	{0x3a1e, 0x03},
	{0x3a1f, 0x34},
	{0x3a22, 0x12},
	{0x3a24, 0x00},
	{0x3a25, 0xfe},
	{0x3a26, 0x01},
	{0x3a27, 0x01},
	{0x3a2a, 0xa8},
	{0x3a2b, 0xa8},
	{0x3a36, 0x00},
	{0x3d84, 0x00},
	{0x3d85, 0x1b},
	{0x3d88, 0x00},
	{0x3d89, 0x00},
	{0x3d8a, 0x03},
	{0x3d8b, 0xff},
	{0x3d8c, 0xa3},
	{0x3d8d, 0xc4},
	{0x3da4, 0x04},
	{0x3daa, 0xa0},
	{0x3dab, 0x10},
	{0x3dac, 0xa1},
	{0x3dad, 0x8c},
	{0x3dae, 0xa1},
	{0x3daf, 0xb3},
	{0x3e00, 0x0e},
	{0x3e01, 0x0e},
	{0x3e02, 0x0e},
	{0x3e03, 0x0e},
	{0x3e04, 0x0e},
	{0x3e05, 0x0e},
	{0x3e06, 0x0e},
	{0x3e07, 0x0e},
	{0x3e09, 0x47},
	{0x3e0b, 0x25},
	{0x3e0d, 0x13},
	{0x3e0f, 0x09},
	{0x3e11, 0x07},
	{0x3e13, 0x06},
	{0x3e15, 0x05},
	{0x3e17, 0x04},
	{0x3e18, 0x38},
	{0x3e19, 0x38},
	{0x3e1a, 0x13},
	{0x3e1b, 0x30},
	{0x3e1c, 0x07},
	{0x3e1d, 0x06},
	{0x3e1e, 0x05},
	{0x3e1f, 0x04},
	{0x3e20, 0x0f},
	{0x3e21, 0x0f},
	{0x3e22, 0x0f},
	{0x3e23, 0x0f},
	{0x3e24, 0x0f},
	{0x3e25, 0x0f},
	{0x3e26, 0x0f},
	{0x3e27, 0x0f},
	{0x3e28, 0x07},
	{0x3e29, 0x07},
	{0x3e2a, 0x07},
	{0x3e2b, 0x02},
	{0x3e2c, 0x07},
	{0x3e2d, 0x07},
	{0x3e30, 0x07},
	{0x3e3a, 0x02},
	{0x3e3b, 0xdf},
	{0x3e3c, 0xff},
	{0x3e3d, 0x44},
	{0x3e3e, 0x00},
	{0x3e3f, 0x00},
	{0x3e40, 0xc1},
	{0x3e42, 0x54},
	{0x3e43, 0x54},
	{0x3e44, 0x54},
	{0x3e45, 0x54},
	{0x3f00, 0x10},
	{0x3f01, 0x26},
	{0x3f03, 0x40},
	{0x4002, 0xf3},
	{0x4009, 0x02},
	{0x400e, 0xc6},
	{0x400f, 0x00},
	{0x4010, 0x38},
	{0x4011, 0x01},
	{0x4012, 0x0d},
	{0x4015, 0x04},
	{0x4016, 0x23},
	{0x4017, 0x00},
	{0x4018, 0x07},
	{0x401a, 0x40},
	{0x401e, 0x00},
	{0x401f, 0xcc},
	{0x4020, 0x04},
	{0x4021, 0x00},
	{0x4022, 0x04},
	{0x4023, 0x00},
	{0x4024, 0x04},
	{0x4025, 0x00},
	{0x4026, 0x04},
	{0x4027, 0x00},
	{0x4028, 0x01},
	{0x4030, 0x00},
	{0x4031, 0x00},
	{0x4032, 0x00},
	{0x4033, 0x00},
	{0x4034, 0x00},
	{0x4035, 0x00},
	{0x4036, 0x00},
	{0x4037, 0x00},
	{0x4040, 0x00},
	{0x4041, 0x40},
	{0x4042, 0x00},
	{0x4043, 0x40},
	{0x4044, 0x00},
	{0x4045, 0x40},
	{0x4046, 0x00},
	{0x4047, 0x40},
	{0x4050, 0x00},
	{0x4051, 0x00},
	{0x4056, 0x25},
	{0x4102, 0xf3},
	{0x4109, 0x02},
	{0x410e, 0xc6},
	{0x410f, 0x00},
	{0x4110, 0x28},
	{0x4111, 0x01},
	{0x4112, 0x0d},
	{0x4115, 0x04},
	{0x4116, 0x1b},
	{0x4117, 0x00},
	{0x4118, 0x07},
	{0x411a, 0x40},
	{0x411e, 0x00},
	{0x411f, 0xcc},
	{0x4128, 0x01},
	{0x4156, 0x25},
	{0x4702, 0xf3},
	{0x4709, 0x02},
	{0x470e, 0xc6},
	{0x470f, 0x00},
	{0x4710, 0x28},
	{0x4711, 0x01},
	{0x4712, 0x0d},
	{0x4715, 0x04},
	{0x4716, 0x1b},
	{0x4717, 0x00},
	{0x4718, 0x07},
	{0x471a, 0x40},
	{0x471e, 0x00},
	{0x471f, 0xcc},
	{0x4728, 0x01},
	{0x4756, 0x25},
	{0x4301, 0x00},
	{0x4303, 0x00},
	{0x4305, 0x00},
	{0x4307, 0x00},
	{0x4308, 0x00},
	{0x430b, 0xff},
	{0x430d, 0x00},
	{0x430e, 0x00},
	{0x4503, 0x0f},
	{0x4504, 0x82},
	{0x4508, 0x00},
	{0x451d, 0x00},
	{0x451e, 0x00},
	{0x451f, 0x00},
	{0x4523, 0x00},
	{0x4526, 0x00},
	{0x4527, 0x00},
	{0x4530, 0x80},
	{0x4547, 0x06},
	{0x4640, 0x00},
	{0x4641, 0x30},
	{0x4643, 0x00},
	{0x4645, 0x13},
	{0x464a, 0x00},
	{0x464b, 0x30},
	{0x4680, 0x00},
	{0x4681, 0x24},
	{0x4683, 0x00},
	{0x4800, 0x64},
	{0x480b, 0x10},
	{0x480c, 0x80},
	{0x480e, 0x04},
	{0x480f, 0x32},
	{0x4826, 0x32},
	{0x4833, 0x18},
	{0x4837, 0x06},
	{0x484b, 0x27},
	{0x4850, 0x47},
	{0x4860, 0x00},
	{0x4861, 0xec},
	{0x4862, 0x04},
	{0x4883, 0x00},
	{0x4885, 0x10},
	{0x4888, 0x10},
	{0x4889, 0x03},
	{0x4d00, 0x04},
	{0x4d01, 0x8f},
	{0x4d02, 0xb9},
	{0x4d03, 0xc1},
	{0x4d04, 0xb6},
	{0x4d05, 0x7e},
	{0x5000, 0xfb},
	{0x5001, 0xdb},
	{0x5002, 0x15},
	{0x5003, 0x01},
	{0x5007, 0x1e},
	{0x5008, 0x00},
	{0x5009, 0x00},
	{0x500a, 0x00},
	{0x500b, 0x30},
	{0x500c, 0x12},
	{0x500d, 0x1f},
	{0x500e, 0x0a},
	{0x500f, 0x0f},
	{0x504b, 0x40},
	{0x5081, 0x00},
	{0x50c4, 0xaa},
	{0x50d0, 0x00},
	{0x50d1, 0x10},
	{0x50d2, 0x01},
	{0x50d3, 0xb3},
	{0x515a, 0x06},
	{0x515b, 0x06},
	{0x515c, 0x02},
	{0x515d, 0x02},
	{0x515e, 0x02},
	{0x515f, 0x06},
	{0x5160, 0x0a},
	{0x5161, 0x0e},
	{0x5180, 0x09},
	{0x5181, 0x10},
	{0x5182, 0x05},
	{0x5183, 0x20},
	{0x5184, 0x00},
	{0x5185, 0x08},
	{0x5186, 0x00},
	{0x5187, 0x00},
	{0x5188, 0x09},
	{0x5189, 0x00},
	{0x518a, 0x05},
	{0x518b, 0x20},
	{0x518d, 0x09},
	{0x5192, 0x00},
	{0x5193, 0x30},
	{0x51d2, 0x10},
	{0x51da, 0x00},
	{0x51db, 0x30},
	{0x5250, 0xec},
	{0x5251, 0x00},
	{0x5252, 0x10},
	{0x5254, 0x00},
	{0x5255, 0x72},
	{0x5256, 0x00},
	{0x5257, 0xca},
	{0x5258, 0x12},
	{0x5259, 0x20},
	{0x525a, 0x0a},
	{0x525b, 0x40},
	{0x525e, 0x00},
	{0x525f, 0x30},
	{0x5260, 0x00},
	{0x5261, 0x10},
	{0x5262, 0x00},
	{0x5263, 0x10},
	{0x5264, 0x12},
	{0x5265, 0x00},
	{0x5266, 0x0a},
	{0x5267, 0x20},
	{0x5268, 0x00},
	{0x5269, 0x00},
	{0x526a, 0x00},
	{0x526b, 0x00},
	{0x526c, 0x12},
	{0x526d, 0x10},
	{0x526e, 0x0a},
	{0x526f, 0x30},
	{0x5278, 0x10},
	{0x5279, 0x20},
	{0x527a, 0x01},
	{0x527b, 0x01},
	{0x527c, 0x0c},
	{0x527d, 0x0c},
	{0x527e, 0x04},
	{0x527f, 0x04},
	{0x5280, 0x04},
	{0x5281, 0x0c},
	{0x5282, 0x14},
	{0x5283, 0x1c},
	{0x5381, 0x00},
	{0x53c4, 0xaa},
	{0x545a, 0x06},
	{0x545b, 0x06},
	{0x545c, 0x02},
	{0x545d, 0x02},
	{0x545e, 0x02},
	{0x545f, 0x06},
	{0x5460, 0x0a},
	{0x5461, 0x0e},
	{0x5480, 0x09},
	{0x5481, 0x10},
	{0x5482, 0x05},
	{0x5483, 0x20},
	{0x5484, 0x00},
	{0x5485, 0x08},
	{0x5486, 0x00},
	{0x5487, 0x00},
	{0x5488, 0x09},
	{0x5489, 0x00},
	{0x548a, 0x05},
	{0x548b, 0x20},
	{0x548d, 0x09},
	{0x54d2, 0x10},
	{0x54da, 0x00},
	{0x54da, 0x00},
	{0x54db, 0x30},
	{0x54db, 0x30},
	{0x5550, 0xec},
	{0x5551, 0x00},
	{0x5552, 0x10},
	{0x5554, 0x00},
	{0x5555, 0x72},
	{0x5556, 0x00},
	{0x5557, 0xca},
	{0x5558, 0x12},
	{0x5559, 0x10},
	{0x555a, 0x0a},
	{0x555b, 0x30},
	{0x555e, 0x00},
	{0x555f, 0x30},
	{0x5560, 0x00},
	{0x5561, 0x00},
	{0x5562, 0x00},
	{0x5563, 0x08},
	{0x5564, 0x12},
	{0x5565, 0x10},
	{0x5566, 0x0a},
	{0x5567, 0x30},
	{0x5568, 0x00},
	{0x5569, 0x00},
	{0x556a, 0x00},
	{0x556b, 0x00},
	{0x556c, 0x12},
	{0x556d, 0x10},
	{0x556e, 0x0a},
	{0x556f, 0x30},
	{0x557c, 0x06},
	{0x557d, 0x06},
	{0x557e, 0x02},
	{0x557f, 0x02},
	{0x5580, 0x02},
	{0x5581, 0x06},
	{0x5582, 0x0a},
	{0x5583, 0x0e},
	{0x5681, 0x00},
	{0x56c4, 0xaa},
	{0x575a, 0x06},
	{0x575b, 0x06},
	{0x575c, 0x02},
	{0x575d, 0x02},
	{0x575e, 0x02},
	{0x575f, 0x06},
	{0x5760, 0x0a},
	{0x5761, 0x0e},
	{0x5780, 0x09},
	{0x5781, 0x10},
	{0x5782, 0x05},
	{0x5783, 0x20},
	{0x5784, 0x00},
	{0x5785, 0x08},
	{0x5786, 0x00},
	{0x5787, 0x00},
	{0x5788, 0x09},
	{0x5789, 0x00},
	{0x578a, 0x05},
	{0x578b, 0x20},
	{0x578d, 0x09},
	{0x5792, 0x00},
	{0x5793, 0x30},
	{0x57d2, 0x10},
	{0x57da, 0x00},
	{0x57db, 0x30},
	{0x5850, 0xec},
	{0x5851, 0x00},
	{0x5852, 0x10},
	{0x5854, 0x00},
	{0x5855, 0x72},
	{0x5856, 0x00},
	{0x5857, 0xca},
	{0x5858, 0x12},
	{0x5859, 0x10},
	{0x585a, 0x0a},
	{0x585b, 0x30},
	{0x585e, 0x00},
	{0x585f, 0x30},
	{0x5860, 0x00},
	{0x5861, 0x00},
	{0x5862, 0x00},
	{0x5863, 0x08},
	{0x5864, 0x12},
	{0x5865, 0x10},
	{0x5866, 0x0a},
	{0x5867, 0x30},
	{0x5868, 0x00},
	{0x5869, 0x00},
	{0x586a, 0x00},
	{0x586b, 0x00},
	{0x586c, 0x12},
	{0x586d, 0x10},
	{0x586e, 0x0a},
	{0x586f, 0x30},
	{0x587c, 0x06},
	{0x587d, 0x06},
	{0x587e, 0x02},
	{0x587f, 0x02},
	{0x5880, 0x02},
	{0x5881, 0x06},
	{0x5882, 0x0a},
	{0x5883, 0x0e},
	{0x5f06, 0x10},
	{0x5f07, 0x20},
	{0x5f08, 0x0c},
	{0x5f09, 0x0c},
	{0x5f0a, 0x04},
	{0x5f0b, 0x04},
	{0x5f0c, 0x04},
	{0x5f0d, 0x0c},
	{0x5f0e, 0x14},
	{0x5f0f, 0x1c},
	{0x5f10, 0x01},
	{0x5f11, 0x01},
	{0x5f18, 0x12},
	{0x5f19, 0x20},
	{0x5f1a, 0x0a},
	{0x5f1b, 0x40},
	{0x5f1d, 0x10},
	{0x5f1f, 0x10},
	{0x5f20, 0x12},
	{0x5f21, 0x00},
	{0x5f22, 0x0a},
	{0x5f23, 0x20},
	{0x5f25, 0x09},
	{0x5f2a, 0x00},
	{0x5f2b, 0x30},
	{0x6600, 0x00},
	{0x6601, 0x00},
	{0x6602, 0x00},
	{0x6603, 0x83},
	{0x6960, 0x0f},
	{0x69a2, 0x09},
	{0x69a3, 0x00},
	{0x69a6, 0x05},
	{0x69a7, 0x10},
	{0x69aa, 0x09},
	{0x69ab, 0x00},
	{0x69ae, 0x05},
	{0x69af, 0x10},
	{0x69b2, 0x09},
	{0x69b3, 0x00},
	{0x69b6, 0x05},
	{0x69b7, 0x10},
	{0x69ba, 0x09},
	{0x69bb, 0x00},
	{0x69be, 0x05},
	{0x69bf, 0x10},
	{0x6a24, 0x09},
	{0x6a25, 0x00},
	{0x6a2a, 0x05},
	{0x6a2b, 0x10},
	{0x6a61, 0x40},
	{0x6a64, 0x09},
	{0x6a65, 0x00},
	{0x6a6a, 0x05},
	{0x6a6b, 0x10},
	{0x6a23, 0x00},
	{0x6a27, 0x00},
	{0x6a63, 0x00},
	{0x6a67, 0x00},
	{0x69a1, 0x00},
	{0x69a5, 0x00},
	{0x69a9, 0x00},
	{0x69ad, 0x00},
	{0x69b1, 0x00},
	{0x69b5, 0x00},
	{0x69b9, 0x00},
	{0x69bd, 0x00},
	{0xfff4, 0x01},
	{0xfff6, 0x00},
	{0x0361, 0x07},
	{0x3644, 0x20},
	{0x50d4, 0x00},
	{0x5a02, 0x0f},
	{0x5002, 0x55},
	{0x5331, 0x0a},
	{0x5332, 0x43},
	{0x5333, 0x45},
	{0x5284, 0x03},
	{0x5285, 0x02},
	{0x5286, 0x02},
	{0x5287, 0x03},
	{0x5f15, 0xeb},
	{0x5290, 0x00},
	{0x5291, 0x50},
	{0x5292, 0x00},
	{0x5293, 0x50},
	{0x5294, 0x00},
	{0x5295, 0x50},
	{0x5296, 0x00},
	{0x5297, 0x50},
	{0x5298, 0x00},
	{0x5299, 0x50},
	{0x529a, 0x01},
	{0x529b, 0x00},
	{0x529c, 0x01},
	{0x529d, 0x00},
	{0x529e, 0x00},
	{0x529f, 0x50},
	{0x52a0, 0x00},
	{0x52a1, 0x50},
	{0x52a2, 0x01},
	{0x52a3, 0x00},
	{0x52a4, 0x01},
	{0x52a5, 0x00},
	{0x52a6, 0x00},
	{0x52a7, 0x50},
	{0x52a8, 0x00},
	{0x52a9, 0x50},
	{0x52aa, 0x00},
	{0x52ab, 0x50},
	{0x52ac, 0x00},
	{0x52ad, 0x50},
	{0x52ae, 0x00},
	{0x52af, 0x50},
	{0x52b0, 0x00},
	{0x52b1, 0x50},
	{0x52b2, 0x00},
	{0x52b3, 0x50},
	{0x52b4, 0x00},
	{0x52b5, 0x50},
	{0x52b6, 0x00},
	{0x52b7, 0x50},
	{0x52b8, 0x00},
	{0x52b9, 0x50},
	{0x52ba, 0x01},
	{0x52bb, 0x00},
	{0x52bc, 0x01},
	{0x52bd, 0x00},
	{0x52be, 0x00},
	{0x52bf, 0x50},
	{0x52c0, 0x00},
	{0x52c1, 0x50},
	{0x52c2, 0x01},
	{0x52c3, 0x00},
	{0x52c4, 0x01},
	{0x52c5, 0x00},
	{0x52c6, 0x00},
	{0x52c7, 0x50},
	{0x52c8, 0x00},
	{0x52c9, 0x50},
	{0x52ca, 0x00},
	{0x52cb, 0x50},
	{0x52cc, 0x00},
	{0x52cd, 0x50},
	{0x52ce, 0x00},
	{0x52cf, 0x50},
	{0x5f00, 0x29},
	{0x5f2d, 0x28},
	{0x5f2e, 0x28},
	{0x52f0, 0x04},
	{0x52f1, 0x03},
	{0x52f2, 0x02},
	{0x52f3, 0x01},
	{0x52f4, 0x08},
	{0x52f5, 0x07},
	{0x52f6, 0x06},
	{0x52f7, 0x05},
	{0x52f8, 0x0c},
	{0x52f9, 0x0b},
	{0x52fa, 0x0a},
	{0x52fb, 0x09},
	{0x52fc, 0x10},
	{0x52fd, 0x0f},
	{0x52fe, 0x0e},
	{0x52ff, 0x0d},
	{0x5300, 0x14},
	{0x5301, 0x13},
	{0x5302, 0x12},
	{0x5303, 0x11},
	{0x5304, 0x18},
	{0x5305, 0x17},
	{0x5306, 0x16},
	{0x5307, 0x15},
	{0x5308, 0x1c},
	{0x5309, 0x1b},
	{0x530a, 0x1a},
	{0x530b, 0x19},
	{0x530c, 0x20},
	{0x530d, 0x1f},
	{0x530e, 0x1e},
	{0x530f, 0x1d},
	{0x5353, 0x21},
	{0x5354, 0x01},
	{0x5355, 0x02},
	{0x5356, 0x04},
	{0x5357, 0x06},
	{0x5358, 0x08},
	{0x5359, 0x0c},
	{0x535a, 0x10},
	{0x535b, 0x10},
	{0x5990, 0x00},
	{0x5991, 0x00},
	{0x5992, 0x01},
	{0x5993, 0x01},
	{0x5994, 0x02},
	{0x5995, 0x04},
	{0x5996, 0x06},
	{0x5997, 0x08},
	{0x5998, 0x0a},
	{0x5999, 0x0c},
	{0x599a, 0x0e},
	{0x599b, 0x10},
	{0x599c, 0x12},
	{0x599d, 0x14},
	{0x599e, 0x16},
	{0x599f, 0x18},
	{0x3222, 0x03},
	{0x3208, 0x06},
	{0x3938, 0x41},
	{0x393b, 0x41},
	{0x3208, 0x16},
	{0x3208, 0x07},
	{0x3938, 0x43},
	{0x393b, 0x44},
	{0x3208, 0x17},
	{0x3208, 0x08},
	{0x3938, 0x45},
	{0x393b, 0x46},
	{0x3208, 0x18},
	{0x3208, 0x09},
	{0x3938, 0x4b},
	{0x393b, 0x4b},
	{0x3208, 0x19},
	{0x5000, 0x29},
	{0x5001, 0xd1},
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
static const struct ov12d2q_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 2256,
		.height = 1256,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0560,
		.hts_def = 0x021b*5,
		.vts_def = 0x0570,
		.reg_list = ov12d2q_2256x1256_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.link_freq_idx = 0,
		.bpp = 10,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 4512,
		.height = 2512,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0ad0,
		.hts_def = 0x021c*9,
		.vts_def = 0x0ae0,
		.reg_list = ov12d2q_4512x2512_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.link_freq_idx = 0,
		.bpp = 10,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_625M,
};

static const char * const ov12d2q_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

static int __ov12d2q_power_on(struct ov12d2q *ov12d2q);

/* Write registers up to 4 at a time */
static int ov12d2q_write_reg(struct i2c_client *client, u16 reg,
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

static int ov12d2q_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= ov12d2q_write_reg(client, regs[i].addr,
					 OV12D2Q_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ov12d2q_read_reg(struct i2c_client *client,
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

static int ov12d2q_get_reso_dist(const struct ov12d2q_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov12d2q_mode *
ov12d2q_find_best_fit(struct ov12d2q *ov12d2q, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ov12d2q->cfg_num; i++) {
		dist = ov12d2q_get_reso_dist(&supported_modes[i], framefmt);
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

static int ov12d2q_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	const struct ov12d2q_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&ov12d2q->mutex);

	mode = ov12d2q_find_best_fit(ov12d2q, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov12d2q->mutex);
		return -ENOTTY;
#endif
	} else {
		ov12d2q->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov12d2q->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov12d2q->vblank, vblank_def,
					 OV12D2Q_VTS_MAX - mode->height,
					 1, vblank_def);
		if (mode->hdr_mode == NO_HDR) {
			dst_link_freq = 0;
			dst_pixel_rate = PIXEL_RATE_WITH_625M;
		}
		__v4l2_ctrl_s_ctrl_int64(ov12d2q->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(ov12d2q->link_freq,
				   dst_link_freq);
		ov12d2q->cur_fps = mode->max_fps;
	}

	mutex_unlock(&ov12d2q->mutex);

	return 0;
}

static int ov12d2q_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	const struct ov12d2q_mode *mode = ov12d2q->cur_mode;

	mutex_lock(&ov12d2q->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov12d2q->mutex);
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
	mutex_unlock(&ov12d2q->mutex);

	return 0;
}

static int ov12d2q_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int ov12d2q_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);

	if (fse->index >= ov12d2q->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov12d2q_enable_test_pattern(struct ov12d2q *ov12d2q, u32 pattern)
{
	u32 val;
	int ret = 0;

	if (pattern)
		val = ((pattern - 1) << 4) | OV12D2Q_TEST_PATTERN_ENABLE;
	else
		val = OV12D2Q_TEST_PATTERN_DISABLE;

	ret = ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_TEST_PATTERN,
				OV12D2Q_REG_VALUE_08BIT, val);

	return ret;
}

static int ov12d2q_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	const struct ov12d2q_mode *mode = ov12d2q->cur_mode;

	if (ov12d2q->streaming)
		fi->interval = ov12d2q->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct ov12d2q_mode *ov12d2q_find_mode(struct ov12d2q *ov12d2q, int fps)
{
	const struct ov12d2q_mode *mode = NULL;
	const struct ov12d2q_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == ov12d2q->cur_mode->width &&
		    mode->height == ov12d2q->cur_mode->height &&
		    mode->hdr_mode == ov12d2q->cur_mode->hdr_mode &&
		    mode->bus_fmt == ov12d2q->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int ov12d2q_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	const struct ov12d2q_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	u32 lane_num = OV12D2Q_LANES;
	int fps;

	if (ov12d2q->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = ov12d2q_find_mode(ov12d2q, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	ov12d2q->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(ov12d2q->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(ov12d2q->vblank, vblank_def,
				 OV12D2Q_VTS_MAX - mode->height,
				 1, vblank_def);
	pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] / mode->bpp * 2 * lane_num;

	__v4l2_ctrl_s_ctrl_int64(ov12d2q->pixel_rate,
				 pixel_rate);
	__v4l2_ctrl_s_ctrl(ov12d2q->link_freq,
			   mode->link_freq_idx);
	ov12d2q->cur_fps = mode->max_fps;
	return 0;
}

static int ov12d2q_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	const struct ov12d2q_mode *mode = ov12d2q->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (OV12D2Q_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (OV12D2Q_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void ov12d2q_get_module_inf(struct ov12d2q *ov12d2q,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, OV12D2Q_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov12d2q->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov12d2q->len_name, sizeof(inf->base.lens));
}

static long ov12d2q_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode == ov12d2q->cur_mode->hdr_mode)
			return 0;
		w = ov12d2q->cur_mode->width;
		h = ov12d2q->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(ov12d2q->cur_mode->max_fps.denominator,
			ov12d2q->cur_mode->max_fps.numerator);
		for (i = 0; i < ov12d2q->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr_cfg->hdr_mode &&
			    supported_modes[i].bus_fmt == ov12d2q->cur_mode->bus_fmt) {
				cur_fps = DIV_ROUND_CLOSEST(supported_modes[i].max_fps.denominator,
					supported_modes[i].max_fps.numerator);
				cur_dist = abs(cur_fps - dst_fps);
				if (cur_best_fit_dist == -1 || cur_dist < cur_best_fit_dist) {
					cur_best_fit_dist = cur_dist;
					cur_best_fit = i;
				} else if (cur_dist == cur_best_fit_dist) {
					cur_best_fit = i;
					break;
				}
			}
		}
		if (cur_best_fit == -1) {
			dev_err(&ov12d2q->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			ov12d2q->cur_mode = &supported_modes[cur_best_fit];
			w = ov12d2q->cur_mode->hts_def - ov12d2q->cur_mode->width;
			h = ov12d2q->cur_mode->vts_def - ov12d2q->cur_mode->height;
			__v4l2_ctrl_modify_range(ov12d2q->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(ov12d2q->vblank, h,
						 OV12D2Q_VTS_MAX - ov12d2q->cur_mode->height,
						 1, h);
			ov12d2q->cur_fps = ov12d2q->cur_mode->max_fps;
			dev_info(&ov12d2q->client->dev,
				"sensor mode: %d\n",
				ov12d2q->cur_mode->hdr_mode);
		}
		break;
	case RKMODULE_GET_MODULE_INFO:
		ov12d2q_get_module_inf(ov12d2q, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = ov12d2q->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_CTRL_MODE,
				OV12D2Q_REG_VALUE_08BIT, OV12D2Q_MODE_STREAMING);
		else
			ret = ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_CTRL_MODE,
				OV12D2Q_REG_VALUE_08BIT, OV12D2Q_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov12d2q_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov12d2q_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
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
			ret = ov12d2q_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov12d2q_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
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
			ret = ov12d2q_ioctl(sd, cmd, hdr);
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
			ret = ov12d2q_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = ov12d2q_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov12d2q_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov12d2q_start_stream(struct ov12d2q *ov12d2q)
{
	int ret;

	if (!ov12d2q->is_thunderboot) {
		ret = ov12d2q_write_array(ov12d2q->client, ov12d2q_global_regs);
		if (ret) {
			dev_err(&ov12d2q->client->dev,
				"could not set init registers\n");
			return ret;
		}

		ret = ov12d2q_write_array(ov12d2q->client, ov12d2q->cur_mode->reg_list);
		if (ret)
			return ret;
	}

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ov12d2q->ctrl_handler);
	if (ret)
		return ret;

	return ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_CTRL_MODE,
				 OV12D2Q_REG_VALUE_08BIT, OV12D2Q_MODE_STREAMING);
}

static int __ov12d2q_stop_stream(struct ov12d2q *ov12d2q)
{
	if (ov12d2q->is_thunderboot)
		ov12d2q->is_first_streamoff = true;
	return ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_CTRL_MODE,
				 OV12D2Q_REG_VALUE_08BIT, OV12D2Q_MODE_SW_STANDBY);
}

static int ov12d2q_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	struct i2c_client *client = ov12d2q->client;
	int ret = 0;

	mutex_lock(&ov12d2q->mutex);
	on = !!on;
	if (on == ov12d2q->streaming)
		goto unlock_and_return;

	if (on) {
		if (ov12d2q->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			ov12d2q->is_thunderboot = false;
			__ov12d2q_power_on(ov12d2q);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov12d2q_start_stream(ov12d2q);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov12d2q_stop_stream(ov12d2q);
		pm_runtime_put(&client->dev);
	}

	ov12d2q->streaming = on;

unlock_and_return:
	mutex_unlock(&ov12d2q->mutex);

	return ret;
}

static int ov12d2q_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	struct i2c_client *client = ov12d2q->client;
	int ret = 0;

	mutex_lock(&ov12d2q->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov12d2q->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!ov12d2q->is_thunderboot) {
			ret |= ov12d2q_write_reg(ov12d2q->client,
						 OV12D2Q_SOFTWARE_RESET_REG,
						 OV12D2Q_REG_VALUE_08BIT,
						 0x01);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
			usleep_range(100, 200);
		}

		ov12d2q->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov12d2q->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov12d2q->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov12d2q_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV12D2Q_XVCLK_FREQ / 1000 / 1000);
}

static int __ov12d2q_power_on(struct ov12d2q *ov12d2q)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov12d2q->client->dev;

	if (ov12d2q->is_thunderboot)
		return 0;

	if (!IS_ERR_OR_NULL(ov12d2q->pins_default)) {
		ret = pinctrl_select_state(ov12d2q->pinctrl,
					   ov12d2q->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov12d2q->xvclk, OV12D2Q_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov12d2q->xvclk) != OV12D2Q_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov12d2q->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(ov12d2q->reset_gpio))
		gpiod_direction_output(ov12d2q->reset_gpio, 1);

	ret = regulator_bulk_enable(OV12D2Q_NUM_SUPPLIES, ov12d2q->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov12d2q->reset_gpio))
		gpiod_direction_output(ov12d2q->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(ov12d2q->pwdn_gpio))
		gpiod_direction_output(ov12d2q->pwdn_gpio, 1);
	/*
	 * There is no need to wait for the delay of RC circuit
	 * if the reset signal is directly controlled by GPIO.
	 */
	if (!IS_ERR(ov12d2q->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov12d2q_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov12d2q->xvclk);

	return ret;
}

static void __ov12d2q_power_off(struct ov12d2q *ov12d2q)
{
	int ret;
	struct device *dev = &ov12d2q->client->dev;

	if (ov12d2q->is_thunderboot) {
		if (ov12d2q->is_first_streamoff) {
			ov12d2q->is_thunderboot = false;
			ov12d2q->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(ov12d2q->pwdn_gpio))
		gpiod_direction_output(ov12d2q->pwdn_gpio, 0);

	clk_disable_unprepare(ov12d2q->xvclk);

	if (!IS_ERR(ov12d2q->reset_gpio))
		gpiod_direction_output(ov12d2q->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov12d2q->pins_sleep)) {
		ret = pinctrl_select_state(ov12d2q->pinctrl,
					   ov12d2q->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

	if (ov12d2q->is_thunderboot_ng) {
		ov12d2q->is_thunderboot_ng = false;
		regulator_bulk_disable(OV12D2Q_NUM_SUPPLIES, ov12d2q->supplies);
	}
}

static int ov12d2q_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);

	return __ov12d2q_power_on(ov12d2q);
}

static int ov12d2q_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);

	__ov12d2q_power_off(ov12d2q);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov12d2q_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov12d2q_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov12d2q->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov12d2q->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov12d2q_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);

	if (fie->index >= ov12d2q->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops ov12d2q_pm_ops = {
	SET_RUNTIME_PM_OPS(ov12d2q_runtime_suspend,
			   ov12d2q_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov12d2q_internal_ops = {
	.open = ov12d2q_open,
};
#endif

static const struct v4l2_subdev_core_ops ov12d2q_core_ops = {
	.s_power = ov12d2q_s_power,
	.ioctl = ov12d2q_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov12d2q_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov12d2q_video_ops = {
	.s_stream = ov12d2q_s_stream,
	.g_frame_interval = ov12d2q_g_frame_interval,
	.s_frame_interval = ov12d2q_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov12d2q_pad_ops = {
	.enum_mbus_code = ov12d2q_enum_mbus_code,
	.enum_frame_size = ov12d2q_enum_frame_sizes,
	.enum_frame_interval = ov12d2q_enum_frame_interval,
	.get_fmt = ov12d2q_get_fmt,
	.set_fmt = ov12d2q_set_fmt,
	.get_mbus_config = ov12d2q_g_mbus_config,
};

static const struct v4l2_subdev_ops ov12d2q_subdev_ops = {
	.core	= &ov12d2q_core_ops,
	.video	= &ov12d2q_video_ops,
	.pad	= &ov12d2q_pad_ops,
};

static void ov12d2q_modify_fps_info(struct ov12d2q *ov12d2q)
{
	const struct ov12d2q_mode *mode = ov12d2q->cur_mode;

	ov12d2q->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				      ov12d2q->cur_vts;
}

static int ov12d2q_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov12d2q *ov12d2q = container_of(ctrl->handler,
					       struct ov12d2q, ctrl_handler);
	struct i2c_client *client = ov12d2q->client;
	s64 max;
	int ret = 0;
	u32 again, dgain;
	u32 val = 0, x_win = 0, y_win = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov12d2q->cur_mode->height + ctrl->val - 16;
		__v4l2_ctrl_modify_range(ov12d2q->exposure,
					 ov12d2q->exposure->minimum, max,
					 ov12d2q->exposure->step,
					 ov12d2q->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (ov12d2q->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end;
		ret = ov12d2q_write_reg(ov12d2q->client,
					OV12D2Q_REG_EXP_L_H,
					OV12D2Q_REG_VALUE_16BIT,
					ctrl->val);
		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (ov12d2q->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end;
		if (ctrl->val > 1984) {// >15.5x
			dgain = ctrl->val * 10 / 155;
			again = 1984;
		} else {
			dgain = 1024;
			again = ctrl->val;
		}
		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_REG_AGAIN_L,
					 OV12D2Q_REG_VALUE_16BIT,
					 (again << 1) & 0x7ffe);

		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_REG_DGAIN_L_H_B,
					 OV12D2Q_REG_VALUE_24BIT,
					 (dgain << 6) & 0xfffc0);

		dev_dbg(&client->dev, "set gain 0x%x set analog gain 0x%x digital gain 0x%x\n", 
			ctrl->val, again, dgain);
		break;
	case V4L2_CID_VBLANK:
		ret = ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_VTS,
					OV12D2Q_REG_VALUE_16BIT,
					ctrl->val + ov12d2q->cur_mode->height);
		ov12d2q->cur_vts = ctrl->val + ov12d2q->cur_mode->height;
		ov12d2q_modify_fps_info(ov12d2q);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov12d2q_enable_test_pattern(ov12d2q, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov12d2q_read_reg(ov12d2q->client, OV12D2Q_MIRROR_REG,
				       OV12D2Q_REG_VALUE_08BIT,
				       &val);
		if (ctrl->val)
			val |= MIRROR_BIT_MASK;
		else
			val &= ~MIRROR_BIT_MASK;

		ret |= ov12d2q_read_reg(ov12d2q->client, OV12D2Q_REG_ISP_X_WIN,
					OV12D2Q_REG_VALUE_16BIT,
					&x_win);

		if((x_win == 0x0021) && (!(val & 0x04))) {
			x_win = 0x0020;
		}else if((x_win == 0x0020) && (val & 0x04)) {
			x_win = 0x0021;
		}

		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_START_DATA);

		ret |= ov12d2q_write_reg(ov12d2q->client, OV12D2Q_MIRROR_REG,
					 OV12D2Q_REG_VALUE_08BIT,
					 val);
		ret |= ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_ISP_X_WIN,
					 OV12D2Q_REG_VALUE_16BIT,
					 x_win);

		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_END_DATA);
		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_LAUNCH);
		break;
	case V4L2_CID_VFLIP:
		ret = ov12d2q_read_reg(ov12d2q->client, OV12D2Q_FLIP_REG,
				       OV12D2Q_REG_VALUE_08BIT,
				       &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;

		ret |= ov12d2q_read_reg(ov12d2q->client, OV12D2Q_REG_ISP_Y_WIN,
					OV12D2Q_REG_VALUE_16BIT,
					&y_win);

		if ((y_win == 0x0004) && (val & 0x04)) {
			y_win = 0x0005;
		} else if ((y_win == 0x0005) && (!(val & 0x04))) {
			y_win = 0x0004;
		}

		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_START_DATA);

		ret |= ov12d2q_write_reg(ov12d2q->client, OV12D2Q_FLIP_REG,
					 OV12D2Q_REG_VALUE_08BIT,
					 val);
		ret |= ov12d2q_write_reg(ov12d2q->client, OV12D2Q_REG_ISP_Y_WIN,
					 OV12D2Q_REG_VALUE_16BIT,
					 y_win);

		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_END_DATA);
		ret |= ov12d2q_write_reg(ov12d2q->client,
					 OV12D2Q_GROUP_UPDATE_ADDRESS,
					 OV12D2Q_REG_VALUE_08BIT,
					 OV12D2Q_GROUP_UPDATE_LAUNCH);
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

static const struct v4l2_ctrl_ops ov12d2q_ctrl_ops = {
	.s_ctrl = ov12d2q_set_ctrl,
};

static int ov12d2q_initialize_controls(struct ov12d2q *ov12d2q)
{
	const struct ov12d2q_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &ov12d2q->ctrl_handler;
	mode = ov12d2q->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &ov12d2q->mutex;

	ov12d2q->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						    V4L2_CID_LINK_FREQ,
						    0, 0, link_freq_menu_items);

	if (ov12d2q->cur_mode->bus_fmt == MEDIA_BUS_FMT_SBGGR10_1X10) {
		dst_link_freq = 0;
		dst_pixel_rate = PIXEL_RATE_WITH_625M;
	}
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	ov12d2q->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_PIXEL_RATE,
			0, PIXEL_RATE_WITH_625M,
			1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(ov12d2q->link_freq,
			   dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	ov12d2q->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov12d2q->hblank)
		ov12d2q->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov12d2q->vblank = v4l2_ctrl_new_std(handler, &ov12d2q_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV12D2Q_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 16;
	ov12d2q->exposure = v4l2_ctrl_new_std(handler, &ov12d2q_ctrl_ops,
				V4L2_CID_EXPOSURE, OV12D2Q_EXPOSURE_MIN,
				exposure_max, OV12D2Q_EXPOSURE_STEP,
				mode->exp_def);

	ov12d2q->anal_gain = v4l2_ctrl_new_std(handler, &ov12d2q_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, OV12D2Q_GAIN_MIN,
				OV12D2Q_GAIN_MAX, OV12D2Q_GAIN_STEP,
				OV12D2Q_GAIN_DEFAULT);

	ov12d2q->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov12d2q_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov12d2q_test_pattern_menu) - 1,
				0, 0, ov12d2q_test_pattern_menu);

	ov12d2q->h_flip = v4l2_ctrl_new_std(handler, &ov12d2q_ctrl_ops,
					    V4L2_CID_HFLIP, 0, 1, 1, 0);

	ov12d2q->v_flip = v4l2_ctrl_new_std(handler, &ov12d2q_ctrl_ops,
					    V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov12d2q->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov12d2q->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov12d2q_check_sensor_id(struct ov12d2q *ov12d2q,
				  struct i2c_client *client)
{
	struct device *dev = &ov12d2q->client->dev;
	u32 id = 0;
	int ret;

	if (ov12d2q->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = ov12d2q_read_reg(client, OV12D2Q_REG_CHIP_ID,
			       OV12D2Q_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int ov12d2q_configure_regulators(struct ov12d2q *ov12d2q)
{
	unsigned int i;

	for (i = 0; i < OV12D2Q_NUM_SUPPLIES; i++)
		ov12d2q->supplies[i].supply = ov12d2q_supply_names[i];

	return devm_regulator_bulk_get(&ov12d2q->client->dev,
				       OV12D2Q_NUM_SUPPLIES,
				       ov12d2q->supplies);
}

static int ov12d2q_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov12d2q *ov12d2q;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov12d2q = devm_kzalloc(dev, sizeof(*ov12d2q), GFP_KERNEL);
	if (!ov12d2q)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov12d2q->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov12d2q->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov12d2q->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov12d2q->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov12d2q->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE,
			&hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}
	ov12d2q->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < ov12d2q->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			ov12d2q->cur_mode = &supported_modes[i];
			break;
		}
	}
	ov12d2q->client = client;

	ov12d2q->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov12d2q->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov12d2q->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ov12d2q->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov12d2q->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(ov12d2q->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ov12d2q->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov12d2q->pinctrl)) {
		ov12d2q->pins_default =
			pinctrl_lookup_state(ov12d2q->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov12d2q->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov12d2q->pins_sleep =
			pinctrl_lookup_state(ov12d2q->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov12d2q->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov12d2q_configure_regulators(ov12d2q);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov12d2q->mutex);

	sd = &ov12d2q->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov12d2q_subdev_ops);
	ret = ov12d2q_initialize_controls(ov12d2q);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov12d2q_power_on(ov12d2q);
	if (ret)
		goto err_free_handler;

	ret = ov12d2q_check_sensor_id(ov12d2q, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov12d2q_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov12d2q->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov12d2q->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov12d2q->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov12d2q->module_index, facing,
		 OV12D2Q_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

    return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov12d2q_power_off(ov12d2q);
err_free_handler:
	v4l2_ctrl_handler_free(&ov12d2q->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov12d2q->mutex);

	return ret;
}

static int ov12d2q_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov12d2q *ov12d2q = to_ov12d2q(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov12d2q->ctrl_handler);
	mutex_destroy(&ov12d2q->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov12d2q_power_off(ov12d2q);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov12d2q_of_match[] = {
	{ .compatible = "ovti,ov12d2q" },
	{},
};
MODULE_DEVICE_TABLE(of, ov12d2q_of_match);
#endif

static const struct i2c_device_id ov12d2q_match_id[] = {
	{ "ovti,ov12d2q", 0 },
	{ },
};

static struct i2c_driver ov12d2q_i2c_driver = {
	.driver = {
		.name = OV12D2Q_NAME,
		.pm = &ov12d2q_pm_ops,
		.of_match_table = of_match_ptr(ov12d2q_of_match),
	},
	.probe		= &ov12d2q_probe,
	.remove		= &ov12d2q_remove,
	.id_table	= ov12d2q_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(ov12d2q_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov12d2q_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov12d2q_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("OmniVision ov12d2q sensor driver");
MODULE_LICENSE("GPL v2");
