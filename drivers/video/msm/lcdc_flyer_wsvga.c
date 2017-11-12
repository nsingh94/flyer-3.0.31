/* adapted from linux/arch/arm/mach-msm/panel-sonywvga-s6d16a0x21-7x30.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 HTC.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include <mach/panel_id.h>
#include <mach/debug_display.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <mach/atmega_microp.h>
#include "msm_fb.h"
//#include "../../../../drivers/video/msm/msm_fb.h"

/*
#define PANEL_ID_FLR_SMD_XC	(0x15 | BL_UP	| IF_LCDC | DEPTH_RGB888)
#define PANEL_ID_FLR_LG_XC	(0x18 | BL_UP	| IF_LCDC | DEPTH_RGB888)
#define PANEL_ID_FLR_LG_WS2	(0x19 | BL_UP	| IF_LCDC | DEPTH_RGB888)
*/

#define DEBUG_LCM

#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)     printk("[lcm]%s"fmt, __func__, ##arg)
#else
#define LCMDBG(fmt, arg...)     {}
#endif

#define LCDC_FLYER_PANEL_NAME 			"lcdc_flyer"

#define BRIGHTNESS_DEFAULT_LEVEL        102

#define LCM_MDELAY	0x03

#define PWM_USER_DEF			143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			20
#define PWM_USER_MAX			255

#define PWM_SAM_DEF			82
#define PWM_SAM_MIN			25
#define PWM_SAM_MAX			255

#define PWM_LG_DEF			80
#define PWM_LG_MIN			27
#define PWM_LG_MAX			200

#define PWM_LGWS2_DEF			80
#define PWM_LGWS2_MIN			27
#define PWM_LGWS2_MAX			200

static int flyer_adjust_backlight(enum led_brightness val);

extern int panel_type;
static DEFINE_MUTEX(panel_lock);
static void (*panel_power_gpio)(int on);
static struct wake_lock panel_idle_lock;

static atomic_t lcm_init_done = ATOMIC_INIT(1);
static uint8_t last_val = BRIGHTNESS_DEFAULT_LEVEL;
static bool screen_on = true;

struct lcm_cmd {
	uint8_t		cmd;
	uint8_t		data;
};

static void flyer_wsvga_panel_power(int on)
{
	if (panel_power_gpio)
		(*panel_power_gpio)(on);
	if (on == 1 && (panel_type == PANEL_ID_FLR_LG_XC || panel_type == PANEL_ID_FLR_LG_WS2 || panel_type == PANEL_ID_FLR_SMD_XC))
		screen_on = true;
}

static int lcdc_flyer_panel_on(struct platform_device *pdev)
{
	screen_on = true;
	LCMDBG("\n");
	mutex_lock(&panel_lock);
	atomic_set(&lcm_init_done, 1);
	mutex_unlock(&panel_lock);

	LCMDBG(": backlight to %d\n", last_val);
	flyer_adjust_backlight(last_val);
	return 0;
}

static int lcdc_flyer_panel_off(struct platform_device *pdev)
{
	screen_on = false;
	LCMDBG(": backlight off\n");
	flyer_adjust_backlight(0);
	mutex_lock(&panel_lock);
	atomic_set(&lcm_init_done, 0);
	mutex_unlock(&panel_lock);
	return 0;
}

static int flyer_shrink_pwm(int brightness, int user_def,
                int user_min, int user_max, int panel_def,
                int panel_min, int panel_max)
{
        if (brightness < PWM_USER_DIM) {
                return 0;
        }

        if (brightness < user_min) {
                return panel_min;
        }

        if (brightness > user_def) {
                brightness = (panel_max - panel_def) *
                        (brightness - user_def) /
                        (user_max - user_def) +
                        panel_def;
        } else {
                        brightness = (panel_def - panel_min) *
                        (brightness - user_min) /
                        (user_def - user_min) +
                        panel_min;
        }

        return brightness;
}

/*----------------------------------------------------------------------------*/

static int flyer_adjust_backlight(enum led_brightness val)
{
	uint8_t shrink_br = 0;
        uint8_t data[4] = {     /* PWM setting of microp, see p.8 */
                0x05,           /* Fading time; suggested: 5/10/15/20/25 */
                val,            /* Duty Cycle */
                0x00,           /* Channel H byte */
                0x20,           /* Channel L byte */
                };

	if(val == 0)
		data[0] = 0;

        mutex_lock(&panel_lock);

	if(panel_type == PANEL_ID_FLR_LG_XC)
		shrink_br = flyer_shrink_pwm(val, PWM_USER_DEF,
                                PWM_USER_MIN, PWM_USER_MAX, PWM_LG_DEF,
                                PWM_LG_MIN, PWM_LG_MAX);
	else if(panel_type == PANEL_ID_FLR_LG_WS2)
		shrink_br = flyer_shrink_pwm(val, PWM_USER_DEF,
                                PWM_USER_MIN, PWM_USER_MAX, PWM_LGWS2_DEF,
                                PWM_LGWS2_MIN, PWM_LGWS2_MAX);
	else
		shrink_br = flyer_shrink_pwm(val, PWM_USER_DEF,
                                PWM_USER_MIN, PWM_USER_MAX, PWM_SAM_DEF,
                                PWM_SAM_MIN, PWM_SAM_MAX);

        data[1] = shrink_br;

		PR_DISP_DEBUG("[lcm](%d), shrink_br=%d\n", val, shrink_br);
        microp_i2c_write(0x25, data, sizeof(data));
        last_val = shrink_br ? shrink_br: last_val;
        mutex_unlock(&panel_lock);

#if 0
        return shrink_br;
#else
	return val;
#endif
}

static void flyer_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
#if 0
	uint8_t data[4] = {	/* PWM setting of microp, see p.8 */
		0x05,		/* Fading time; suggested: 5/10/15/20/25 */
		128,		/* Duty Cycle */
		0x00,		/* Channel H byte */
		0x20,		/* Channel L byte */
		};

	if (atomic_read(&lcm_init_done) == 0) {
		last_val = val ? val : last_val;
		LCMDBG(":lcm not ready, val=%d\n", val);
		return;
	}
	mutex_lock(&panel_lock);
	LCMDBG("brightness=%d\n", val);
	microp_i2c_write(0x25, data, sizeof(data));
	last_val = val ? val : last_val;
	mutex_unlock(&panel_lock);
#else
	if (atomic_read(&lcm_init_done) == 0) {
		last_val = val ? val : last_val;
		LCMDBG(":lcm not ready, val=%d\n", val);
		return;
	}
	led_cdev->brightness = flyer_adjust_backlight(val);
#endif
}

static struct led_classdev flyer_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = flyer_brightness_set,
};

static int flyer_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &flyer_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device flyer_backlight = {
	.name = "flyer-backlight",
};

static struct platform_driver flyer_backlight_driver = {
	.probe          = flyer_backlight_probe,
	.driver         = {
		.name   = "flyer-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init flyer_wsvga_init_panel(void)
{
	int ret;

	/* set gpio to proper state in the beginning */
	flyer_wsvga_panel_power(1);

	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	ret = platform_device_register(&flyer_backlight);
	if (ret)
		return ret;

	return 0;
}

static int flyer_wsvga_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct msm_panel_common_pdata *lcdc_flyer_pdata;

	pr_info("%s: id=%d\n", __func__, pdev->id);

	/* power control */
	lcdc_flyer_pdata = pdev->dev.platform_data;
	panel_power_gpio = lcdc_flyer_pdata->panel_config_gpio;

	rc = flyer_wsvga_init_panel();
	if (rc)
		printk(KERN_ERR "%s fail %d\n", __func__, rc);

	return rc;
}

static struct platform_driver this_driver = {
	.probe = flyer_wsvga_probe,
	.driver = {
		.name = LCDC_FLYER_PANEL_NAME,
	},
};

static struct msm_fb_panel_data flyer_wsvga_panel_data= {
	.on = lcdc_flyer_panel_on,
	.off = lcdc_flyer_panel_off,
};

static struct platform_device this_device = {
	.name	= "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &flyer_wsvga_panel_data,
	},
};

static int __init flyer_wsvga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	if (msm_fb_detect_client(LCDC_FLYER_PANEL_NAME)) {
		pr_err("%s: detect failed\n", __func__);
		return 0;
	}

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}

	pinfo = &flyer_wsvga_panel_data.panel_info;
	pinfo->xres = 1024;
	pinfo->yres = 600;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);

	switch (panel_type)
	{
		case PANEL_ID_FLR_LG_XC:
		case PANEL_ID_FLR_LG_WS2:
			pinfo->clk_rate				= 53512000;
			pinfo->lcdc.h_back_porch	= 160;
			pinfo->lcdc.h_front_porch	= 160;
			pinfo->lcdc.h_pulse_width	= 30;
			pinfo->lcdc.v_back_porch	= 23;
			pinfo->lcdc.v_front_porch	= 12;
			pinfo->lcdc.v_pulse_width	= 10;
			break;
		case PANEL_ID_FLR_SMD_XC:
			pinfo->clk_rate				= 40960000;
			pinfo->lcdc.h_back_porch	= 60;
			pinfo->lcdc.h_front_porch	= 36;
			pinfo->lcdc.h_pulse_width	= 30;
			pinfo->lcdc.v_back_porch	= 11;
			pinfo->lcdc.v_front_porch	= 10;
			pinfo->lcdc.v_pulse_width	= 10;
			break;
		default:
			return -EINVAL;
			break;
	}

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			__func__);
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

static int __init flyer_backlight_init(void)
{
	return platform_driver_register(&flyer_backlight_driver);
}

device_initcall(flyer_wsvga_init);
module_init(flyer_backlight_init);
