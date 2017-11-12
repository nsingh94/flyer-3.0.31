/* linux/arch/arm/mach-msm/board-flyer-panel.c
 *
 * Copyright (c) 2010 HTC.
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

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_memtypes.h>
#include <mach/atmega_microp.h>
#include <mach/vreg.h>
#include <mach/panel_id.h>

#include "board-flyer.h"
#include "devices.h"
#include "proc_comm.h"
#include "../../../drivers/video/msm/mdp_hw.h"

#define DEBUG_LCM

#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)     printk("[lcm]%s"fmt, __func__, ##arg)
#else
#define LCMDBG(fmt, arg...)     {}
#endif

#define LCDC_FLYER_PANEL_NAME			"lcdc_flyer"

enum {
	PANEL_ID_FLR_SMD_XB,
};

extern int panel_type;
static struct regulator *vreg_ldo19;

#define LCM_GPIO_CFG(gpio, func) \
	PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)

static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(FLYER_LCD_PCLK, 1),
	LCM_GPIO_CFG(FLYER_LCD_DE, 1),
	LCM_GPIO_CFG(FLYER_LCD_VSYNC, 1),
	LCM_GPIO_CFG(FLYER_LCD_HSYNC, 1),
	LCM_GPIO_CFG(FLYER_LCD_G0, 1),
	LCM_GPIO_CFG(FLYER_LCD_G1, 1),
	LCM_GPIO_CFG(FLYER_LCD_G2, 1),
	LCM_GPIO_CFG(FLYER_LCD_G3, 1),
	LCM_GPIO_CFG(FLYER_LCD_G4, 1),
	LCM_GPIO_CFG(FLYER_LCD_G5, 1),
	LCM_GPIO_CFG(FLYER_LCD_G6, 1),
	LCM_GPIO_CFG(FLYER_LCD_G7, 1),
	LCM_GPIO_CFG(FLYER_LCD_B0, 1),
	LCM_GPIO_CFG(FLYER_LCD_B1, 1),
	LCM_GPIO_CFG(FLYER_LCD_B2, 1),
	LCM_GPIO_CFG(FLYER_LCD_B3, 1),
	LCM_GPIO_CFG(FLYER_LCD_B4, 1),
	LCM_GPIO_CFG(FLYER_LCD_B5, 1),
	LCM_GPIO_CFG(FLYER_LCD_B6, 1),
	LCM_GPIO_CFG(FLYER_LCD_B7, 1),
	LCM_GPIO_CFG(FLYER_LCD_R0, 1),
	LCM_GPIO_CFG(FLYER_LCD_R1, 1),
	LCM_GPIO_CFG(FLYER_LCD_R2, 1),
	LCM_GPIO_CFG(FLYER_LCD_R3, 1),
	LCM_GPIO_CFG(FLYER_LCD_R4, 1),
	LCM_GPIO_CFG(FLYER_LCD_R5, 1),
	LCM_GPIO_CFG(FLYER_LCD_R6, 1),
	LCM_GPIO_CFG(FLYER_LCD_R7, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(FLYER_LCD_PCLK, 0),
	LCM_GPIO_CFG(FLYER_LCD_DE, 0),
	LCM_GPIO_CFG(FLYER_LCD_VSYNC, 0),
	LCM_GPIO_CFG(FLYER_LCD_HSYNC, 0),
	LCM_GPIO_CFG(FLYER_LCD_G0, 0),
	LCM_GPIO_CFG(FLYER_LCD_G1, 0),
	LCM_GPIO_CFG(FLYER_LCD_G2, 0),
	LCM_GPIO_CFG(FLYER_LCD_G3, 0),
	LCM_GPIO_CFG(FLYER_LCD_G4, 0),
	LCM_GPIO_CFG(FLYER_LCD_G5, 0),
	LCM_GPIO_CFG(FLYER_LCD_G6, 0),
	LCM_GPIO_CFG(FLYER_LCD_G7, 0),
	LCM_GPIO_CFG(FLYER_LCD_B0, 0),
	LCM_GPIO_CFG(FLYER_LCD_B1, 0),
	LCM_GPIO_CFG(FLYER_LCD_B2, 0),
	LCM_GPIO_CFG(FLYER_LCD_B3, 0),
	LCM_GPIO_CFG(FLYER_LCD_B4, 0),
	LCM_GPIO_CFG(FLYER_LCD_B5, 0),
	LCM_GPIO_CFG(FLYER_LCD_B6, 0),
	LCM_GPIO_CFG(FLYER_LCD_B7, 0),
	LCM_GPIO_CFG(FLYER_LCD_R0, 0),
	LCM_GPIO_CFG(FLYER_LCD_R1, 0),
	LCM_GPIO_CFG(FLYER_LCD_R2, 0),
	LCM_GPIO_CFG(FLYER_LCD_R3, 0),
	LCM_GPIO_CFG(FLYER_LCD_R4, 0),
	LCM_GPIO_CFG(FLYER_LCD_R5, 0),
	LCM_GPIO_CFG(FLYER_LCD_R6, 0),
	LCM_GPIO_CFG(FLYER_LCD_R7, 0),
};

static int panel_gpio_switch(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table : display_off_gpio_table,
		!!on ? ARRAY_SIZE(display_on_gpio_table) : ARRAY_SIZE(display_off_gpio_table));

	return 0;
}

static void flyer_panel_power(bool on_off)
{
	int gpio_lcm_en, gpio_lvds_on;

	if (panel_type != PANEL_ID_FLR_SMD_XB) {
		gpio_lcm_en = FLYER_LCM_3V3_EN_XC;
		gpio_lvds_on = FLYER_LVDS_ON_XC;
	} else {
		gpio_lcm_en = FLYER_LCM_3V3_EN;
		gpio_lvds_on = FLYER_LVDS_ON;
	}

	if (!!on_off) {
		LCMDBG("%s(%d):\n", __func__, on_off);
		regulator_enable(vreg_ldo19);
		gpio_set_value(gpio_lcm_en, 1);
		if(panel_type == PANEL_ID_FLR_LG_XC || panel_type == PANEL_ID_FLR_LG_WS2)
			msleep(50);
		else
			msleep(90);
		gpio_set_value(gpio_lvds_on, 1);
	} else {
		gpio_set_value(gpio_lcm_en, 0);
		if(panel_type == PANEL_ID_FLR_LG_XC || panel_type == PANEL_ID_FLR_LG_WS2)
			msleep(50);
		else
			msleep(60);
		gpio_set_value(gpio_lvds_on, 0);
		regulator_disable(vreg_ldo19);
	}


}

static int panel_power(int on)
{
	flyer_panel_power(on == 1 ? true : false);
	return 0;
}

int device_fb_detect_panel(const char *name)
{
	if (!strcmp(name, LCDC_FLYER_PANEL_NAME)) {
		return 0;
	}
	return 0;
}

/* a hacky interface to control the panel power */
static void lcdc_config_gpios(int on)
{
	printk(KERN_INFO "%s: power goes to %d\n", __func__, on);

	if (panel_power(on))
		printk(KERN_ERR "%s: panel_power failed!\n", __func__);
	if (panel_gpio_switch(on))
		printk(KERN_ERR "%s: panel_gpio_switch failed!\n", __func__);
}

static struct msm_panel_common_pdata lcdc_panel_data = {
	.panel_config_gpio = lcdc_config_gpios,
};

static struct platform_device lcdc_flyer_panel_device = {
	.name   = LCDC_FLYER_PANEL_NAME,
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_panel_data,
	}
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_max_clk = 192000000,
	.mdp_rev = MDP_REV_40,
	.mem_hid = MEMTYPE_EBI0,
};

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	return panel_power(on);
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save = lcdc_panel_power,
};

static int panel_init_power(void)
{
//	int rc;

	vreg_ldo19 = regulator_get(0, "wlan2");
	if (IS_ERR(vreg_ldo19))
		return PTR_ERR(vreg_ldo19);
	return 0;
}

int __init flyer_init_panel(void)
{
	int ret;

	ret = panel_init_power();
	if (ret)
		return ret;

	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);

	ret = platform_device_register(&lcdc_flyer_panel_device);
	if (ret != 0)
		return ret;

	return 0;
}
