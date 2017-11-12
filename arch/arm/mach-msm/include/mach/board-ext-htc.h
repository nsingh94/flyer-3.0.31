/* arch/arm/mach-msm/include/mach/BOARD-EXT-HTC.h
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_MSM_BOARD_HTC_H
#define __ASM_ARCH_MSM_BOARD_HTC_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

struct msm_pmem_setting{
	resource_size_t pmem_start;
	resource_size_t pmem_size;
	resource_size_t pmem_adsp_start;
	resource_size_t pmem_adsp_size;
	resource_size_t pmem_gpu0_start;
	resource_size_t pmem_gpu0_size;
	resource_size_t pmem_gpu1_start;
	resource_size_t pmem_gpu1_size;
	resource_size_t pmem_camera_start;
	resource_size_t pmem_camera_size;
	resource_size_t ram_console_start;
	resource_size_t ram_console_size;
};

enum {
	MSM_SERIAL_UART1	= 0,
	MSM_SERIAL_UART2,
	MSM_SERIAL_UART3,
#ifdef CONFIG_SERIAL_MSM_HS
	MSM_SERIAL_UART1DM,
	MSM_SERIAL_UART2DM,
#endif
	MSM_SERIAL_NUM,
};

enum {
	KERNEL_FLAG_WDOG_DISABLE = BIT(0),
	KERNEL_FLAG_SERIAL_HSL_ENABLE = BIT(1),
	KERNEL_FLAG_RESERVED_2 = BIT(2),
	KERNEL_FLAG_WDOG_FIQ = BIT(3),
	KERNEL_FLAG_KMEMLEAK = BIT(4),
	KERNEL_FLAG_RMT_STORAGE = BIT(5),
	KERNEL_FLAG_MDM_CHARM_DEBUG = BIT(6),
	KERNEL_FLAG_MSM_SMD_DEBUG = BIT(7),
	KERNEL_FLAG_RESERVED_8 = BIT(8),
	KERNEL_FLAG_RESERVED_9 = BIT(9),
	KERNEL_FLAG_RESERVED_10 = BIT(10),
	KERNEL_FLAG_RESERVED_11 = BIT(11),
	KERNEL_FLAG_RESERVED_12 = BIT(12),
	KERNEL_FLAG_RESERVED_13 = BIT(13),
	KERNEL_FLAG_RESERVED_14 = BIT(14),
	KERNEL_FLAG_TEST_PWR_SUPPLY = BIT(15),
	KERNEL_FLAG_RIL_DBG_DMUX = BIT(16),
	KERNEL_FLAG_RIL_DBG_RMNET = BIT(17),
	KERNEL_FLAG_RIL_DBG_CMUX = BIT(18),
	KERNEL_FLAG_RIL_DBG_DATA_LPM = BIT(19),
	KERNEL_FLAG_RIL_DBG_CTL = BIT(20),
	KERNEL_FLAG_RIL_DBG_ALDEBUG_LAWDATA = BIT(21),
	KERNEL_FLAG_RIL_DBG_MEMCPY = BIT(22),
	KERNEL_FLAG_RIL_DBG_RPC = BIT(23),
	KERNEL_FLAG_RESERVED_24 = BIT(24),
	KERNEL_FLAG_PM_MONITOR = BIT(25),
	KERNEL_FLAG_ENABLE_FAST_CHARGE = BIT(26),
	KERNEL_FLAG_RESERVED_27 = BIT(27),
	KERNEL_FLAG_RESERVED_28 = BIT(28),
	KERNEL_FLAG_RESERVED_29 = BIT(29),
	KERNEL_FLAG_RESERVED_30 = BIT(30),
	KERNEL_FLAG_GPIO_DUMP = BIT(31),
};

#define SHIP_BUILD	0
#define MFG_BUILD	1
#define ENG_BUILD	2

/* START: add USB connected notify function */
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int htc_usb_register_notifier(struct t_usb_status_notifier *notifer);
static LIST_HEAD(g_lh_usb_notifier_list);

/***********************************
Direction: cable detect drvier -> battery driver or other
***********************************/
struct t_cable_status_notifier{
	struct list_head cable_notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int cable_detect_register_notifier(struct t_cable_status_notifier *);
static LIST_HEAD(g_lh_calbe_detect_notifier_list);

/***********************************
Direction: 1-wire drvier -> battery driver or other
***********************************/
struct t_owe_charging_notifier{
	struct list_head owe_charging_notifier_link;
	const char *name;
	void (*func)(int charging_type);
};
int owe_charging_register_notifier(struct t_owe_charging_notifier *);
static LIST_HEAD(g_lh_owe_charging_notifier_list);

/***********************************
 Direction: sii9234 drvier -> cable detect driver
***********************************/
struct t_mhl_status_notifier{
	struct list_head mhl_notifier_link;
	const char *name;
	void (*func)(bool isMHL, int charging_type);
};
int mhl_detect_register_notifier(struct t_mhl_status_notifier *);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);

/***********************************
Direction: cable detect drvier -> usb driver
 ***********************************/
struct t_usb_host_status_notifier{
	struct list_head usb_host_notifier_link;
	const char *name;
	void (*func)(bool cable_in);
};
int usb_host_detect_register_notifier(struct t_usb_host_status_notifier *);
static LIST_HEAD(g_lh_usb_host_detect_notifier_list);
/* END: add USB connected notify function */

int usb_get_connect_type(void);
void msm_otg_set_vbus_state(int online);
//void msm_hsusb_set_vbus_state(int online);
void msm_hsusb_vbus_notif_register(void (*vbus_notif)(int));
enum usb_connect_type {
	CONNECT_TYPE_CLEAR = -2,
	CONNECT_TYPE_UNKNOWN = -1,
	CONNECT_TYPE_NONE = 0,
	CONNECT_TYPE_USB,
	CONNECT_TYPE_AC,
	CONNECT_TYPE_9V_AC,
	CONNECT_TYPE_WIRELESS,
	CONNECT_TYPE_INTERNAL,
	CONNECT_TYPE_UNSUPPORTED,
	CONNECT_TYPE_USB_9V_AC,
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_usb_devices(void (*phy_reset) (void));
void __init msm_add_mem_devices(struct msm_pmem_setting *setting);
void __init msm_init_pmic_vibrator(int);

struct mmc_platform_data;
int __init msm_add_sdcc_devices(unsigned int controller, struct mmc_platform_data *plat);
int __init msm_add_serial_devices(unsigned uart);

int board_mfg_mode(void);
int board_build_flag(void);
int __init parse_tag_skuid(const struct tag *tags);
int __init parse_tag_memsize(const struct tag *tags);
int __init parse_tag_ddr_id(const struct tag *tags);
int __init parse_tag_engineerid(const struct tag *tags);
int __init parse_tag_cam(const struct tag *tag);

int __init board_mfg_mode(void);
int __init parse_tag_smi(const struct tag *tags);
int __init parse_tag_hwid(const struct tag * tags);

int parse_tag_engineerid(const struct tag * tags);
void board_get_sku_color_tag(char **);

char *board_serialno(void);
unsigned long get_kernel_flag(void);
unsigned int get_radio_flag(void);
char *get_model_id(void);
unsigned get_engineerid(void);

#endif
