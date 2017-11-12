/* arch/arm/mach-msm/proc_comm.h
 *
 * Copyright (c) 2007-2009,2011 The Linux Foundation. All rights reserved.
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

#ifndef _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_H_
#define _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_H_

enum {
	PCOM_CMD_IDLE = 0x0,
	PCOM_CMD_DONE,
	PCOM_RESET_APPS,
	PCOM_RESET_CHIP,
	PCOM_CONFIG_NAND_MPU,
	PCOM_CONFIG_USB_CLKS,
	PCOM_GET_POWER_ON_STATUS,
	PCOM_GET_WAKE_UP_STATUS,
	PCOM_GET_BATT_LEVEL,
	PCOM_CHG_IS_CHARGING,
	PCOM_POWER_DOWN,
	PCOM_USB_PIN_CONFIG,
	PCOM_USB_PIN_SEL,
	PCOM_SET_RTC_ALARM,
	PCOM_NV_READ,
	PCOM_NV_WRITE,
	PCOM_GET_UUID_HIGH,
	PCOM_GET_UUID_LOW,
	PCOM_GET_HW_ENTROPY,
	PCOM_RPC_GPIO_TLMM_CONFIG_REMOTE,
	PCOM_CLKCTL_RPC_ENABLE,
	PCOM_CLKCTL_RPC_DISABLE,
	PCOM_CLKCTL_RPC_RESET,
	PCOM_CLKCTL_RPC_SET_FLAGS,
	PCOM_CLKCTL_RPC_SET_RATE,
	PCOM_CLKCTL_RPC_MIN_RATE,
	PCOM_CLKCTL_RPC_MAX_RATE,
	PCOM_CLKCTL_RPC_RATE,
	PCOM_CLKCTL_RPC_PLL_REQUEST,
	PCOM_CLKCTL_RPC_ENABLED,
	PCOM_VREG_SWITCH,
	PCOM_VREG_SET_LEVEL,
	PCOM_GPIO_TLMM_CONFIG_GROUP,
	PCOM_GPIO_TLMM_UNCONFIG_GROUP,
	PCOM_NV_WRITE_BYTES_4_7,
	PCOM_CONFIG_DISP,
	PCOM_GET_FTM_BOOT_COUNT,
	PCOM_RPC_GPIO_TLMM_CONFIG_EX,
	PCOM_PM_MPP_CONFIG,
	PCOM_GPIO_IN,
	PCOM_GPIO_OUT,
	PCOM_RESET_MODEM,
	PCOM_RESET_CHIP_IMM,
	PCOM_PM_VID_EN,
	PCOM_VREG_PULLDOWN,
	PCOM_GET_MODEM_VERSION,
	PCOM_CLK_REGIME_SEC_RESET,
	PCOM_CLK_REGIME_SEC_RESET_ASSERT,
	PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
	PCOM_CLK_REGIME_SEC_PLL_REQUEST_WRP,
	PCOM_CLK_REGIME_SEC_ENABLE,
	PCOM_CLK_REGIME_SEC_DISABLE,
	PCOM_CLK_REGIME_SEC_IS_ON,
	PCOM_CLK_REGIME_SEC_SEL_CLK_INV,
	PCOM_CLK_REGIME_SEC_SEL_CLK_SRC,
	PCOM_CLK_REGIME_SEC_SEL_CLK_DIV,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_ENABLE,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_DISABLE,
	PCOM_CLK_REGIME_SEC_SEL_SPEED,
	PCOM_CLK_REGIME_SEC_CONFIG_GP_CLK_WRP,
	PCOM_CLK_REGIME_SEC_CONFIG_MDH_CLK_WRP,
	PCOM_CLK_REGIME_SEC_USB_XTAL_ON,
	PCOM_CLK_REGIME_SEC_USB_XTAL_OFF,
	PCOM_CLK_REGIME_SEC_SET_QDSP_DME_MODE,
	PCOM_CLK_REGIME_SEC_SWITCH_ADSP_CLK,
	PCOM_CLK_REGIME_SEC_GET_MAX_ADSP_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_GET_I2C_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_MSM_GET_CLK_FREQ_KHZ,
	PCOM_CLK_REGIME_SEC_SEL_VFE_SRC,
	PCOM_CLK_REGIME_SEC_MSM_SEL_CAMCLK,
	PCOM_CLK_REGIME_SEC_MSM_SEL_LCDCLK,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_ON,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_ON,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_ON,
	PCOM_CLK_REGIME_SEC_LCD_CTRL,
	PCOM_CLK_REGIME_SEC_REGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_DEREGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_RESOURCE_REQUEST_WRP,
	PCOM_CLK_REGIME_MSM_SEC_SEL_CLK_OWNER,
	PCOM_CLK_REGIME_SEC_DEVMAN_REQUEST_WRP,
	PCOM_GPIO_CONFIG,
	PCOM_GPIO_CONFIGURE_GROUP,
	PCOM_GPIO_TLMM_SET_PORT,
	PCOM_GPIO_TLMM_CONFIG_EX,
	PCOM_SET_FTM_BOOT_COUNT,
	PCOM_RESERVED0,
	PCOM_RESERVED1,
	PCOM_CUSTOMER_CMD1,
	PCOM_CUSTOMER_CMD2,
	PCOM_CUSTOMER_CMD3,
	PCOM_CLK_REGIME_ENTER_APPSBL_CHG_MODE,
	PCOM_CLK_REGIME_EXIT_APPSBL_CHG_MODE,
	PCOM_CLK_REGIME_SEC_RAIL_DISABLE,
	PCOM_CLK_REGIME_SEC_RAIL_ENABLE,
	PCOM_CLK_REGIME_SEC_RAIL_CONTROL,
	PCOM_SET_SW_WATCHDOG_STATE,
	PCOM_PM_MPP_CONFIG_DIGITAL_INPUT,
	PCOM_PM_MPP_CONFIG_I_SINK,
	PCOM_RESERVED_101,
	PCOM_MSM_HSUSB_PHY_RESET,
	PCOM_GET_BATT_MV_LEVEL,
	PCOM_CHG_USB_IS_PC_CONNECTED,
	PCOM_CHG_USB_IS_CHARGER_CONNECTED,
	PCOM_CHG_USB_IS_DISCONNECTED,
	PCOM_CHG_USB_IS_AVAILABLE,
	PCOM_CLK_REGIME_SEC_MSM_SEL_FREQ,
	PCOM_CLK_REGIME_SEC_SET_PCLK_AXI_POLICY,
	PCOM_CLKCTL_RPC_RESET_ASSERT,
	PCOM_CLKCTL_RPC_RESET_DEASSERT,
	PCOM_CLKCTL_RPC_RAIL_ON,
	PCOM_CLKCTL_RPC_RAIL_OFF,
	PCOM_CLKCTL_RPC_RAIL_ENABLE,
	PCOM_CLKCTL_RPC_RAIL_DISABLE,
	PCOM_CLKCTL_RPC_RAIL_CONTROL,
	PCOM_CLKCTL_RPC_MIN_MSMC1,
	PCOM_CLKCTL_RPC_SRC_REQUEST,
	PCOM_NPA_INIT,
	PCOM_NPA_ISSUE_REQUIRED_REQUEST,
	PCOM_CLKCTL_RPC_SET_EXT_CONFIG,
};

enum {
	PCOM_OEM_FIRST_CMD = 0x10000000,
	PCOM_OEM_TEST_CMD = PCOM_OEM_FIRST_CMD,

	/* add OEM PROC COMM commands here */

	PCOM_OEM_LAST = PCOM_OEM_TEST_CMD,
};

enum {
	PCOM_INVALID_STATUS = 0x0,
	PCOM_READY,
	PCOM_CMD_RUNNING,
	PCOM_CMD_SUCCESS,
	PCOM_CMD_FAIL,
	PCOM_CMD_FAIL_FALSE_RETURNED,
	PCOM_CMD_FAIL_CMD_OUT_OF_BOUNDS_SERVER,
	PCOM_CMD_FAIL_CMD_OUT_OF_BOUNDS_CLIENT,
	PCOM_CMD_FAIL_CMD_UNREGISTERED,
	PCOM_CMD_FAIL_CMD_LOCKED,
	PCOM_CMD_FAIL_SERVER_NOT_YET_READY,
	PCOM_CMD_FAIL_BAD_DESTINATION,
	PCOM_CMD_FAIL_SERVER_RESET,
	PCOM_CMD_FAIL_SMSM_NOT_INIT,
	PCOM_CMD_FAIL_PROC_COMM_BUSY,
	PCOM_CMD_FAIL_PROC_COMM_NOT_INIT,
};

#ifdef CONFIG_MACH_HTC
/* List of VREGs that support the Pull Down Resistor setting. */
enum vreg_pdown_id {
	PM_VREG_PDOWN_MSMA_ID,
	PM_VREG_PDOWN_MSMP_ID,
	PM_VREG_PDOWN_MSME1_ID,	/* Not supported in Panoramix */
	PM_VREG_PDOWN_MSMC1_ID,	/* Not supported in PM6620 */
	PM_VREG_PDOWN_MSMC2_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_GP3_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_MSME2_ID,	/* Supported in PM7500 and Panoramix only */
	PM_VREG_PDOWN_GP4_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_GP1_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_TCXO_ID,
	PM_VREG_PDOWN_PA_ID,
	PM_VREG_PDOWN_RFTX_ID,
	PM_VREG_PDOWN_RFRX1_ID,
	PM_VREG_PDOWN_RFRX2_ID,
	PM_VREG_PDOWN_SYNT_ID,
	PM_VREG_PDOWN_WLAN_ID,
	PM_VREG_PDOWN_USB_ID,
	PM_VREG_PDOWN_MMC_ID,
	PM_VREG_PDOWN_RUIM_ID,
	PM_VREG_PDOWN_MSMC0_ID,	/* Supported in PM6610 only */
	PM_VREG_PDOWN_GP2_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_GP5_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_GP6_ID,	/* Supported in PM7500 only */
	PM_VREG_PDOWN_RF_ID,
	PM_VREG_PDOWN_RF_VCO_ID,
	PM_VREG_PDOWN_MPLL_ID,
	PM_VREG_PDOWN_S2_ID,
	PM_VREG_PDOWN_S3_ID,
	PM_VREG_PDOWN_RFUBM_ID,

	/* new for HAN */
	PM_VREG_PDOWN_RF1_ID,
	PM_VREG_PDOWN_RF2_ID,
	PM_VREG_PDOWN_RFA_ID,
	PM_VREG_PDOWN_CDC2_ID,
	PM_VREG_PDOWN_RFTX2_ID,
	PM_VREG_PDOWN_USIM_ID,
	PM_VREG_PDOWN_USB2P6_ID,
	PM_VREG_PDOWN_USB3P3_ID,
	PM_VREG_PDOWN_INVALID_ID,

	/* backward compatible enums only */
	PM_VREG_PDOWN_CAM_ID = PM_VREG_PDOWN_GP1_ID,
	PM_VREG_PDOWN_MDDI_ID = PM_VREG_PDOWN_GP2_ID,
	PM_VREG_PDOWN_RUIM2_ID = PM_VREG_PDOWN_GP3_ID,
	PM_VREG_PDOWN_AUX_ID = PM_VREG_PDOWN_GP4_ID,
	PM_VREG_PDOWN_AUX2_ID = PM_VREG_PDOWN_GP5_ID,
	PM_VREG_PDOWN_BT_ID = PM_VREG_PDOWN_GP6_ID,

	PM_VREG_PDOWN_MSME_ID = PM_VREG_PDOWN_MSME1_ID,
	PM_VREG_PDOWN_MSMC_ID = PM_VREG_PDOWN_MSMC1_ID,
	PM_VREG_PDOWN_RFA1_ID = PM_VREG_PDOWN_RFRX2_ID,
	PM_VREG_PDOWN_RFA2_ID = PM_VREG_PDOWN_RFTX2_ID,
	PM_VREG_PDOWN_XO_ID = PM_VREG_PDOWN_TCXO_ID
};

enum {
	PCOM_CLKRGM_APPS_RESET_USB_PHY  = 34,
	PCOM_CLKRGM_APPS_RESET_USBH     = 37,
};

/* gpio info for PCOM_RPC_GPIO_TLMM_CONFIG_EX */

#define GPIO_ENABLE	0
#define GPIO_DISABLE	1

#define GPIO_INPUT	0
#define GPIO_OUTPUT	1

#define GPIO_NO_PULL	0
#define GPIO_PULL_DOWN	1
#define GPIO_KEEPER	2
#define GPIO_PULL_UP	3

#define GPIO_2MA	0
#define GPIO_4MA	1
#define GPIO_6MA	2
#define GPIO_8MA	3
#define GPIO_10MA	4
#define GPIO_12MA	5
#define GPIO_14MA	6
#define GPIO_16MA	7

#define PCOM_GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)	| \
		((func) & 0xf)			| \
		(((dir) & 0x1) << 14)		| \
		(((pull) & 0x3) << 15)		| \
		(((drvstr) & 0xF) << 17))

#endif

#ifdef CONFIG_MSM_PROC_COMM
void msm_proc_comm_reset_modem_now(void);
int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2);
#else
static inline void msm_proc_comm_reset_modem_now(void) { }
static inline int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{ return 0; }
#endif

#endif
