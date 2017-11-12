subdir-ccflags-$(CONFIG_ARCH_MSM) += -Iarch/arm/mach-msm

-include $(src)/Makefile.board

obj-y += drv_callback.o radio_feedback.o pwrtest.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_ARCH_MSM7X30) += devices_htc.o emmc_partitions.o msm_vibrator.o
obj-$(CONFIG_ARCH_MSM7X30) += htc_awb_cal.o htc_wifi_nvs.o htc_bdaddress.o htc_sdservice.o htc_lpm.o
	ifdef CONFIG_MSM_NONSMD_PACKET_FILTER
		obj-$(CONFIG_MSM_SMD) += htc_port_list_lte.o
	else
		obj-$(CONFIG_MSM_SMD) += htc_port_list.o
	endif
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_MSM7KV2_AUDIO) += htc_acoustic_7x30.o htc_acdb_7x30.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_HTC_HEADSET_MGR) += htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_GPIO) += htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP) += htc_headset_microp.o
obj-$(CONFIG_HTC_HEADSET_PMIC) += htc_headset_pmic.o
obj-$(CONFIG_HTC_HEADSET_MISC) += htc_headset_misc.o
obj-$(CONFIG_CABLE_DETECT_8X55) += cable_detect_8x55.o

CFLAGS_msm_vibrator.o += -Idrivers/staging/android
