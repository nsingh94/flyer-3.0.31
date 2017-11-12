#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_AKM8975)   += akm8975.o
obj-$(CONFIG_SENSORS_BMA150)    += bma150.o
obj-$(CONFIG_SENSORS_CAPSENSE)	+= cap_sense.o
obj-$(CONFIG_TPS65200)          += tps65200.o
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

