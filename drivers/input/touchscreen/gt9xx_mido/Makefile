#gt915 touchpanel driver

EXTRA_CFLAGS +=-Wno-error=date-time
EXTRA_CFLAGS +=-Wno-date-time
EXTRA_CFLAGS += $(call cc-option,-funroll-loops,)

obj-$(CONFIG_GT9XX_TOUCHPANEL_DRIVER)	+= gt9xx.o
obj-$(CONFIG_GT9XX_TOUCHPANEL_UPDATE)	+= gt9xx_update.o
obj-$(CONFIG_GT9XX_TOUCHPANEL_DEBUG)	+= goodix_tool.o
