# auto-detect subdirs
ifeq ($(CONFIG_ARCH_SDM845), y)
include $(srctree)/techpack/audio/config/sdm845auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm845auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM670), y)
include $(srctree)/techpack/audio/config/sdm710auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm710auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM632), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM450), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM439), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM429), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
include $(srctree)/techpack/audio/config/sdxpoorwillsauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdxpoorwillsauto.conf)
endif
ifeq ($(CONFIG_ARCH_QM215), y)
include $(srctree)/techpack/audio/config/msm8909auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/msm8909auto.conf)
endif
ifeq ($(CONFIG_ARCH_MSM8953), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_MSM8940), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_MSM8937), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_MSM8917), y)
include $(srctree)/techpack/audio/config/sdm450auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm450auto.conf)
endif
ifeq ($(CONFIG_ARCH_MSM8909), y)
include $(srctree)/techpack/audio/config/msm8909auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/msm8909auto.conf)
endif

# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/audio/include/uapi \

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/audio/include/uapi \
                -I$(srctree)/techpack/audio/include

ifeq ($(CONFIG_ARCH_SDM845), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm845autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM670), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm710autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM632), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM450), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM439), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM429), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdxpoorwillsautoconf.h
endif
ifeq ($(CONFIG_ARCH_QM215), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/msm8909autoconf.h
endif
ifeq ($(CONFIG_ARCH_MSM8953), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_MSM8940), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_MSM8937), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_MSM8917), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm450autoconf.h
endif
ifeq ($(CONFIG_ARCH_MSM8909), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/msm8909autoconf.h
endif

obj-y += asoc/
obj-y += dsp/
obj-y += ipc/
obj-y += soc/
