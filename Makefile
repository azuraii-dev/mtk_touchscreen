subdir-ccflags-y += -Wno-error
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mtk_touchscreen_hodafonehw
# In case the platform does NOT support this type of sensors

obj-$(CONFIG_TOUCHSCREEN_MTK_HODAFONEHW) += mtk_touchscreen_hodafonehw.o
mtk_touchscreen_hodafonehw-y :=
mtk_touchscreen_hodafonehw-y   +=  mtk_tpd.o
mtk_touchscreen_hodafonehw-y   +=  tpd_button.o
mtk_touchscreen_hodafonehw-y   +=  tpd_calibrate.o
mtk_touchscreen_hodafonehw-y   +=  tpd_debug.o
mtk_touchscreen_hodafonehw-y   +=  tpd_default.o
mtk_touchscreen_hodafonehw-y   +=  tpd_misc.o
mtk_touchscreen_hodafonehw-y   +=  tpd_setting.o
mtk_touchscreen_hodafonehw-y   += hodafone_gesture.o

include $(srctree)/drivers/input/touchscreen/mtk_touchscreen_hodafonehw/synaptics_dsx/Makefile
include $(srctree)/drivers/input/touchscreen/mtk_touchscreen_hodafonehw/q20_touch/Makefile
