#
# Makefile for the Omnivision TCM touchscreen driver.
#

# Each configuration option enables a list of files.
ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mtk_touchscreen_hodafonehw/

mtk_touchscreen_hodafonehw-$(CONFIG_TOUCHSCREEN_Q20_TOUCH) += q20_touch/s2716b_ts.o
