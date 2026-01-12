#
# Makefile for the Omnivision TCM touchscreen driver.
#

# Each configuration option enables a list of files.
ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mtk_touchscreen_hodafonehw/

mtk_touchscreen_hodafonehw-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C) += synaptics_dsx/synaptics_dsx_core.o
mtk_touchscreen_hodafonehw-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C) += synaptics_dsx/synaptics_dsx_i2c.o
mtk_touchscreen_hodafonehw-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) +=  synaptics_dsx/synaptics_dsx_fw_update.o
