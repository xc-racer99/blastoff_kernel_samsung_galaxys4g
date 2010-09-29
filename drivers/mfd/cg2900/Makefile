#
# Makefile for ST-Ericsson CG2900 connectivity combo controller
#

obj-$(CONFIG_MFD_CG2900)            += cg2900.o
cg2900-objs                         := cg2900_core.o cg2900_char_devices.o
export-objs                         := cg2900_core.o

obj-$(CONFIG_MFD_CG2900_CHIP)       += cg2900_chip.o
obj-$(CONFIG_MFD_STLC2690_CHIP)     += stlc2690_chip.o

obj-$(CONFIG_MFD_CG2900_UART)       += cg2900_uart.o

obj-$(CONFIG_MFD_CG2900_AUDIO)      += cg2900_audio.o
