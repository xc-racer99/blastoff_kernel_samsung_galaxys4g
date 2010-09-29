#
# Makefile for the linux kernel, U8500 machine.
#

obj-y				:= clock.o cpu.o devices.o
obj-$(CONFIG_UX500_SOC_DB5500)	+= cpu-db5500.o devices-db5500.o
obj-$(CONFIG_UX500_SOC_DB8500)	+= cpu-db8500.o devices-db8500.o
obj-$(CONFIG_MACH_U8500_MOP)	+= board-mop500.o
obj-$(CONFIG_MACH_U5500)	+= board-u5500.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_LOCAL_TIMERS)	+= localtimer.o
obj-$(CONFIG_U5500_MODEM_IRQ)	+= modem_irq.o
obj-$(CONFIG_U5500_MBOX)	+= mbox.o
obj-$(CONFIG_U5500_PWM)			+= pwm.o
obj-$(CONFIG_MOP500_NUIB)		+= board-mop500-nuib.o
obj-$(CONFIG_TEE_UX500)			+= tee_ux500.o
obj-$(CONFIG_TEE_SVP)			+= tee_service_svp.o
obj-$(CONFIG_TEE_SVP)			+= tee_ta_start_modem_svp.o
obj-$(CONFIG_MFD_CG2900)		+= devices-cg2900.o
