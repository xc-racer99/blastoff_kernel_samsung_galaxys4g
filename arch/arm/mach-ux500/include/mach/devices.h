/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __ASM_ARCH_DEVICES_H__
#define __ASM_ARCH_DEVICES_H__

struct platform_device;
struct amba_device;

extern struct platform_device u5500_gpio_devs[];
extern struct platform_device u8500_gpio_devs[];

extern struct amba_device ux500_pl031_device;
extern struct amba_device u8500_ssp0_device;
extern struct amba_device ux500_uart0_device;
extern struct amba_device ux500_uart1_device;
extern struct amba_device ux500_uart2_device;

extern struct platform_device ux500_i2c1_device;
extern struct platform_device ux500_i2c2_device;
extern struct platform_device ux500_i2c3_device;

extern struct platform_device u8500_i2c0_device;
extern struct platform_device u8500_i2c4_device;
extern struct platform_device u8500_dma40_device;
extern struct platform_device ux500_mcde_device;
extern struct platform_device u8500_hsit_device;
extern struct platform_device u8500_hsir_device;
extern struct platform_device u8500_shrm_device;
extern struct platform_device ux500_b2r2_device;
extern struct platform_device ux500_hwmem_device;
extern struct amba_device ux500_rtc_device;
extern struct platform_device ux500_hash1_device;
extern struct platform_device ux500_musb_device;
extern struct platform_device ux500_cg2900_device;
extern struct platform_device u5500_pwm0_device;
extern struct platform_device u5500_pwm1_device;
extern struct platform_device u5500_pwm2_device;
extern struct platform_device u5500_pwm3_device;

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/**
 * Touchpanel related macros declaration
 */
#define TOUCH_GPIO_PIN 	84

void dma40_u8500ed_fixup(void);

#endif
