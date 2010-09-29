/*
 * arch/arm/mach-ux500/devices-cg2900.c
 *
 * Copyright (C) ST-Ericsson SA 2010
 * Authors:
 * Par-Gunnar Hjalmdahl (par-gunnar.p.hjalmdahl@stericsson.com) for ST-Ericsson.
 * Henrik Possung (henrik.possung@stericsson.com) for ST-Ericsson.
 * Josef Kindberg (josef.kindberg@stericsson.com) for ST-Ericsson.
 * Dariusz Szymszak (dariusz.xd.szymczak@stericsson.com) for ST-Ericsson.
 * Kjell Andersson (kjell.k.andersson@stericsson.com) for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Board specific device support for the Linux Bluetooth HCI H:4 Driver
 * for ST-Ericsson connectivity controller.
 */

#include <asm/byteorder.h>
#include <asm-generic/errno-base.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mfd/cg2900.h>
#include <plat/pincfg.h>

#include "pins-db8500.h"

#ifdef CONFIG_MFD_CG2900

#define BT_ENABLE_GPIO			170
#define GBF_ENA_RESET_GPIO		171
#define BT_CTS_GPIO			0

#define GBF_ENA_RESET_NAME		"gbf_ena_reset"
#define BT_ENABLE_NAME			"bt_enable"
#define CG2900_NAME			"cg2900_devices"

#define UART_LINES_NUM			4

#define BT_VS_POWER_SWITCH_OFF		0xFD40

#define H4_HEADER_LENGTH		0x01
#define BT_HEADER_LENGTH		0x03

#define STLC2690_HCI_REV		0x0600
#define CG2900_PG1_HCI_REV		0x0101
#define CG2900_PG2_HCI_REV		0x0200
#define CG2900_PG1_SPECIAL_HCI_REV	0x0700

struct vs_power_sw_off_cmd {
	__le16	op_code;
	u8	len;
	u8	gpio_0_7_pull_up;
	u8	gpio_8_15_pull_up;
	u8	gpio_16_20_pull_up;
	u8	gpio_0_7_pull_down;
	u8	gpio_8_15_pull_down;
	u8	gpio_16_20_pull_down;
} __attribute__((packed));

static u16 cg2900_hci_revision;

/* Pin configuration for UART functions. */
static pin_cfg_t uart0_enabled[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH
};

/* Pin configuration for sleep mode. */
static pin_cfg_t uart0_disabled[] = {
	GPIO0_GPIO   | PIN_INPUT_PULLUP,	/* CTS pull up. */
	GPIO1_GPIO   | PIN_OUTPUT_HIGH,		/* RTS high - flow off. */
	GPIO2_GPIO   | PIN_INPUT_PULLUP,	/* RX pull down. */
	GPIO3_GPIO   | PIN_OUTPUT_LOW		/* TX low - break on. */
};

static void cg2900_enable_chip(void)
{
	gpio_set_value(GBF_ENA_RESET_GPIO, 1);
}

static void cg2900_disable_chip(void)
{
	gpio_set_value(GBF_ENA_RESET_GPIO, 0);
}

static void cg2900_set_hci_revision(u8 hci_version, u16 hci_revision,
				    u8 lmp_version, u8 lmp_subversion,
				    u16 manufacturer)
{
	cg2900_hci_revision = hci_revision;
	/* We don't care about the other values */
}

static struct sk_buff *cg2900_get_power_switch_off_cmd(u16 *op_code)
{
	struct sk_buff *skb;
	struct vs_power_sw_off_cmd *cmd;

	/* If connected chip does not support the command return NULL */
	if (CG2900_PG1_SPECIAL_HCI_REV != cg2900_hci_revision &&
	    CG2900_PG1_HCI_REV != cg2900_hci_revision &&
	    CG2900_PG2_HCI_REV != cg2900_hci_revision)
		return NULL;

	skb = alloc_skb(sizeof(*cmd) + H4_HEADER_LENGTH, GFP_KERNEL);
	if (!skb) {
		pr_err(CG2900_NAME "Could not allocate skb");
		return NULL;
	}

	skb_reserve(skb, H4_HEADER_LENGTH);
	cmd = (struct vs_power_sw_off_cmd *)skb_put(skb, sizeof(*cmd));
	cmd->op_code = cpu_to_le16(BT_VS_POWER_SWITCH_OFF);
	cmd->len = sizeof(*cmd) - BT_HEADER_LENGTH;
	/*
	 * Enter system specific GPIO settings here:
	 * Section data[3-5] is GPIO pull-up selection
	 * Section data[6-8] is GPIO pull-down selection
	 * Each section is a bitfield where
	 * - byte 0 bit 0 is GPIO 0
	 * - byte 0 bit 1 is GPIO 1
	 * - up to
	 * - byte 2 bit 4 which is GPIO 20
	 * where each bit means:
	 * - 0: No pull-up / no pull-down
	 * - 1: Pull-up / pull-down
	 * All GPIOs are set as input.
	 */
	cmd->gpio_0_7_pull_up = 0x00;
	cmd->gpio_8_15_pull_up = 0x00;
	cmd->gpio_16_20_pull_up = 0x00;
	cmd->gpio_0_7_pull_down = 0x00;
	cmd->gpio_8_15_pull_down = 0x00;
	cmd->gpio_16_20_pull_down = 0x00;

	if (op_code)
		*op_code = BT_VS_POWER_SWITCH_OFF;

	return skb;
}
static int cg2900_init(void)
{
	int err = 0;

	err = gpio_request(GBF_ENA_RESET_GPIO, GBF_ENA_RESET_NAME);
	if (err < 0) {
		pr_err(CG2900_NAME "gpio_request failed with err: %d", err);
		goto finished;
	}

	err = gpio_direction_output(GBF_ENA_RESET_GPIO, 1);
	if (err < 0) {
		pr_err(CG2900_NAME "gpio_direction_output failed with err: %d",
		       err);
		goto error_handling;
	}

	err = gpio_request(BT_ENABLE_GPIO, BT_ENABLE_NAME);
	if (err < 0) {
		pr_err(CG2900_NAME "gpio_request failed with err: %d", err);
		goto finished;
	}

	err = gpio_direction_output(BT_ENABLE_GPIO, 1);
	if (err < 0) {
		pr_err(CG2900_NAME "gpio_direction_output failed with err: %d",
		       err);
		goto error_handling;
	}

	goto finished;

error_handling:
	gpio_free(GBF_ENA_RESET_GPIO);

finished:
	cg2900_disable_chip();
	return err;
}

void cg2900_exit(void)
{
	cg2900_disable_chip();
	gpio_free(GBF_ENA_RESET_GPIO);
}

#ifdef CONFIG_MFD_CG2900_UART

static int cg2900_disable_uart(void)
{
	int err;

	/*
	 * Without this delay we get interrupt on CTS immediately
	 * due to some turbulences on this line.
	 */
	mdelay(4);

	/* Disable UART functions. */
	err = nmk_config_pins(uart0_disabled, UART_LINES_NUM);
	if (err)
		goto error;

	return 0;

error:
	(void)nmk_config_pins(uart0_enabled, UART_LINES_NUM);
	pr_err(CG2900_NAME "Cannot set interrupt (%d)", err);
	return err;
}


static int cg2900_enable_uart(void)
{
	int err;

	/* Restore UART settings. */
	err = nmk_config_pins(uart0_enabled, UART_LINES_NUM);
	if (err)
		pr_err(CG2900_NAME "Unable to enable UART (%d)", err);

	return err;
}

#endif /* CONFIG_MFD_CG2900_UART */

struct cg2900_platform_data cg2900_platform_data = {
	.init = cg2900_init,
	.exit = cg2900_exit,
	.enable_chip = cg2900_enable_chip,
	.disable_chip = cg2900_disable_chip,
	.set_hci_revision = cg2900_set_hci_revision,
	.get_power_switch_off_cmd = cg2900_get_power_switch_off_cmd,

#ifdef CONFIG_MFD_CG2900_UART
	.uart = {
		.cts_irq = GPIO_TO_IRQ(BT_CTS_GPIO),
		.enable_uart = cg2900_enable_uart,
		.disable_uart = cg2900_disable_uart
	},
#endif /* CONFIG_MFD_CG2900_UART */
};

struct platform_device ux500_cg2900_device = {
	.name = "cg2900",
	.dev = {
		.platform_data = &cg2900_platform_data,
	}
};
#endif /* CONFIG_MFD_CG2900 */
