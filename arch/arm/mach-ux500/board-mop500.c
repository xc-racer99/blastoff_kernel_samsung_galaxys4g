/*
 * Copyright (C) 2008-2009 ST-Ericsson
 *
 * Author: Srinidhi KASAGAR <srinidhi.kasagar@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/hsi.h>
#include <linux/i2s/i2s.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mfd/stmpe.h>
#include <linux/mfd/tc35892.h>
#include <linux/i2c/lp5521.h>
#include <linux/power_supply.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/input/bu21013.h>
#include <linux/spi/stm_msp.h>
#include <linux/leds_pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/mfd/ab8500/ab8500-bm.h>
#include <linux/mfd/ab8500/denc.h>
#include <linux/ste_timed_vibra.h>
#include <linux/mfd/cg2900.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>

#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/i2c.h>

#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/devices.h>
#include <mach/irqs.h>

#include "devices-cg2900.h"
#include "devices-db8500.h"
#include "board-mop500-regulators.h"
#include "regulator-u8500.h"
#include "pins-db8500.h"
#include "board-mop500.h"

#define IRQ_KP 1 /*To DO*/

/*
 * This variable holds the number of touchscreen (bu21013) enabled.
 */
static int cs_en_check;

static pin_cfg_t mop500_pins[] = {
	/* I2C */
	GPIO147_I2C0_SCL,
	GPIO148_I2C0_SDA,
	GPIO16_I2C1_SCL,
	GPIO17_I2C1_SDA,
	GPIO10_I2C2_SDA,
	GPIO11_I2C2_SCL,
	GPIO229_I2C3_SDA,
	GPIO230_I2C3_SCL,

	/* SSP0 */
	GPIO143_SSP0_CLK,
	GPIO144_SSP0_FRM,
	GPIO145_SSP0_RXD | PIN_PULL_DOWN,
	GPIO146_SSP0_TXD,

	/* MSP0 */
	GPIO12_MSP0_TXD,
	GPIO13_MSP0_TFS,
	GPIO14_MSP0_TCK,
	GPIO15_MSP0_RXD,

	/* MSP2 */
	GPIO193_MSP2_TXD,
	GPIO194_MSP2_TCK,
	GPIO195_MSP2_TFS,
	GPIO196_MSP2_RXD | PIN_OUTPUT_LOW,

	GPIO84_GPIO	| PIN_INPUT_PULLUP, /* Touchscreen IRQ */
	GPIO217_GPIO	| PIN_INPUT_PULLUP, /* TC35892 IRQ */
	GPIO218_GPIO	| PIN_INPUT_PULLUP, /* STMPE1601 IRQ */

	/* SDI0 (MicroSD card) */
	GPIO18_MC0_CMDDIR	| PIN_OUTPUT_HIGH,
	GPIO19_MC0_DAT0DIR	| PIN_OUTPUT_HIGH,
	GPIO20_MC0_DAT2DIR	| PIN_OUTPUT_HIGH,
	GPIO21_MC0_DAT31DIR	| PIN_OUTPUT_HIGH,
	GPIO22_MC0_FBCLK	| PIN_INPUT_NOPULL,
	GPIO23_MC0_CLK		| PIN_OUTPUT_LOW,
	GPIO24_MC0_CMD		| PIN_INPUT_PULLUP,
	GPIO25_MC0_DAT0		| PIN_INPUT_PULLUP,
	GPIO26_MC0_DAT1		| PIN_INPUT_PULLUP,
	GPIO27_MC0_DAT2		| PIN_INPUT_PULLUP,
	GPIO28_MC0_DAT3		| PIN_INPUT_PULLUP,

	/* SDI1 (SDIO) */
	GPIO208_MC1_CLK		| PIN_OUTPUT_LOW,
	GPIO209_MC1_FBCLK	| PIN_INPUT_NOPULL,
	GPIO210_MC1_CMD		| PIN_INPUT_PULLUP,
	GPIO211_MC1_DAT0	| PIN_INPUT_PULLUP,
	GPIO212_MC1_DAT1	| PIN_INPUT_PULLUP,
	GPIO213_MC1_DAT2	| PIN_INPUT_PULLUP,
	GPIO214_MC1_DAT3	| PIN_INPUT_PULLUP,

	/* SDI2 (POP eMMC) */
	GPIO128_MC2_CLK		| PIN_OUTPUT_LOW,
	GPIO129_MC2_CMD		| PIN_INPUT_PULLUP,
	GPIO130_MC2_FBCLK	| PIN_INPUT_NOPULL,
	GPIO131_MC2_DAT0	| PIN_INPUT_PULLUP,
	GPIO132_MC2_DAT1	| PIN_INPUT_PULLUP,
	GPIO133_MC2_DAT2	| PIN_INPUT_PULLUP,
	GPIO134_MC2_DAT3	| PIN_INPUT_PULLUP,
	GPIO135_MC2_DAT4	| PIN_INPUT_PULLUP,
	GPIO136_MC2_DAT5	| PIN_INPUT_PULLUP,
	GPIO137_MC2_DAT6	| PIN_INPUT_PULLUP,
	GPIO138_MC2_DAT7	| PIN_INPUT_PULLUP,

	/* SDI4 (On-board eMMC) */
	GPIO197_MC4_DAT3	| PIN_INPUT_PULLUP,
	GPIO198_MC4_DAT2	| PIN_INPUT_PULLUP,
	GPIO199_MC4_DAT1	| PIN_INPUT_PULLUP,
	GPIO200_MC4_DAT0	| PIN_INPUT_PULLUP,
	GPIO201_MC4_CMD		| PIN_INPUT_PULLUP,
	GPIO202_MC4_FBCLK	| PIN_INPUT_NOPULL,
	GPIO203_MC4_CLK		| PIN_OUTPUT_LOW,
	GPIO204_MC4_DAT7	| PIN_INPUT_PULLUP,
	GPIO205_MC4_DAT6	| PIN_INPUT_PULLUP,
	GPIO206_MC4_DAT5	| PIN_INPUT_PULLUP,
	GPIO207_MC4_DAT4	| PIN_INPUT_PULLUP,

static void ab4500_spi_cs_control(u32 command)
{
	/* set the FRM signal, which is CS  - TODO */
}

struct pl022_config_chip ab4500_chip_info = {
	.lbm = LOOPBACK_DISABLED,
	.com_mode = INTERRUPT_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.endian_rx = SSP_RX_MSB,
	.endian_tx = SSP_TX_MSB,
	.data_size = SSP_DATA_BITS_24,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.clk_phase = SSP_CLK_SECOND_EDGE,
	.clk_pol = SSP_CLK_POL_IDLE_HIGH,
	.cs_control = ab4500_spi_cs_control,
};

static struct spi_board_info u8500_spi_devices[] = {
	{
		.modalias = "ab8500",
		.controller_data = &ab4500_chip_info,
		.max_speed_hz = 12000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.irq = IRQ_AB4500,
	},
};

static struct pl022_ssp_controller ssp0_platform_data = {
	.bus_id = 0,
	/* pl022 not yet supports dma */
	.enable_dma = 0,
	/* on this platform, gpio 31,142,144,214 &
	 * 224 are connected as chip selects
	 */
	.num_chipselect = 5,
};

#define U8500_I2C_CONTROLLER(id, _slsu, _tft, _rft, clk, _sm) \
static struct nmk_i2c_controller u8500_i2c##id##_data = { \
	/*				\
	 * slave data setup time, which is	\
	 * 250 ns,100ns,10ns which is 14,6,2	\
	 * respectively for a 48 Mhz	\
	 * i2c clock			\
	 */				\
	.slsu		= _slsu,	\
	/* Tx FIFO threshold */		\
	.tft		= _tft,		\
	/* Rx FIFO threshold */		\
	.rft		= _rft,		\
	/* std. mode operation */	\
	.clk_freq	= clk,		\
	.sm		= _sm,		\
}

/*
 * The board uses 4 i2c controllers, initialize all of
 * them with slave data setup time of 250 ns,
 * Tx & Rx FIFO threshold values as 1 and standard
 * mode of operation
 */

U8500_I2C_CONTROLLER(0, 0xe, 1, 1, 400000, 500, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(1, 0xe, 1, 1, 400000, 500, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(2, 0xe, 1, 1, 400000, 500, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(3, 0xe, 1, 1, 400000, 500, I2C_FREQ_MODE_FAST);

static struct hsi_board_info __initdata stm_hsi_devices[] = {
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 0,
	 .chan_num = 0, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 1,
	 .chan_num = 0, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 0,
	 .chan_num = 1, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 1,
	 .chan_num = 1, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 0,
	 .chan_num = 2, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 1,
	 .chan_num = 2, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 0,
	 .chan_num = 3, .mode = 1},
	{.type = "HSI_LOOPBACK", .flags = 0, .controller_id = 1,
	 .chan_num = 3, .mode = 1},
};

#ifdef AB8500_BAT_INTERNAL_NTC
/*
 * These are the defined batteries that uses a NTC and ID resistor placed
 * inside of the battery pack.
 * Note that the res_to_temp table must be strictly sorted by falling resistance
 * values to work.
 */
static struct res_to_temp temp_tbl_type_A[] = {
	{-5, 53407},
	{ 0, 48594},
	{ 5, 43804},
	{10, 39188},
	{15, 34870},
	{20, 30933},
	{25, 27422},
	{30, 24347},
	{35, 21694},
	{40, 19431},
	{45, 17517},
	{50, 15908},
	{55, 14561},
	{60, 13437},
	{65, 12500},
};
static struct res_to_temp temp_tbl_type_B[] = {
	{-5, 165418},
	{ 0, 159024},
	{ 5, 151921},
	{10, 144300},
	{15, 136424},
	{20, 128565},
	{25, 120978},
	{30, 113875},
	{35, 107397},
	{40, 101629},
	{45,  96592},
	{50,  92253},
	{55,  88569},
	{60,  85461},
	{65,  82869},
};
static struct v_to_cap cap_tbl_type_A[] = {
	{4171,	100},
	{4114,	 95},
	{4009,	 83},
	{3947,	 74},
	{3907,	 67},
	{3863,	 59},
	{3830,	 56},
	{3813,	 53},
	{3791,	 46},
	{3771,	 33},
	{3754,	 25},
	{3735,	 20},
	{3717,	 17},
	{3681,	 13},
	{3664,	  8},
	{3651,	  6},
	{3635,	  5},
	{3560,	  3},
	{3408,    1},
	{3247,	  0},
};
static struct v_to_cap cap_tbl_type_B[] = {
	{4161,	100},
	{4124,	 98},
	{4044,	 90},
	{4003,	 85},
	{3966,	 80},
	{3933,	 75},
	{3888,	 67},
	{3849,	 60},
	{3813,	 55},
	{3787,	 47},
	{3772,	 30},
	{3751,	 25},
	{3718,	 20},
	{3681,	 16},
	{3660,	 14},
	{3589,	 10},
	{3546,	  7},
	{3495,	  4},
	{3404,	  2},
	{3250,	  0},
};
#endif
static struct v_to_cap cap_tbl_type_N[] = {
	{4203,	100},
	{4128,	 99},
	{4095,	 98},
	{4044,	 95},
	{3990,	 90},
	{3908,	 80},
	{3842,	 70},
	{3789,	 60},
	{3746,	 50},
	{3713,	 40},
	{3685,	 30},
	{3646,	 20},
	{3628,	 17},
	{3605,	 14},
	{3559,	 10},
	{3501,	  7},
	{3400,	  4},
	{3275,	  2},
	{3098,	  0},
};

/*
 * Note that the pcb_ntc table must be strictly sorted by falling
 * resistance values to work.
 */
static struct res_to_temp table_pcb_ntc[] = {
	{-5, 214834},
	{ 0, 162943},
	{ 5, 124820},
	{10,  96520},
	{15,  75306},
	{20,  59254},
	{25,  47000},
	{30,  37566},
	{35,  30245},
	{40,  24520},
	{45,  20010},
	{50,  16432},
	{55,  13576},
	{60,  11280},
	{65,   9425},
};

static struct battery_type bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.battery_resistance = 300,
		.charge_full_design = 612,
		.termination_vol = 4100,
		.termination_curr = 200,
		.normal_op_cur_lvl = 400,
		.normal_ip_vol_lvl = 4100,
		.maint_a_op_cur_lvl = 400,
		.maint_a_ip_vol_lvl = 4050,
		.maint_a_chg_timer_h = 60,
		.maint_b_op_cur_lvl = 400,
		.maint_b_ip_vol_lvl = 4000,
		.maint_b_chg_timer_h = 200,
		.low_high_op_cur_lvl = 300,
		.low_high_ip_vol_lvl = 4000,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type_N),
		.v_to_cap_tbl = cap_tbl_type_N,
	},

#ifdef AB8500_BAT_INTERNAL_NTC
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 53407,
		.resis_low = 12500,
		.battery_resistance = 300,
		.charge_full_design = 900,
		.termination_vol = 4200,
		.termination_curr = 80,
		.normal_op_cur_lvl = 700,
		.normal_ip_vol_lvl = 4200,
		.maint_a_op_cur_lvl = 600,
		.maint_a_ip_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_op_cur_lvl = 600,
		.maint_b_ip_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_op_cur_lvl = 300,
		.low_high_ip_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_type_A),
		.r_to_t_tbl = temp_tbl_type_A,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type_A),
		.v_to_cap_tbl = cap_tbl_type_A,

	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 165418,
		.resis_low = 82869,
		.battery_resistance = 300,
		.charge_full_design = 900,
		.termination_vol = 4200,
		.termination_curr = 80,
		.normal_op_cur_lvl = 700,
		.normal_ip_vol_lvl = 4200,
		.maint_a_op_cur_lvl = 600,
		.maint_a_ip_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_op_cur_lvl = 600,
		.maint_b_ip_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_op_cur_lvl = 300,
		.low_high_ip_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_type_B),
		.r_to_t_tbl = temp_tbl_type_B,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type_B),
		.v_to_cap_tbl = cap_tbl_type_B,
	},
#else
/*
 * These are the batteries that doesn't have an internal NTC resistor to measure
 * its temperature. The temperature in this case is measure with a NTC placed
 * near the battery but on the PCB.
 */
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 76000,
		.resis_low = 53000,
		.battery_resistance = 300,
		.charge_full_design = 900,
		.termination_vol = 4200,
		.termination_curr = 100,
		.normal_op_cur_lvl = 700,
		.normal_ip_vol_lvl = 4200,
		.maint_a_op_cur_lvl = 600,
		.maint_a_ip_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_op_cur_lvl = 600,
		.maint_b_ip_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_op_cur_lvl = 300,
		.low_high_ip_vol_lvl = 4000,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type_N),
		.v_to_cap_tbl = cap_tbl_type_N,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 95000,
		.resis_low = 76001,
		.battery_resistance = 300,
		.charge_full_design = 950,
		.termination_vol = 4200,
		.termination_curr = 100,
		.normal_op_cur_lvl = 700,
		.normal_ip_vol_lvl = 4200,
		.maint_a_op_cur_lvl = 600,
		.maint_a_ip_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_op_cur_lvl = 600,
		.maint_b_ip_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_op_cur_lvl = 300,
		.low_high_ip_vol_lvl = 4000,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type_N),
		.v_to_cap_tbl = cap_tbl_type_N,
	}
#endif
};

static char *ab8500_charger_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_fg",
	"ab8500_btemp",
};

static char *ab8500_btemp_supplied_to[] = {
	"ab8500_chargalg",
};

static char *ab8500_fg_supplied_to[] = {
	"ab8500_chargalg",
};

static char *ab8500_chargalg_supplied_to[] = {
	"ab8500_fg",
};

static struct ab8500_charger_platform_data ab8500_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
};

static struct ab8500_btemp_platform_data ab8500_btemp_plat_data = {
	.supplied_to = ab8500_btemp_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_btemp_supplied_to),
};

static struct ab8500_fg_platform_data ab8500_fg_plat_data = {
	.supplied_to = ab8500_fg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_fg_supplied_to),
};

static struct ab8500_chargalg_platform_data ab8500_chargalg_plat_data = {
	.supplied_to = ab8500_chargalg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_chargalg_supplied_to),
};

static struct ab8500_bm_capacity_levels cap_levels = {
	.critical	= 2,
	.low		= 10,
	.normal		= 70,
	.high		= 95,
	.full		= 100,
};

static const struct ab8500_fg_parameters fg = {
		.recovery_sleep_timer = 10,
		.recovery_total_time = 100,
		.init_timer = 2,
		.init_discard_time = 6,
		.init_total_time = 120,
		.high_curr_time = 60,
		.accu_charging = 30,
		.accu_high_curr = 30,
		.high_curr_threshold = 50,
		.lowbat_threshold = 3100,
};

static struct ab8500_bm_data ab8500_bm_data = {
	.temp_under		= 3,
	.temp_low		= 8,
	.temp_high		= 55,
	.temp_over		= 60,
	.main_safety_tmr_h	= 4,
	.usb_safety_tmr_h	= 4,
	.bkup_bat_v		= BUP_VCH_SEL_2P6V,
	.bkup_bat_i		= BUP_ICH_SEL_150UA,
#ifdef AB8500_BAT_INTERNAL_NTC
	.therm			= THERM_BATTERY_PACK,
#else
	.therm			= THERM_PCB,
#endif
	.fg_res			= 10,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 0,
	.pcb_ntc		= table_pcb_ntc,
	.n_temp_tbl_elements	= ARRAY_SIZE(table_pcb_ntc),
	.fg_params		= &fg,
};

#ifdef CONFIG_AB8500_DENC
static struct ab8500_denc_platform_data ab8500_denc_pdata = {
	.ddr_enable = true,
	.ddr_little_endian = false,
};
#endif

static struct regulator_init_data *u8500_regulators[U8500_NUM_REGULATORS] = {
	[U8500_REGULATOR_VAPE]			= &u8500_vape_regulator,
	[U8500_REGULATOR_VARM]			= &u8500_varm_regulator,
	[U8500_REGULATOR_VMODEM]		= &u8500_vmodem_regulator,
	[U8500_REGULATOR_VPLL]			= &u8500_vpll_regulator,
	[U8500_REGULATOR_VSMPS1]		= &u8500_vsmps1_regulator,
	[U8500_REGULATOR_VSMPS2]		= &u8500_vsmps2_regulator,
	[U8500_REGULATOR_VSMPS3]		= &u8500_vsmps3_regulator,
	[U8500_REGULATOR_VRF1]			= &u8500_vrf1_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSP]	= &u8500_svammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSPRET]	= &u8500_svammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SVAPIPE]	= &u8500_svapipe_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSP]	= &u8500_siammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSPRET]	= &u8500_siammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SIAPIPE]	= &u8500_siapipe_regulator,
	[U8500_REGULATOR_SWITCH_SGA]		= &u8500_sga_regulator,
	[U8500_REGULATOR_SWITCH_B2R2_MCDE]	= &u8500_b2r2_mcde_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12]	= &u8500_esram12_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12RET]	= &u8500_esram12ret_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34]	= &u8500_esram34_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34RET]	= &u8500_esram34ret_regulator,
};

static struct platform_device u8500_regulator_dev = {
	.name = "u8500-regulators",
	.id   = 0,
	.dev  = {
		.platform_data = u8500_regulators,
	},
};

static struct ab8500_audio_platform_data ab8500_audio_plat_data = {
	.ste_gpio_altf_init = msp13_i2s_init,
	.ste_gpio_altf_exit = msp13_i2s_exit,
};

/*
 * NOTE! The regulator configuration below must be in exactly the same order as
 * the regulator description in the driver, see drivers/regulator/ab8500.c
 */
static struct ab8500_platform_data ab8500_platdata = {
	.irq_base = MOP500_AB8500_IRQ_BASE,
#ifdef CONFIG_REGULATOR
	.regulator = {
		&ab8500_vaux1_regulator,
		&ab8500_vaux2_regulator,
		&ab8500_vaux3_regulator,
		&ab8500_vintcore_regulator,
		&ab8500_vtvout_regulator,
		&ab8500_vusb_regulator,
		&ab8500_vaudio_regulator,
		&ab8500_vamic1_regulator,
		&ab8500_vamic2_regulator,
		&ab8500_vdmic_regulator,
		&ab8500_vana_regulator,
	},
#endif
#ifdef CONFIG_AB8500_DENC
	.denc = &ab8500_denc_pdata,
#endif
	.audio = &ab8500_audio_plat_data,
	.battery = &ab8500_bm_data,
	.charger = &ab8500_charger_plat_data,
	.btemp = &ab8500_btemp_plat_data,
	.fg = &ab8500_fg_plat_data,
	.chargalg = &ab8500_chargalg_plat_data,
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start = IRQ_DB8500_AB8500,
		.end = IRQ_DB8500_AB8500,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device ux500_ab8500_device = {
	.name = "ab8500-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8500_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

#ifdef CONFIG_MFD_CG2900
#define CG2900_BT_ENABLE_GPIO		170
#define CG2900_GBF_ENA_RESET_GPIO	171
#define CG2900_BT_CTS_GPIO		0

enum cg2900_gpio_pull_sleep cg2900_sleep_gpio[21] = {
	CG2900_NO_PULL,		/* GPIO 0:  PTA_CONFX */
	CG2900_PULL_DN,		/* GPIO 1:  PTA_STATUS */
	CG2900_NO_PULL,		/* GPIO 2:  UART_CTSN */
	CG2900_PULL_UP,		/* GPIO 3:  UART_RTSN */
	CG2900_PULL_UP,		/* GPIO 4:  UART_TXD */
	CG2900_NO_PULL,		/* GPIO 5:  UART_RXD */
	CG2900_PULL_DN,		/* GPIO 6:  IOM_DOUT */
	CG2900_NO_PULL,		/* GPIO 7:  IOM_FSC */
	CG2900_NO_PULL,		/* GPIO 8:  IOM_CLK */
	CG2900_NO_PULL,		/* GPIO 9:  IOM_DIN */
	CG2900_PULL_DN,		/* GPIO 10: PWR_REQ */
	CG2900_PULL_DN,		/* GPIO 11: HOST_WAKEUP */
	CG2900_PULL_DN,		/* GPIO 12: IIS_DOUT */
	CG2900_NO_PULL,		/* GPIO 13: IIS_WS */
	CG2900_NO_PULL,		/* GPIO 14: IIS_CLK */
	CG2900_NO_PULL,		/* GPIO 15: IIS_DIN */
	CG2900_PULL_DN,		/* GPIO 16: PTA_FREQ */
	CG2900_PULL_DN,		/* GPIO 17: PTA_RF_ACTIVE */
	CG2900_NO_PULL,		/* GPIO 18: NotConnected (J6428) */
	CG2900_NO_PULL,		/* GPIO 19: EXT_DUTY_CYCLE */
	CG2900_NO_PULL,		/* GPIO 20: EXT_FRM_SYNCH */
};

static struct platform_device ux500_cg2900_device = {
	.name = "cg2900",
};

#ifdef CONFIG_MFD_CG2900_CHIP
static struct platform_device ux500_cg2900_chip_device = {
	.name = "cg2900-chip",
	.dev = {
		.parent = &ux500_cg2900_device.dev,
	},
};
#endif /* CONFIG_MFD_CG2900_CHIP */

#ifdef CONFIG_MFD_STLC2690_CHIP
static struct platform_device ux500_stlc2690_chip_device = {
	.name = "stlc2690-chip",
	.dev = {
		.parent = &ux500_cg2900_device.dev,
	},
};
#endif /* CONFIG_MFD_STLC2690_CHIP */

#ifdef CONFIG_MFD_CG2900_TEST
static struct cg2900_platform_data cg2900_test_platform_data = {
	.bus = HCI_VIRTUAL,
	.gpio_sleep = cg2900_sleep_gpio,
};

static struct platform_device ux500_cg2900_test_device = {
	.name = "cg2900-test",
	.dev = {
		.parent = &ux500_cg2900_device.dev,
		.platform_data = &cg2900_test_platform_data,
	},
};
#endif /* CONFIG_MFD_CG2900_TEST */

#ifdef CONFIG_MFD_CG2900_UART
static struct resource cg2900_uart_resources[] = {
	{
		.start = CG2900_GBF_ENA_RESET_GPIO,
		.end = CG2900_GBF_ENA_RESET_GPIO,
		.flags = IORESOURCE_IO,
		.name = "gbf_ena_reset",
	},
	{
		.start = CG2900_BT_ENABLE_GPIO,
		.end = CG2900_BT_ENABLE_GPIO,
		.flags = IORESOURCE_IO,
		.name = "bt_enable",
	},
	{
		.start = GPIO_TO_IRQ(CG2900_BT_CTS_GPIO),
		.end = GPIO_TO_IRQ(CG2900_BT_CTS_GPIO),
		.flags = IORESOURCE_IRQ,
		.name = "cts_irq",
	},
};

static pin_cfg_t cg2900_uart_enabled[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH
};

static pin_cfg_t cg2900_uart_disabled[] = {
	GPIO0_GPIO   | PIN_INPUT_PULLUP,	/* CTS pull up. */
	GPIO1_GPIO   | PIN_OUTPUT_HIGH,		/* RTS high-flow off. */
	GPIO2_GPIO   | PIN_INPUT_PULLUP,	/* RX pull down. */
	GPIO3_GPIO   | PIN_OUTPUT_LOW		/* TX low - break on. */
};

static struct cg2900_platform_data cg2900_uart_platform_data = {
	.bus = HCI_UART,
	.gpio_sleep = cg2900_sleep_gpio,
	.uart = {
		.n_uart_gpios = 4,
		.uart_enabled = cg2900_uart_enabled,
		.uart_disabled = cg2900_uart_disabled,
	},
};

static struct platform_device ux500_cg2900_uart_device = {
	.name = "cg2900-uart",
	.dev = {
		.platform_data = &cg2900_uart_platform_data,
		.parent = &ux500_cg2900_device.dev,
	},
	.num_resources = ARRAY_SIZE(cg2900_uart_resources),
	.resource = cg2900_uart_resources,
};
#endif /* CONFIG_MFD_CG2900_UART */
#endif /* CONFIG_MFD_CG2900 */

#ifdef CONFIG_LEDS_PWM
static struct led_pwm pwm_leds_data[] = {
	[0] = {
		.name = "lcd-backlight",
		.pwm_id = 1,
		.max_brightness = 255,
		.lth_brightness = 90,
		.pwm_period_ns = 1023,
	},
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	[1] = {
		.name = "sec-lcd-backlight",
		.pwm_id = 2,
		.max_brightness = 255,
		.lth_brightness = 90,
		.pwm_period_ns = 1023,
	},
#endif
};

static struct led_pwm_platform_data u8500_leds_data = {
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	.num_leds = 2,
#else
	.num_leds = 1,
#endif
	.leds = &pwm_leds_data,
};

static struct platform_device ux500_leds_device = {
	.name = "leds_pwm",
	.dev = {
		.platform_data = &u8500_leds_data,
	},
};
#endif

#ifdef CONFIG_BACKLIGHT_PWM
static struct platform_pwm_backlight_data u8500_backlight_data[] = {
	[0] = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 200,
	.lth_brightness = 90,
	.pwm_period_ns = 1023,
	},
	[1] = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 200,
	.lth_brightness = 90,
	.pwm_period_ns = 1023,
	},
};

static struct amba_device *amba_devs[] __initdata = {
	&ux500_uart0_device,
	&ux500_uart1_device,
	&ux500_uart2_device,
	&u8500_ssp0_device,
};

/* add any platform devices here - TODO */
static struct platform_device *platform_devs[] __initdata = {
	&u8500_hsit_device,
	&u8500_hsir_device,
	&u8500_shrm_device,
	&ste_ff_vibra_device,
	&ux500_musb_device,
	&ux500_hwmem_device,
	&ux500_mcde_device,
	&ux500_b2r2_device,
#ifdef CONFIG_CRYPTO_DEV_UX500_HASH
	&ux500_hash1_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&usb_mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	&usb_ecm_device,
#endif
#endif
};

static void __init u8500_init_machine(void)
{
	int i;

	u8500_i2c0_device.dev.platform_data = &u8500_i2c0_data;
	ux500_i2c1_device.dev.platform_data = &u8500_i2c1_data;
	ux500_i2c2_device.dev.platform_data = &u8500_i2c2_data;
	ux500_i2c3_device.dev.platform_data = &u8500_i2c3_data;

	u8500_ssp0_device.dev.platform_data = &ssp0_platform_data;

	/* Register the active AMBA devices on this board */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

	spi_register_board_info(u8500_spi_devices,
			ARRAY_SIZE(u8500_spi_devices));

	u8500_init_devices();

        platform_add_devices(platform_devs,
                        ARRAY_SIZE(platform_devs));

	nmk_config_pins(mop500_pins, ARRAY_SIZE(mop500_pins));

	platform_device_register(&ux500_ab8500_device);

#ifdef CONFIG_MFD_CG2900
#ifdef CONFIG_MFD_CG2900_TEST
	dcg2900_init_platdata(&cg2900_test_platform_data);
#endif /* CONFIG_MFD_CG2900_TEST */
#ifdef CONFIG_MFD_CG2900_UART
	dcg2900_init_platdata(&cg2900_uart_platform_data);
#endif /* CONFIG_MFD_CG2900_UART */

	platform_device_register(&ux500_cg2900_device);
#ifdef CONFIG_MFD_CG2900_UART
	platform_device_register(&ux500_cg2900_uart_device);
#endif /* CONFIG_MFD_CG2900_UART */
#ifdef CONFIG_MFD_CG2900_TEST
	platform_device_register(&ux500_cg2900_test_device);
#endif /* CONFIG_MFD_CG2900_TEST */
#ifdef CONFIG_MFD_CG2900_CHIP
	platform_device_register(&ux500_cg2900_chip_device);
#endif /* CONFIG_MFD_CG2900_CHIP */
#ifdef CONFIG_MFD_STLC2690_CHIP
	platform_device_register(&ux500_stlc2690_chip_device);
#endif /* CONFIG_MFD_STLC2690_CHIP */
#endif /* CONFIG_MFD_CG2900 */

	mop500_i2c_init();
	mop500_msp_init();
	mop500_spi_init();
	mop500_uart_init();

	db8500_add_ske_keypad(&mop500_ske_keypad_data);

	hsi_register_board_info(stm_hsi_devices, ARRAY_SIZE(stm_hsi_devices));
#ifdef CONFIG_U8500_UIB
	u8500_uib_init();
#endif
	platform_add_devices(u8500_platform_devices,
			     ARRAY_SIZE(u8500_platform_devices));
}

MACHINE_START(U8500, "ST-Ericsson MOP500 platform")
	/* Maintainer: Srinidhi Kasagar <srinidhi.kasagar@stericsson.com> */
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	/* we re-use nomadik timer here */
	.timer		= &ux500_timer,
	.init_machine	= u8500_init_machine,
MACHINE_END
