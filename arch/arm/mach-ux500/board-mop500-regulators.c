/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Authors: Sundar Iyer <sundar.iyer@stericsson.com> for ST-Ericsson
 *          Bengt Jonsson <bengt.g.jonsson@stericsson.com> for ST-Ericsson
 *
 * Board specific file for regulator machine initialization
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/regulator/machine.h>

#include <mach/devices.h>

#include "board-mop500-regulators.h"

/*
 * AB8500 Regulator Configuration
 */

/* vana regulator configuration, for analogue part of displays */
#define AB8500_VANA_REGULATOR_MIN_VOLTAGE	0
#define AB8500_VANA_REGULATOR_MAX_VOLTAGE	1200000

static struct regulator_consumer_supply ab8500_vana_consumers[] = {
	{
		.dev = &ux500_mcde_device.dev,
		.supply = "v-ana",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.10",
		.supply = "ana",
	},
#endif
};

struct regulator_init_data ab8500_vana_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vana",
		.min_uV = AB8500_VANA_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VANA_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vana_consumers),
	.consumer_supplies = ab8500_vana_consumers,
};

#ifdef CONFIG_SENSORS1P_MOP
extern struct platform_device sensors1p_device;
#endif

/* vaux1 regulator configuration */
#define AB8500_VAUXN_LDO_MIN_VOLTAGE		1100000
#define AB8500_VAUXN_LDO_MAX_VOLTAGE		3300000

static struct regulator_consumer_supply ab8500_vaux1_consumers[] = {
	{
		.dev = NULL,
		.supply = "v-display",
	},
#ifdef CONFIG_SENSORS1P_MOP
	{
		.dev = &sensors1p_device.dev,
		.supply = "v-proximity",
	},
	{
		.dev = &sensors1p_device.dev,
		.supply = "v-hal",
	},
#endif
	{
		.dev = NULL,
		.supply = "v-mmio-camera",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.0",
		.supply = "aux1",
	},
#endif
	{
		.dev_name = "bu21013_ts.0",
		.supply = "v-touch",
	},

	{
		.dev_name = "bu21013_ts.1",
		.supply = "v-touch",
	},
	{
		.dev_name = "synaptics_rmi4_i2c0",
		.supply = "v-touch",
	},
};

struct regulator_init_data ab8500_vaux1_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vaux1",
		.min_uV = AB8500_VAUXN_LDO_MIN_VOLTAGE,
		.max_uV = AB8500_VAUXN_LDO_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux1_consumers),
	.consumer_supplies = ab8500_vaux1_consumers,
};

/* vaux2 regulator configuration */
static struct regulator_consumer_supply ab8500_vaux2_consumers[] = {
	{
		.dev_name = "sdi4",
		.supply = "v-eMMC",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.1",
		.supply = "aux2",
	},
#endif
};

struct regulator_init_data ab8500_vaux2_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vaux2",
		.min_uV = AB8500_VAUXN_LDO_MIN_VOLTAGE,
		.max_uV = AB8500_VAUXN_LDO_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux2_consumers),
	.consumer_supplies = ab8500_vaux2_consumers,
};

/* vaux3 regulator configuration */
static struct regulator_consumer_supply ab8500_vaux3_consumers[] = {
	{
		.dev_name = "sdi0",
		.supply = "v-MMC-SD"
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.2",
		.supply = "aux3",
	},
#endif
};

struct regulator_init_data ab8500_vaux3_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vaux3",
		.min_uV = AB8500_VAUXN_LDO_MIN_VOLTAGE,
		.max_uV = AB8500_VAUXN_LDO_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux3_consumers),
	.consumer_supplies = ab8500_vaux3_consumers,
};

/* vtvout regulator configuration, supply for tvout, gpadc, TVOUT LDO */
#define AB8500_VTVOUT_LDO_MIN_VOLTAGE		0
#define AB8500_VTVOUT_LDO_MAX_VOLTAGE		2000000

static struct regulator_consumer_supply ab8500_vtvout_consumers[] = {
	{
		.supply = "v-tvout",
	},
	{
		.supply = "ab8500-gpadc",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.4",
		.supply = "tvout",
	},
#endif
};

struct regulator_init_data ab8500_vtvout_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vtvout",
		.min_uV = AB8500_VTVOUT_LDO_MIN_VOLTAGE,
		.max_uV = AB8500_VTVOUT_LDO_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vtvout_consumers),
	.consumer_supplies = ab8500_vtvout_consumers,
};

/* vusb regulator configuration */
#define AB8500_VUSB_REGULATOR_MIN_VOLTAGE	0
#define AB8500_VUSB_REGULATOR_MAX_VOLTAGE	3300000

static struct regulator_consumer_supply ab8500_vusb_consumers[] = {
	{
		.supply = "v-bus",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.5",
		.supply = "usb",
	},
#endif
};

struct regulator_init_data ab8500_vusb_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vusb",
		.min_uV = AB8500_VUSB_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VUSB_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vusb_consumers),
	.consumer_supplies = ab8500_vusb_consumers,
};

/* vaudio regulator configuration, supply for ab8500-vaudio, VAUDIO LDO */
#define AB8500_VAUDIO_REGULATOR_MIN_VOLTAGE	1925000
#define AB8500_VAUDIO_REGULATOR_MAX_VOLTAGE	2075000

static struct regulator_consumer_supply ab8500_vaudio_consumers[] = {
	{
		.supply = "v-audio",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.6",
		.supply = "audio",
	},
#endif
};

struct regulator_init_data ab8500_vaudio_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vaudio",
		.min_uV = AB8500_VAUDIO_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VAUDIO_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vaudio_consumers),
	.consumer_supplies = ab8500_vaudio_consumers,
};

/* vamic1 regulator configuration */
#define AB8500_VAMIC1_REGULATOR_MIN_VOLTAGE	2000000
#define AB8500_VAMIC1_REGULATOR_MAX_VOLTAGE	2100000

static struct regulator_consumer_supply ab8500_vamic1_consumers[] = {
	{
		.supply = "v-amic1",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.7",
		.supply = "anamic1",
	},
#endif
};

struct regulator_init_data ab8500_vamic1_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vamic1",
		.min_uV = AB8500_VAMIC1_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VAMIC1_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vamic1_consumers),
	.consumer_supplies = ab8500_vamic1_consumers,
};

/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
static struct regulator_consumer_supply ab8500_vamic2_consumers[] = {
	{
		.supply = "v-amic2",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.8",
		.supply = "anamic2",
	},
#endif
};

struct regulator_init_data ab8500_vamic2_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vamic2",
		.min_uV = AB8500_VAMIC1_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VAMIC1_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vamic2_consumers),
	.consumer_supplies = ab8500_vamic2_consumers,
};

/* supply for v-dmic, VDMIC LDO */
#define AB8500_VDMIC_REGULATOR_MIN_VOLTAGE	1700000
#define AB8500_VDMIC_REGULATOR_MAX_VOLTAGE	1950000

static struct regulator_consumer_supply ab8500_vdmic_consumers[] = {
	{
		.supply = "v-dmic",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.9",
		.supply = "dmic",
	},
#endif
};

struct regulator_init_data ab8500_vdmic_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vdmic",
		.min_uV = AB8500_VDMIC_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VDMIC_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vdmic_consumers),
	.consumer_supplies = ab8500_vdmic_consumers,
};

/* supply for v-intcore12, VINTCORE12 LDO */
#define AB8500_VINTCORE_REGULATOR_MIN_VOLTAGE	1200000
#define AB8500_VINTCORE_REGULATOR_MAX_VOLTAGE	1350000

static struct regulator_consumer_supply ab8500_vintcore_consumers[] = {
	{
		.supply = "v-intcore",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.3",
		.supply = "intcore",
	},
#endif
};

struct regulator_init_data ab8500_vintcore_regulator = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "ab8500-vintcore",
		.min_uV = AB8500_VINTCORE_REGULATOR_MIN_VOLTAGE,
		.max_uV = AB8500_VINTCORE_REGULATOR_MAX_VOLTAGE,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(ab8500_vintcore_consumers),
	.consumer_supplies = ab8500_vintcore_consumers,
};

/*
 * Power State Regulator Configuration
 */
#define U8500_VAPE_REGULATOR_MIN_VOLTAGE	1800000
#define U8500_VAPE_REGULATOR_MAX_VOLTAGE	2000000

/* vape regulator configuration */
static struct regulator_consumer_supply u8500_vape_consumers[] = {
	{
		.supply = "v-ape",
	},
	{
		.dev_name = "nmk-i2c.0",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.1",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.2",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.3",
		.supply = "v-i2c",
	},
	{
		.dev_name = "sdi0",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi1",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi2",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi4",
		.supply = "v-mmc",
	},
	{
		.dev_name = "dma40.0",
		.supply = "v-dma",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.11",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vape_regulator = {
	.constraints = {
		.name = "u8500-vape",
		.min_uV = U8500_VAPE_REGULATOR_MIN_VOLTAGE,
		.max_uV = U8500_VAPE_REGULATOR_MAX_VOLTAGE,
		.input_uV = 1, /* notional, for set_mode* */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_DRMS |
			REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL |
			REGULATOR_MODE_IDLE,
	},
	.consumer_supplies = u8500_vape_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vape_consumers),
};

/* varm regulator_configuration */
static struct regulator_consumer_supply u8500_varm_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.12",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_varm_regulator = {
	.constraints = {
		.name = "u8500-varm",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_varm_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_varm_consumers),
};

/* vmodem regulator configuration */
static struct regulator_consumer_supply u8500_vmodem_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.13",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vmodem_regulator = {
	.constraints = {
		.name = "u8500-vmodem",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vmodem_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vmodem_consumers),
};

/* vpll regulator configuration */
static struct regulator_consumer_supply u8500_vpll_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.14",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vpll_regulator = {
	.constraints = {
		.name = "u8500-vpll",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vpll_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vpll_consumers),
};

/* vsmps1 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps1_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.15",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vsmps1_regulator = {
	.constraints = {
		.name = "u8500-vsmps1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vsmps1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps1_consumers),
};

/* vsmsp2 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps2_consumers[] = {
	{
		.dev_name = "cg2900-uart.0",
		.supply = "gbf_1v8",
	},
	{
		.dev_name = "cw1200",
		.supply = "wlan_1v8",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.16",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vsmps2_regulator = {
	.constraints = {
		.name = "u8500-vsmps2",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vsmps2_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps2_consumers),
};

/* vsmps3 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps3_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.17",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vsmps3_regulator = {
	.constraints = {
		.name = "u8500-vsmps3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vsmps3_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps3_consumers),
};

/* vrf1 regulator configuration */
static struct regulator_consumer_supply u8500_vrf1_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.18",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_vrf1_regulator = {
	.constraints = {
		.name = "u8500-vrf1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vrf1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vrf1_consumers),
};

/*
 * Power Domain Switch Configuration
 */

/* SVA MMDSP regulator switch */
static struct regulator_consumer_supply u8500_svammdsp_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-mmdsp",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.19",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_svammdsp_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sva-mmdsp",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svammdsp_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svammdsp_consumers),
};

/* SVA MMDSP retention regulator switch */
static struct regulator_consumer_supply u8500_svammdspret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-mmdsp-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.20",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_svammdspret_regulator = {
	.constraints = {
		.name = "u8500-sva-mmdsp-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svammdspret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svammdspret_consumers),
};

/* SVA pipe regulator switch */
static struct regulator_consumer_supply u8500_svapipe_consumers[] = {
	/* Add SVA pipe device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-pipe",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.21",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_svapipe_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sva-pipe",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svapipe_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svapipe_consumers),
};

/* SIA MMDSP regulator switch */
static struct regulator_consumer_supply u8500_siammdsp_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-mmdsp",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.22",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_siammdsp_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sia-mmdsp",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siammdsp_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siammdsp_consumers),
};

/* SIA MMDSP retention regulator switch */
static struct regulator_consumer_supply u8500_siammdspret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-mmdsp-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.23",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_siammdspret_regulator = {
	.constraints = {
		.name = "u8500-sia-mmdsp-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siammdspret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siammdspret_consumers),
};

/* SIA pipe regulator switch */
static struct regulator_consumer_supply u8500_siapipe_consumers[] = {
	/* Add SIA pipe device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-pipe",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.24",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_siapipe_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sia-pipe",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siapipe_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siapipe_consumers),
};

/* SGA regulator switch */
static struct regulator_consumer_supply u8500_sga_consumers[] = {
	/* Add SGA device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sga",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.25",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_sga_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sga",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_sga_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_sga_consumers),
};

/* B2R2-MCDE regulator switch */
static struct regulator_consumer_supply u8500_b2r2_mcde_consumers[] = {
	{
		.dev_name = "U8500-B2R2",
		.supply = "vsupply",
	},
	{
		.dev_name = "mcde",
		.supply = "vsupply",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-b2r2",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-mcde",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.26",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_b2r2_mcde_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-b2r2-mcde",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_b2r2_mcde_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_b2r2_mcde_consumers),
};

/* ESRAM1 and 2 regulator switch */
static struct regulator_consumer_supply u8500_esram12_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram1",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram2",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.27",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_esram12_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-esram12",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram12_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram12_consumers),
};

/* ESRAM1 and 2 retention regulator switch */
static struct regulator_consumer_supply u8500_esram12ret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram1-ret",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram2-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.28",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_esram12ret_regulator = {
	.constraints = {
		.name = "u8500-esram12-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram12ret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram12ret_consumers),
};

/* ESRAM3 and 4 regulator switch */
static struct regulator_consumer_supply u8500_esram34_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram3",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram4",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.29",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_esram34_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-esram34",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram34_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram34_consumers),
};

/* ESRAM3 and 4 retention regulator switch */
static struct regulator_consumer_supply u8500_esram34ret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram3-ret",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram4-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.30",
		.supply = "test",
	},
#endif
};

struct regulator_init_data u8500_esram34ret_regulator = {
	.constraints = {
		.name = "u8500-esram34-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram34ret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram34ret_consumers),
};
