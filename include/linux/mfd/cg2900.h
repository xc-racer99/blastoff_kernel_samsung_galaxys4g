/*
 * include/linux/mfd/cg2900.h
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
 * Linux Bluetooth HCI H:4 Driver for ST-Ericsson CG2900 connectivity
 * controller.
 */

#ifndef _CG2900_H_
#define _CG2900_H_

#include <linux/skbuff.h>

#define CG2900_MAX_NAME_SIZE 30

/*
 * Channel names to use when registering to CG2900 driver
 */

/** CG2900_BT_CMD - Bluetooth HCI H4 channel for Bluetooth commands.
 */
#define CG2900_BT_CMD		"cg2900_bt_cmd"

/** CG2900_BT_ACL - Bluetooth HCI H4 channel for Bluetooth ACL data.
 */
#define CG2900_BT_ACL		"cg2900_bt_acl"

/** CG2900_BT_EVT - Bluetooth HCI H4 channel for Bluetooth events.
 */
#define CG2900_BT_EVT		"cg2900_bt_evt"

/** CG2900_FM_RADIO - Bluetooth HCI H4 channel for FM radio.
 */
#define CG2900_FM_RADIO		"cg2900_fm_radio"

/** CG2900_GNSS - Bluetooth HCI H4 channel for GNSS.
 */
#define CG2900_GNSS		"cg2900_gnss"

/** CG2900_DEBUG - Bluetooth HCI H4 channel for internal debug data.
 */
#define CG2900_DEBUG		"cg2900_debug"

/** CG2900_STE_TOOLS - Bluetooth HCI H4 channel for development tools data.
 */
#define CG2900_STE_TOOLS	"cg2900_ste_tools"

/** CG2900_HCI_LOGGER - BT channel for logging all transmitted H4 packets.
 * Data read is copy of all data transferred on the other channels.
 * Only write allowed is configuration of the HCI Logger.
 */
#define CG2900_HCI_LOGGER	"cg2900_hci_logger"

/** CG2900_US_CTRL - Channel for user space init and control of CG2900.
 */
#define CG2900_US_CTRL		"cg2900_us_ctrl"

/** CG2900_BT_AUDIO - HCI Channel for BT audio configuration commands.
 * Maps to Bluetooth command and event channels.
 */
#define CG2900_BT_AUDIO		"cg2900_bt_audio"

/** CG2900_FM_RADIO_AUDIO - HCI channel for FM audio configuration commands.
 * Maps to FM Radio channel.
 */
#define CG2900_FM_RADIO_AUDIO	"cg2900_fm_audio"

/** CG2900_CORE- Channel for keeping ST-Ericsson CG2900 enabled.
 * Opening this channel forces the chip to stay powered.
 * No data can be written to or read from this channel.
 */
#define CG2900_CORE		"cg2900_core"

struct cg2900_callbacks;

/**
 * struct cg2900_device - Device structure for CG2900 user.
 * @h4_channel:		HCI H:4 channel used by this device.
 * @cb:			Callback functions registered by this device.
 * @logger_enabled:	true if HCI logger is enabled for this channel,
 *			false otherwise.
 * @user_data:		Arbitrary data used by caller.
 * @dev:		Parent device this driver is connected to.
 *
 * Defines data needed to access an HCI channel.
 */
struct cg2900_device {
	int				h4_channel;
	struct cg2900_callbacks		*cb;
	bool				logger_enabled;
	void				*user_data;
	struct device			*dev;
};

/**
 * struct cg2900_callbacks - Callback structure for CG2900 user.
 * @read_cb:	Callback function called when data is received from
 *		the connectivity controller.
 * @reset_cb:	Callback function called when the connectivity controller has
 *		been reset.
 *
 * Defines the callback functions provided from the caller.
 */
struct cg2900_callbacks {
	void (*read_cb) (struct cg2900_device *dev, struct sk_buff *skb);
	void (*reset_cb) (struct cg2900_device *dev);
};

/**
 * struct cg2900_rev_data - Contains revision data for the local controller.
 * @revision:		Revision of the controller, e.g. to indicate that it is
 *			a CG2900 controller.
 * @sub_version:	Subversion of the controller, e.g. to indicate a certain
 *			tape-out of the controller.
 *
 * The values to match retrieved values to each controller may be retrieved from
 * the manufacturer.
 */
struct cg2900_rev_data {
	int revision;
	int sub_version;
};

/**
 * struct cg2900_platform_data - Contains platform data for CG2900.
 * @init:		Callback called upon system start.
 * @exit:		Callback called upon system shutdown.
 * @enable_chip:	Callback called for enabling CG2900 chip.
 * @disable_chip:	Callback called for disabling CG2900 chip.
 * @set_hci_revision:	Callback called when HCI revision has been detected.
 * @get_power_switch_off_cmd:	Callback called to retrieve
 *				HCI VS_Power_Switch_Off command (command
 *				HCI requires platform specific GPIO data).
 * @bus:		Transport used, see @include/net/bluetooth/hci.h.
 * @cts_irq:		Interrupt for the UART CTS pin.
 * @enable_uart:	Callback called when switching from UART GPIO to
 *			UART HW.
 * @disable_uart:	Callback called when switching from UART HW to
 *			UART GPIO.
 * @uart:		Platform data structure for UART transport.
 *
 * Any callback may be NULL if not needed.
 */
struct cg2900_platform_data {
	int (*init)(void);
	void (*exit)(void);
	void (*enable_chip)(void);
	void (*disable_chip)(void);
	void (*set_hci_revision)(u8 hci_version, u16 hci_revision,
				 u8 lmp_version, u8 lmp_subversion,
				 u16 manufacturer);
	struct sk_buff* (*get_power_switch_off_cmd)(u16 *op_code);

	__u8 bus;

	struct {
		int cts_irq;
		int (*enable_uart)(void);
		int (*disable_uart)(void);
	} uart;
};

/**
 * struct cg2900_bt_platform_data - Contains platform data for CG2900 Bluetooth.
 * @bus:	Transport used, see @include/net/bluetooth/hci.h.
 */
struct cg2900_bt_platform_data {
	__u8 bus;
};

extern struct cg2900_device *cg2900_register_user(char *name,
						  struct cg2900_callbacks *cb);
extern void cg2900_deregister_user(struct cg2900_device *dev);
extern int cg2900_reset(struct cg2900_device *dev);
extern struct sk_buff *cg2900_alloc_skb(unsigned int size, gfp_t priority);
extern int cg2900_write(struct cg2900_device *dev, struct sk_buff *skb);
extern bool cg2900_get_local_revision(struct cg2900_rev_data *rev_data);

#endif /* _CG2900_H_ */
