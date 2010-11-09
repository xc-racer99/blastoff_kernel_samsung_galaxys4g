/*
 * drivers/mfd/cg2900/cg2900_core.h
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
 * Linux Bluetooth HCI H:4 Driver for ST-Ericsson CG2900 GPS/BT/FM controller.
 */

#ifndef _CG2900_CORE_H_
#define _CG2900_CORE_H_

#include <linux/device.h>
#include <linux/skbuff.h>

/* Bluetooth lengths */
#define HCI_BT_SEND_FILE_MAX_CHUNK_SIZE		254

#define BT_BDADDR_SIZE				6

/* Reserve 1 byte for the HCI H:4 header */
#define HCI_H4_SIZE				1
#define CG2900_SKB_RESERVE			HCI_H4_SIZE

/* Bluetooth error codes */
#define HCI_BT_ERROR_NO_ERROR			0x00

/* Standardized Bluetooth H:4 channels */
#define HCI_BT_CMD_H4_CHANNEL			0x01
#define HCI_BT_ACL_H4_CHANNEL			0x02
#define HCI_BT_SCO_H4_CHANNEL			0x03
#define HCI_BT_EVT_H4_CHANNEL			0x04

/* Default H4 channels which may change depending on connected controller */
#define HCI_FM_RADIO_H4_CHANNEL			0x08
#define HCI_GNSS_H4_CHANNEL			0x09

struct cg2900_h4_channels {
	int	bt_cmd_channel;
	int	bt_acl_channel;
	int	bt_evt_channel;
	int	gnss_channel;
	int	fm_radio_channel;
	int	debug_channel;
	int	ste_tools_channel;
	int	hci_logger_channel;
	int	us_ctrl_channel;
	int	core_channel;
};

/**
  * struct cg2900_hci_logger_config - Configures the HCI logger.
  * @bt_cmd_enable: Enable BT command logging.
  * @bt_acl_enable: Enable BT ACL logging.
  * @bt_evt_enable: Enable BT event logging.
  * @gnss_enable: Enable GNSS logging.
  * @fm_radio_enable: Enable FM radio logging.
  * @bt_audio_enable: Enable BT audio command logging.
  * @fm_radio_audio_enable: Enable FM radio audio command logging.
  *
  * Set using cg2900_write on CHANNEL_HCI_LOGGER H4 channel.
  */
struct cg2900_hci_logger_config {
	bool bt_cmd_enable;
	bool bt_acl_enable;
	bool bt_evt_enable;
	bool gnss_enable;
	bool fm_radio_enable;
	bool bt_audio_enable;
	bool fm_radio_audio_enable;
};

/**
 * struct cg2900_chip_info - Chip info structure.
 * @manufacturer:	Chip manufacturer.
 * @hci_revision:	Chip revision, i.e. which chip is this.
 * @hci_sub_version:	Chip sub-version, i.e. which tape-out is this.
 *
 * Note that these values match the Bluetooth Assigned Numbers,
 * see http://www.bluetooth.org/
 */
struct cg2900_chip_info {
	int manufacturer;
	int hci_revision;
	int hci_sub_version;
};

struct cg2900_chip_dev;

/**
 * struct cg2900_chip_callbacks - Callback functions registered by chip handler.
 * @chip_startup:		Called when chip is started up.
 * @chip_shutdown:		Called when chip is shut down.
 * @data_to_chip:		Called when data shall be transmitted to chip.
 *				Return true when CG2900 Core shall not send it
 *				to chip.
 * @data_from_chip:		Called when data shall be transmitted to user.
 *				Return true when packet is taken care of by
 *				Return chip return handler.
 * @get_h4_channel:		Connects channel name with H:4 channel number.
 * @is_bt_audio_user:		Return true if current packet is for
 *				the BT audio user.
 * @is_fm_audio_user:		Return true if current packet is for
 *				the FM audio user.
 * @last_bt_user_removed:	Last BT channel user has been removed.
 * @last_fm_user_removed:	Last FM channel user has been removed.
 * @last_gnss_user_removed:	Last GNSS channel user has been removed.
 *
 * Note that some callbacks may be NULL. They must always be NULL checked before
 * calling.
 */
struct cg2900_chip_callbacks {
	int (*chip_startup)(struct cg2900_chip_dev *dev);
	int (*chip_shutdown)(struct cg2900_chip_dev *dev);
	bool (*data_to_chip)(struct cg2900_chip_dev *dev,
			     struct cg2900_device *cg2900_dev,
			     struct sk_buff *skb);
	bool (*data_from_chip)(struct cg2900_chip_dev *dev,
			       struct cg2900_device *cg2900_dev,
			       struct sk_buff *skb);
	int (*get_h4_channel)(char *name, int *h4_channel);
	bool (*is_bt_audio_user)(int h4_channel,
				 const struct sk_buff * const skb);
	bool (*is_fm_audio_user)(int h4_channel,
				 const struct sk_buff * const skb);
	void (*last_bt_user_removed)(struct cg2900_chip_dev *dev);
	void (*last_fm_user_removed)(struct cg2900_chip_dev *dev);
	void (*last_gnss_user_removed)(struct cg2900_chip_dev *dev);
};

/**
 * struct cg2900_chip_dev - Chip handler info structure.
 * @chip:	Chip info such as manufacturer.
 * @cb:		Callback structure for the chip handler.
 * @user_data:	Arbitrary data set by chip handler.
 */
struct cg2900_chip_dev {
	struct cg2900_chip_info		chip;
	struct cg2900_chip_callbacks	cb;
	void				*user_data;
};

/**
 * struct cg2900_id_callbacks - Chip handler identification callbacks.
 * @check_chip_support:	Called when chip is connected. If chip is supported by
 *			driver, return true and fill in @callbacks in @dev.
 *
 * Note that the callback may be NULL. It must always be NULL checked before
 * calling.
 */
struct cg2900_id_callbacks {
	bool (*check_chip_support)(struct cg2900_chip_dev *dev);
};

extern int cg2900_register_chip_driver(struct cg2900_id_callbacks *cb);
extern int cg2900_chip_startup_finished(int err);
extern int cg2900_chip_shutdown_finished(int err);
extern int cg2900_send_to_chip(struct sk_buff *skb, bool use_logger);
extern struct cg2900_device *cg2900_get_bt_cmd_dev(void);
extern struct cg2900_device *cg2900_get_fm_radio_dev(void);
extern struct cg2900_device *cg2900_get_bt_audio_dev(void);
extern struct cg2900_device *cg2900_get_fm_audio_dev(void);
extern struct cg2900_hci_logger_config *cg2900_get_hci_logger_config(void);

/* module_param declared in cg2900_core.c */
extern u8 bd_address[BT_BDADDR_SIZE];
extern int default_manufacturer;
extern int default_hci_revision;
extern int default_sub_version;

#endif /* _CG2900_CORE_H_ */
