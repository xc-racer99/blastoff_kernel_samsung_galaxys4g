/*
 * drivers/bluetooth/cg2900_hci.c
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
 * Linux Bluetooth HCI H:4 Driver for ST-Ericsson CG2900 connectivity controller
 * towards the BlueZ Bluetooth stack.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <asm/byteorder.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <net/bluetooth/hci_core.h>

#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/timer.h>

#include <linux/mfd/cg2900.h>

#define BT_VS_BT_ENABLE			0xFF10

#define VS_BT_DISABLE			0x00
#define VS_BT_ENABLE			0x01

#define BT_HEADER_LENGTH		0x03

#define STLC2690_HCI_REV		0x0600
#define CG2900_PG1_HCI_REV		0x0101
#define CG2900_PG2_HCI_REV		0x0200
#define CG2900_PG1_SPECIAL_HCI_REV	0x0700

#define NAME				"CG2900 HCI"

#define CG2900_SET_STATE(__name, __var, __new_state)		\
do {								\
	BT_DBG("New %s: 0x%X", __name, (uint32_t)__new_state);	\
	__var = __new_state;					\
} while (0)

/* HCI device type */
#define HCI_CG2900			HCI_VIRTUAL

/* Wait for 5 seconds for a response to our requests */
#define RESP_TIMEOUT			5000

/* State-setting defines */
#define SET_RESET_STATE(__hci_reset_new_state) \
	CG2900_SET_STATE("reset_state", hci_info->reset_state, \
			 __hci_reset_new_state)
#define SET_ENABLE_STATE(__hci_enable_new_state) \
	CG2900_SET_STATE("enable_state", hci_info->enable_state, \
			 __hci_enable_new_state)

/* Bluetooth error codes */
#define HCI_ERR_NO_ERROR		0x00
#define HCI_ERR_CMD_DISALLOWED		0x0C

/**
 * enum reset_state - RESET-states of the HCI driver.
 *
 * @RESET_IDLE:		No reset in progress.
 * @RESET_ACTIVATED:	Reset in progress.
 * @RESET_UNREGISTERED:	hdev is unregistered.
 */

enum reset_state {
	RESET_IDLE,
	RESET_ACTIVATED,
	RESET_UNREGISTERED
};

/**
 * enum enable_state - ENABLE-states of the HCI driver.
 *
 * @ENABLE_IDLE:			The HCI driver is loaded but not opened.
 * @ENABLE_WAITING_BT_ENABLED_CC:	The HCI driver is waiting for a command
 *					complete event from the BT chip as a
 *					response to a BT Enable (true) command.
 * @ENABLE_BT_ENABLED:			The BT chip is enabled.
 * @ENABLE_WAITING_BT_DISABLED_CC:	The HCI driver is waiting for a command
 *					complete event from the BT chip as a
 *					response to a BT Enable (false) command.
 * @ENABLE_BT_DISABLED:			The BT chip is disabled.
 * @ENABLE_BT_ERROR:			The HCI driver is in a bad state, some
 *					thing has failed and is not expected to
 *					work properly.
 */
enum enable_state {
	ENABLE_IDLE,
	ENABLE_WAITING_BT_ENABLED_CC,
	ENABLE_BT_ENABLED,
	ENABLE_WAITING_BT_DISABLED_CC,
	ENABLE_BT_DISABLED,
	ENABLE_BT_ERROR
};

/**
 * struct hci_info - Specifies HCI driver private data.
 *
 * This type specifies CG2900 HCI driver private data.
 *
 * @bt_cmd:		Device structure for BT command channel.
 * @bt_evt:		Device structure for BT event channel.
 * @bt_acl:		Device structure for BT ACL channel.
 * @hdev:		Device structure for HCI device.
 * @reset_state:	Device enum for HCI driver reset state.
 * @enable_state:	Device enum for HCI driver BT enable state.
 */
struct hci_info {
	struct cg2900_device	*bt_cmd;
	struct cg2900_device	*bt_evt;
	struct cg2900_device	*bt_acl;
	struct hci_dev		*hdev;
	enum reset_state	reset_state;
	enum enable_state	enable_state;
};

/**
 * struct dev_info - Specifies private data used when receiving callbacks from CPD.
 *
 * @hdev:		Device structure for HCI device.
 * @hci_data_type:	Type of data according to BlueZ.
 */
struct dev_info {
	struct hci_dev	*hdev;
	u8		hci_data_type;
};

/**
 * struct vs_bt_enable_cmd - Specifies HCI VS Bluetooth_Enable command.
 *
 * @op_code:	HCI command op code.
 * @len:	Parameter length of command.
 * @enable:	0 for disable BT, 1 for enable BT.
 */
struct vs_bt_enable_cmd {
	__le16	op_code;
	u8	len;
	u8	enable;
} __attribute__((packed));

static struct hci_info *hci_info;

/*
 * hci_wait_queue - Main Wait Queue in HCI driver.
 */
static DECLARE_WAIT_QUEUE_HEAD(hci_wait_queue);

/* Internal function declarations */
static int register_to_bluez(void);

/* Internal functions */

/**
 * get_bt_enable_cmd() - Get HCI BT enable command.
 * @bt_enable:	true if Bluetooth IP shall be enabled, false otherwise.
 *
 * Returns:
 *   NULL if no command shall be sent,
 *   sk_buffer with command otherwise.
 */
struct sk_buff *get_bt_enable_cmd(bool bt_enable)
{
	struct sk_buff *skb;
	struct vs_bt_enable_cmd *cmd;
	struct cg2900_rev_data rev_data;

	if (!cg2900_get_local_revision(&rev_data)) {
		BT_ERR(NAME "Couldn't get revision");
		return NULL;
	}

	/* If connected chip does not support the command return NULL */
	if (CG2900_PG1_SPECIAL_HCI_REV != rev_data.revision &&
	    CG2900_PG1_HCI_REV != rev_data.revision &&
	    CG2900_PG2_HCI_REV != rev_data.revision)
		return NULL;

	/* CG2900 used */
	skb = cg2900_alloc_skb(sizeof(*cmd), GFP_KERNEL);
	if (!skb) {
		BT_ERR("Could not allocate skb");
		return NULL;
	}

	cmd = (struct vs_bt_enable_cmd *)skb_put(skb, sizeof(*cmd));
	cmd->op_code = cpu_to_le16(BT_VS_BT_ENABLE);
	cmd->len = sizeof(*cmd) - BT_HEADER_LENGTH;
	if (bt_enable)
		cmd->enable = VS_BT_ENABLE;
	else
		cmd->enable = VS_BT_DISABLE;

	return skb;
}

/**
 * remove_bt_users() - Unregister and remove any existing BT users.
 * @info:	HCI driver info structure.
 */
static void remove_bt_users(struct hci_info *info)
{
	if (info->bt_cmd) {
		kfree(info->bt_cmd->user_data);
		info->bt_cmd->user_data = NULL;
		cg2900_deregister_user(info->bt_cmd);
		info->bt_cmd = NULL;
	}

	if (info->bt_evt) {
		kfree(info->bt_evt->user_data);
		info->bt_evt->user_data = NULL;
		cg2900_deregister_user(info->bt_evt);
		info->bt_evt = NULL;
	}

	if (info->bt_acl) {
		kfree(info->bt_acl->user_data);
		info->bt_acl->user_data = NULL;
		cg2900_deregister_user(info->bt_acl);
		info->bt_acl = NULL;
	}
}

/**
 * hci_read_cb() - Callback for handling data received from CG2900 driver.
 * @dev:	Device receiving data.
 * @skb:	Buffer with data coming from device.
 */
static void hci_read_cb(struct cg2900_device *dev, struct sk_buff *skb)
{
	int err = 0;
	struct dev_info *dev_info;
	struct hci_event_hdr *evt;
	struct hci_ev_cmd_complete *cmd_complete;
	struct hci_ev_cmd_status *cmd_status;
	u8 status;

	if (!skb) {
		BT_ERR("NULL supplied for skb");
		return;
	}

	if (!dev) {
		BT_ERR("dev == NULL");
		goto fin_free_skb;
	}

	dev_info = (struct dev_info *)dev->user_data;

	if (!dev_info) {
		BT_ERR("dev_info == NULL");
		goto fin_free_skb;
	}

	evt = (struct hci_event_hdr *)skb->data;
	cmd_complete = (struct hci_ev_cmd_complete *)(skb->data + sizeof(*evt));
	cmd_status = (struct hci_ev_cmd_status *)(skb->data + sizeof(*evt));

	/*
	 * Check if HCI Driver it self is expecting a Command Complete packet
	 * from the chip after a BT Enable command.
	 */
	if ((hci_info->enable_state == ENABLE_WAITING_BT_ENABLED_CC ||
	     hci_info->enable_state == ENABLE_WAITING_BT_DISABLED_CC) &&
	    hci_info->bt_evt->h4_channel == dev->h4_channel &&
	    evt->evt == HCI_EV_CMD_COMPLETE &&
	    le16_to_cpu(cmd_complete->opcode) == BT_VS_BT_ENABLE) {
		/*
		 * This is the command complete event for
		 * the HCI_Cmd_VS_Bluetooth_Enable.
		 * Check result and update state.
		 *
		 * The BT chip is enabled/disabled. Either it was enabled/
		 * disabled now (status NO_ERROR) or it was already enabled/
		 * disabled (assuming status CMD_DISALLOWED is already enabled/
		 * disabled).
		 */
		status = *(skb->data + sizeof(*evt) + sizeof(*cmd_complete));
		if (status != HCI_ERR_NO_ERROR &&
		    status != HCI_ERR_CMD_DISALLOWED) {
			BT_ERR("Could not enable/disable BT core (0x%X)",
				   status);
			SET_ENABLE_STATE(ENABLE_BT_ERROR);
			goto fin_free_skb;
		}

		if (hci_info->enable_state == ENABLE_WAITING_BT_ENABLED_CC) {
			SET_ENABLE_STATE(ENABLE_BT_ENABLED);
			BT_INFO("BT core is enabled");
		} else {
			SET_ENABLE_STATE(ENABLE_BT_DISABLED);
			BT_INFO("BT core is disabled");
		}

		/* Wake up whom ever is waiting for this result. */
		wake_up_interruptible(&hci_wait_queue);
		goto fin_free_skb;
	} else if ((hci_info->enable_state == ENABLE_WAITING_BT_DISABLED_CC ||
		    hci_info->enable_state == ENABLE_WAITING_BT_ENABLED_CC) &&
		   hci_info->bt_evt->h4_channel == dev->h4_channel &&
		   evt->evt == HCI_EV_CMD_STATUS &&
		   le16_to_cpu(cmd_status->opcode) == BT_VS_BT_ENABLE) {
		/*
		 * Clear the status events since the Bluez is not expecting
		 * them.
		 */
		BT_DBG("HCI Driver received Command Status (BT enable): 0x%X",
		       cmd_status->status);
		/*
		 * This is the command status event for
		 * the HCI_Cmd_VS_Bluetooth_Enable.
		 * Just free the packet.
		 */
		goto fin_free_skb;
	} else {
		bt_cb(skb)->pkt_type = dev_info->hci_data_type;
		skb->dev = (struct net_device *)dev_info->hdev;
		/* Update BlueZ stats */
		dev_info->hdev->stat.byte_rx += skb->len;
		if (bt_cb(skb)->pkt_type == HCI_ACLDATA_PKT)
			dev_info->hdev->stat.acl_rx++;
		else
			dev_info->hdev->stat.evt_rx++;

		BT_DBG("Data receive %d bytes", skb->len);

		/* Provide BlueZ with received frame*/
		err = hci_recv_frame(skb);
		/* If err, skb have been freed in hci_recv_frame() */
		if (err)
			BT_ERR("Failed in supplying packet to BlueZ (%d)", err);
	}

	return;

fin_free_skb:
	kfree_skb(skb);
}

/**
 * hci_reset_cb() - Callback for handling reset from CG2900 driver.
 * @dev:	CPD device resetting.
 */
static void hci_reset_cb(struct cg2900_device *dev)
{
	int err;
	struct hci_dev *hdev;
	struct dev_info *dev_info;
	struct hci_info *info;

	BT_INFO("BluezDriver: hci_reset_cb");

	if (!dev) {
		BT_ERR("NULL supplied for dev");
		return;
	}

	dev_info = (struct dev_info *)dev->user_data;
	if (!dev_info) {
		BT_ERR("NULL supplied for dev_info");
		return;
	}

	hdev = dev_info->hdev;
	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return;
	}

	info = (struct hci_info *)hdev->driver_data;
	if (!info) {
		BT_ERR("NULL supplied for driver_data");
		return;
	}

	switch (dev_info->hci_data_type) {

	case HCI_EVENT_PKT:
		info->bt_evt = NULL;
		break;

	case HCI_COMMAND_PKT:
		info->bt_cmd = NULL;
		break;

	case HCI_ACLDATA_PKT:
		info->bt_acl = NULL;
		break;

	default:
		BT_ERR("Unknown HCI data type:%d", dev_info->hci_data_type);
		return;
	}

	SET_RESET_STATE(RESET_ACTIVATED);

	/* Free userdata as device info structure will be freed by CG2900
	 * when this callback returns */
	kfree(dev->user_data);
	dev->user_data = NULL;

	/*
	 * Continue to deregister hdev if all channels has been reset else
	 * return.
	 */
	if (info->bt_evt || info->bt_cmd || info->bt_acl)
		return;

	/*
	 * Deregister HCI device. Close and Destruct functions should
	 * in turn be called by BlueZ.
	 */
	BT_INFO("Deregister HCI device");
	err = hci_unregister_dev(hdev);
	if (err)
		BT_ERR("Can not deregister HCI device! (%d)", err);
		/*
		 * Now we are in trouble. Try to register a new hdev
		 * anyway even though this will cost some memory.
		 */

	wait_event_interruptible_timeout(hci_wait_queue,
				(RESET_UNREGISTERED == hci_info->reset_state),
				msecs_to_jiffies(RESP_TIMEOUT));
	if (RESET_UNREGISTERED != hci_info->reset_state)
		/*
		 * Now we are in trouble. Try to register a new hdev
		 * anyway even though this will cost some memory.
		 */
		BT_ERR("Timeout expired. Could not deregister HCI device");

	/* Init and register hdev */
	BT_INFO("Register HCI device");
	err = register_to_bluez();
	if (err)
		BT_ERR("HCI Device registration error (%d).", err);
}

/*
 * struct cg2900_cb - Specifies callback structure for CG2900 user.
 *
 * @read_cb:	Callback function called when data is received from
 *		the controller.
 * @reset_cb:	Callback function called when the controller has been reset.
 */
static struct cg2900_callbacks cg2900_cb = {
	.read_cb = hci_read_cb,
	.reset_cb = hci_reset_cb
};

/**
 * open_from_hci() - Open HCI interface.
 * @hdev:	HCI device being opened.
 *
 * BlueZ callback function for opening HCI interface to device.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL if NULL pointer is supplied.
 *   -EOPNOTSUPP if supplied packet type is not supported.
 *   -EBUSY if device is already opened.
 *   -EACCES if opening of channels failed.
 */
static int open_from_hci(struct hci_dev *hdev)
{
	struct hci_info *info;
	struct dev_info *dev_info;
	struct sk_buff *enable_cmd;
	int err;

	BT_INFO("Open ST-Ericsson connectivity HCI driver");

	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct hci_info *)hdev->driver_data;
	if (!info) {
		BT_ERR("NULL supplied for driver_data");
		return -EINVAL;
	}

	if (test_and_set_bit(HCI_RUNNING, &(hdev->flags))) {
		BT_ERR("Device already opened!");
		return -EBUSY;
	}

	if (!(info->bt_cmd)) {
		info->bt_cmd = cg2900_register_user(CG2900_BT_CMD,
						     &cg2900_cb);
		if (info->bt_cmd) {
			dev_info = kmalloc(sizeof(*dev_info), GFP_KERNEL);
			if (dev_info) {
				dev_info->hdev = hdev;
				dev_info->hci_data_type = HCI_COMMAND_PKT;
			}
			info->bt_cmd->user_data = dev_info;
		} else {
			BT_ERR("Couldn't register CG2900_BT_CMD to CG2900");
			err = -EACCES;
			goto handle_error;
		}
	}

	if (!(info->bt_evt)) {
		info->bt_evt = cg2900_register_user(CG2900_BT_EVT,
						     &cg2900_cb);
		if (info->bt_evt) {
			dev_info = kmalloc(sizeof(*dev_info), GFP_KERNEL);
			if (dev_info) {
				dev_info->hdev = hdev;
				dev_info->hci_data_type = HCI_EVENT_PKT;
			}
			info->bt_evt->user_data = dev_info;
		} else {
			BT_ERR("Couldn't register CG2900_BT_EVT to CG2900");
			err = -EACCES;
			goto handle_error;
		}
	}

	if (!(info->bt_acl)) {
		info->bt_acl = cg2900_register_user(CG2900_BT_ACL,
						     &cg2900_cb);
		if (info->bt_acl) {
			dev_info = kmalloc(sizeof(*dev_info), GFP_KERNEL);
			if (dev_info) {
				dev_info->hdev = hdev;
				dev_info->hci_data_type = HCI_ACLDATA_PKT;
			}
			info->bt_acl->user_data = dev_info;
		} else {
			BT_ERR("Couldn't register CG2900_BT_ACL to CG2900");
			err = -EACCES;
			goto handle_error;
		}
	}

	if (info->reset_state == RESET_ACTIVATED)
		SET_RESET_STATE(RESET_IDLE);

	/*
	 * Call platform specific function that returns the chip dependent
	 * bt enable(true) HCI command.
	 * If NULL is returned, then no bt_enable command should be sent to the
	 * chip.
	 */
	enable_cmd = get_bt_enable_cmd(true);
	if (!enable_cmd) {
		/* The chip is enabled by default */
		SET_ENABLE_STATE(ENABLE_BT_ENABLED);
		return 0;
	}

	/* Set the HCI state before sending command to chip. */
	SET_ENABLE_STATE(ENABLE_WAITING_BT_ENABLED_CC);

	/* Send command to chip */
	cg2900_write(info->bt_cmd, enable_cmd);

	/*
	 * Wait for callback to receive command complete and then wake us up
	 * again.
	 */
	wait_event_interruptible_timeout(hci_wait_queue,
				(info->enable_state == ENABLE_BT_ENABLED),
				msecs_to_jiffies(RESP_TIMEOUT));
	/* Check the current state to se that it worked. */
	if (info->enable_state != ENABLE_BT_ENABLED) {
		BT_ERR("Could not enable BT core (%d)", info->enable_state);
		err = -EACCES;
		SET_ENABLE_STATE(ENABLE_BT_DISABLED);
		goto handle_error;
	}

	return 0;

handle_error:
	remove_bt_users(info);
	clear_bit(HCI_RUNNING, &(hdev->flags));
	return err;

}

/**
 * flush_from_hci() - Flush HCI interface.
 * @hdev:	HCI device being flushed.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL if NULL pointer is supplied.
 */
static int flush_from_hci(struct hci_dev *hdev)
{
	BT_INFO("flush_from_hci");

	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return -EINVAL;
	}

	return 0;
}

/**
 * close_from_hci() - Close HCI interface.
 * @hdev:	HCI device being closed.
 *
 * BlueZ callback function for closing HCI interface.
 * It flushes the interface first.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL if NULL pointer is supplied.
 *   -EOPNOTSUPP if supplied packet type is not supported.
 *   -EBUSY if device is not opened.
 */
static int close_from_hci(struct hci_dev *hdev)
{
	struct hci_info *info = NULL;
	struct sk_buff *enable_cmd;

	BT_INFO("close_from_hci");

	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct hci_info *)hdev->driver_data;
	if (!info) {
		BT_ERR("NULL supplied for driver_data");
		return -EINVAL;
	}

	if (!test_and_clear_bit(HCI_RUNNING, &(hdev->flags))) {
		BT_ERR("Device already closed!");
		return -EBUSY;
	}

	flush_from_hci(hdev);

	/* Do not do this if there is an reset ongoing */
	if (hci_info->reset_state == RESET_ACTIVATED)
		goto remove_users;

	/*
	 * Get the chip dependent BT Enable HCI command. The command parameter
	 * shall be set to false to disable the BT core.
	 * If NULL is returned, then no BT Enable command should be sent to the
	 * chip.
	 */
	enable_cmd = get_bt_enable_cmd(false);
	if (!enable_cmd) {
		/*
		 * The chip is enabled by default and we should not disable it.
		 */
		SET_ENABLE_STATE(ENABLE_BT_ENABLED);
		goto remove_users;
	}

	/* Set the HCI state before sending command to chip */
	SET_ENABLE_STATE(ENABLE_WAITING_BT_DISABLED_CC);

	/* Send command to chip */
	cg2900_write(info->bt_cmd, enable_cmd);

	/*
	 * Wait for callback to receive command complete and then wake us up
	 * again.
	 */
	wait_event_interruptible_timeout(hci_wait_queue,
				(info->enable_state == ENABLE_BT_DISABLED),
				msecs_to_jiffies(RESP_TIMEOUT));
	/* Check the current state to se that it worked. */
	if (info->enable_state != ENABLE_BT_DISABLED) {
		BT_ERR("Could not disable BT core.");
		SET_ENABLE_STATE(ENABLE_BT_ENABLED);
	}

remove_users:
	/* Finally deregister all users and free allocated data */
	remove_bt_users(info);
	return 0;
}

/**
 * send_from_hci() - Send packet to device.
 * @skb:	sk buffer to be sent.
 *
 * BlueZ callback function for sending sk buffer.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL if NULL pointer is supplied.
 *   -EOPNOTSUPP if supplied packet type is not supported.
 *   Error codes from cg2900_write.
 */
static int send_from_hci(struct sk_buff *skb)
{
	struct hci_dev *hdev;
	struct hci_info *info;
	int err = 0;

	if (!skb) {
		BT_ERR("NULL supplied for skb");
		return -EINVAL;
	}

	hdev = (struct hci_dev *)(skb->dev);
	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct hci_info *)hdev->driver_data;
	if (!info) {
		BT_ERR("NULL supplied for info");
		return -EINVAL;
	}

	/* Update BlueZ stats */
	hdev->stat.byte_tx += skb->len;

	BT_DBG("Data transmit %d bytes", skb->len);

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		BT_DBG("Sending HCI_COMMAND_PKT");
		err = cg2900_write(info->bt_cmd, skb);
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		BT_DBG("Sending HCI_ACLDATA_PKT");
		err = cg2900_write(info->bt_acl, skb);
		hdev->stat.acl_tx++;
		break;
	default:
		BT_ERR("Trying to transmit unsupported packet type (0x%.2X)",
		       bt_cb(skb)->pkt_type);
		err = -EOPNOTSUPP;
		break;
	};

	return err;
}

/**
 * destruct_from_hci() - Destruct HCI interface.
 * @hdev:	HCI device being destructed.
 */
static void destruct_from_hci(struct hci_dev *hdev)
{
	BT_INFO("destruct_from_hci");

	if (!hci_info)
		return;

	/* When being reset, register a new hdev when hdev is deregistered */
	if (hci_info->reset_state == RESET_ACTIVATED) {
		if (hci_info->hdev)
			hci_free_dev(hci_info->hdev);
		SET_RESET_STATE(RESET_UNREGISTERED);
	}
}

/**
 * notify_from_hci() - Notification from the HCI interface.
 * @hdev:	HCI device notifying.
 * @evt:	Notification event.
 */
static void notify_from_hci(struct hci_dev *hdev, unsigned int evt)
{
	BT_INFO("notify_from_hci - evt = 0x%.2X", evt);
}

/**
 * ioctl_from_hci() - Handle IOCTL call to the HCI interface.
 * @hdev: HCI device.
 * @cmd:  IOCTL command.
 * @arg:  IOCTL argument.
 *
 * Returns:
 *   -EINVAL if NULL is supplied for hdev.
 *   -EPERM otherwise since current driver supports no IOCTL.
 */
static int ioctl_from_hci(struct hci_dev *hdev, unsigned int cmd,
			  unsigned long arg)
{
	BT_INFO("ioctl_from_hci - cmd 0x%X", cmd);
	BT_DBG("DIR: %d, TYPE: %d, NR: %d, SIZE: %d",
	       _IOC_DIR(cmd), _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd));

	if (!hdev) {
		BT_ERR("NULL supplied for hdev");
		return -EINVAL;;
	}

	return -EPERM;
}

/**
 * register_to_bluez() - Initialize module.
 *
 * Alloc, init, and register HCI device to BlueZ.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation fails.
 *   Error codes from hci_register_dev.
 */
static int register_to_bluez(void)
{
	int err;

	hci_info->hdev = hci_alloc_dev();
	if (!hci_info->hdev) {
		BT_ERR("Could not allocate mem for HCI driver");
		return -ENOMEM;
	}

	hci_info->hdev->bus = HCI_CG2900;
	hci_info->hdev->driver_data = hci_info;
	hci_info->hdev->owner = THIS_MODULE;
	hci_info->hdev->open = open_from_hci;
	hci_info->hdev->close = close_from_hci;
	hci_info->hdev->flush = flush_from_hci;
	hci_info->hdev->send = send_from_hci;
	hci_info->hdev->destruct = destruct_from_hci;
	hci_info->hdev->notify = notify_from_hci;
	hci_info->hdev->ioctl = ioctl_from_hci;

	err = hci_register_dev(hci_info->hdev);
	if (err) {
		BT_ERR("Can not register HCI device (%d)", err);
		hci_free_dev(hci_info->hdev);
	}

	SET_ENABLE_STATE(ENABLE_IDLE);
	SET_RESET_STATE(RESET_IDLE);

	return err;
}

/**
 * hci_init() - Initialize module.
 *
 * Allocate and initialize private data. Register to BlueZ.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation fails.
 *   Error codes from register_to_bluez.
 */
static int __init hci_init(void)
{
	int err;
	BT_INFO("hci_init");

	/* Initialize private data. */
	hci_info = kzalloc(sizeof(*hci_info), GFP_KERNEL);
	if (!hci_info) {
		BT_ERR("Could not alloc hci_info struct.");
		return -ENOMEM;
	}

	/* Init and register hdev */
	err = register_to_bluez();
	if (err) {
		BT_ERR("HCI Device registration error (%d)", err);
		kfree(hci_info);
		hci_info = NULL;
		return err;
	}

	return 0;
}

/**
 * hci_exit() - Remove module.
 *
 * Remove HCI device from CG2900 driver.
 */
static void __exit hci_exit(void)
{
	int err = 0;

	BT_INFO("hci_exit");

	if (!hci_info)
		return;

	if (!hci_info->hdev)
		goto finished;

	err = hci_unregister_dev(hci_info->hdev);
	if (err)
		BT_ERR("Can not unregister HCI device (%d)", err);
	hci_free_dev(hci_info->hdev);

finished:
	kfree(hci_info);
	hci_info = NULL;
}

module_init(hci_init);
module_exit(hci_exit);

MODULE_AUTHOR("Par-Gunnar Hjalmdahl ST-Ericsson");
MODULE_AUTHOR("Henrik Possung ST-Ericsson");
MODULE_AUTHOR("Josef Kindberg ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Linux Bluetooth HCI H:4 Driver for ST-Ericsson controller");
