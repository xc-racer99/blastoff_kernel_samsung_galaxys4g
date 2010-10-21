/*
 * Bluetooth driver for ST-Ericsson CG2900 connectivity controller.
 *
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Authors:
 * Par-Gunnar Hjalmdahl (par-gunnar.p.hjalmdahl@stericsson.com)
 * Henrik Possung (henrik.possung@stericsson.com)
 * Josef Kindberg (josef.kindberg@stericsson.com)
 * Dariusz Szymszak (dariusz.xd.szymczak@stericsson.com)
 * Kjell Andersson (kjell.k.andersson@stericsson.com)
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <asm/byteorder.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/mfd/cg2900.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <net/bluetooth/hci_core.h>

#define BT_VS_BT_ENABLE			0xFF10

#define VS_BT_DISABLE			0x00
#define VS_BT_ENABLE			0x01

#define BT_HEADER_LENGTH		0x03

#define STLC2690_HCI_REV		0x0600
#define CG2900_PG1_HCI_REV		0x0101
#define CG2900_PG2_HCI_REV		0x0200
#define CG2900_PG1_SPECIAL_HCI_REV	0x0700

#define NAME				"BTCG2900 "

/* Wait for 5 seconds for a response to our requests */
#define RESP_TIMEOUT			5000

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
 * struct btcg2900_info - Specifies HCI driver private data.
 *
 * This type specifies CG2900 HCI driver private data.
 *
 * @bt_cmd:		Device structure for BT command channel.
 * @bt_evt:		Device structure for BT event channel.
 * @bt_acl:		Device structure for BT ACL channel.
 * @pdev:		Device structure for platform device.
 * @hdev:		Device structure for HCI device.
 * @reset_state:	Device enum for HCI driver reset state.
 * @enable_state:	Device enum for HCI driver BT enable state.
 */
struct btcg2900_info {
	struct cg2900_device	*bt_cmd;
	struct cg2900_device	*bt_evt;
	struct cg2900_device	*bt_acl;
	struct platform_device	*pdev;
	struct hci_dev		*hdev;
	enum reset_state	reset_state;
	enum enable_state	enable_state;
};

/**
 * struct dev_info - Specifies private data used when receiving callbacks from CG2900 driver.
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

static struct btcg2900_info *btcg2900_info;

/*
 * hci_wait_queue - Main Wait Queue in HCI driver.
 */
static DECLARE_WAIT_QUEUE_HEAD(hci_wait_queue);

/* Internal function declarations */
static int register_bluetooth(void);

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
		BT_ERR(NAME "Could not allocate skb");
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
static void remove_bt_users(struct btcg2900_info *info)
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
		BT_ERR(NAME "NULL supplied for skb");
		return;
	}

	if (!dev) {
		BT_ERR(NAME "dev == NULL");
		goto fin_free_skb;
	}

	dev_info = (struct dev_info *)dev->user_data;

	if (!dev_info) {
		BT_ERR(NAME "dev_info == NULL");
		goto fin_free_skb;
	}

	evt = (struct hci_event_hdr *)skb->data;
	cmd_complete = (struct hci_ev_cmd_complete *)(skb->data + sizeof(*evt));
	cmd_status = (struct hci_ev_cmd_status *)(skb->data + sizeof(*evt));

	/*
	 * Check if HCI Driver it self is expecting a Command Complete packet
	 * from the chip after a BT Enable command.
	 */
	if ((btcg2900_info->enable_state == ENABLE_WAITING_BT_ENABLED_CC ||
	     btcg2900_info->enable_state == ENABLE_WAITING_BT_DISABLED_CC) &&
	    btcg2900_info->bt_evt->h4_channel == dev->h4_channel &&
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
			BT_ERR(NAME "Could not enable/disable BT core (0x%X)",
				   status);
			BT_DBG("New enable_state: ENABLE_BT_ERROR");
			btcg2900_info->enable_state = ENABLE_BT_ERROR;
			goto fin_free_skb;
		}

		if (btcg2900_info->enable_state ==
				ENABLE_WAITING_BT_ENABLED_CC) {
			BT_DBG("New enable_state: ENABLE_BT_ENABLED");
			btcg2900_info->enable_state = ENABLE_BT_ENABLED;
			BT_INFO("CG2900 BT core is enabled");
		} else {
			BT_DBG("New enable_state: ENABLE_BT_DISABLED");
			btcg2900_info->enable_state = ENABLE_BT_DISABLED;
			BT_INFO("CG2900 BT core is disabled");
		}

		/* Wake up whom ever is waiting for this result. */
		wake_up_interruptible(&hci_wait_queue);
		goto fin_free_skb;
	} else if ((btcg2900_info->enable_state ==
			ENABLE_WAITING_BT_DISABLED_CC ||
		    btcg2900_info->enable_state ==
			ENABLE_WAITING_BT_ENABLED_CC) &&
		   btcg2900_info->bt_evt->h4_channel == dev->h4_channel &&
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
			BT_ERR(NAME "Failed in supplying packet to Bluetooth"
			       " stack (%d)", err);
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
	struct btcg2900_info *info;

	BT_INFO(NAME "hci_reset_cb");

	if (!dev) {
		BT_ERR(NAME "NULL supplied for dev");
		return;
	}

	dev_info = (struct dev_info *)dev->user_data;
	if (!dev_info) {
		BT_ERR(NAME "NULL supplied for dev_info");
		return;
	}

	hdev = dev_info->hdev;
	if (!hdev) {
		BT_ERR(NAME "NULL supplied for hdev");
		return;
	}

	info = (struct btcg2900_info *)hdev->driver_data;
	if (!info) {
		BT_ERR(NAME "NULL supplied for driver_data");
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
		BT_ERR(NAME "Unknown HCI data type:%d",
		       dev_info->hci_data_type);
		return;
	}

	BT_DBG("New reset_state: RESET_ACTIVATED");
	btcg2900_info->reset_state = RESET_ACTIVATED;

	/*
	 * Free userdata as device info structure will be freed by CG2900
	 * when this callback returns.
	 */
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
	BT_DBG("Deregister HCI device");
	err = hci_unregister_dev(hdev);
	if (err)
		BT_ERR(NAME "Can not deregister HCI device! (%d)", err);
		/*
		 * Now we are in trouble. Try to register a new hdev
		 * anyway even though this will cost some memory.
		 */

	wait_event_interruptible_timeout(hci_wait_queue,
			(RESET_UNREGISTERED == btcg2900_info->reset_state),
			msecs_to_jiffies(RESP_TIMEOUT));
	if (RESET_UNREGISTERED != btcg2900_info->reset_state)
		/*
		 * Now we are in trouble. Try to register a new hdev
		 * anyway even though this will cost some memory.
		 */
		BT_ERR(NAME "Timeout expired. Could not deregister HCI device");

	/* Init and register hdev */
	BT_DBG("Register HCI device");
	err = register_bluetooth();
	if (err)
		BT_ERR(NAME "HCI Device registration error (%d).", err);
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
 * btcg2900_open() - Open HCI interface.
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
static int btcg2900_open(struct hci_dev *hdev)
{
	struct btcg2900_info *info;
	struct dev_info *dev_info;
	struct sk_buff *enable_cmd;
	int err;

	BT_INFO("Open ST-Ericsson CG2900 driver");

	if (!hdev) {
		BT_ERR(NAME "NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct btcg2900_info *)hdev->driver_data;
	if (!info) {
		BT_ERR(NAME "NULL supplied for driver_data");
		return -EINVAL;
	}

	if (test_and_set_bit(HCI_RUNNING, &(hdev->flags))) {
		BT_ERR(NAME "Device already opened!");
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

	if (info->reset_state == RESET_ACTIVATED) {
		BT_DBG("New reset_state: RESET_IDLE");
		btcg2900_info->reset_state = RESET_IDLE;
	}

	/*
	 * Call function that returns the chip dependent vs_bt_enable(true)
	 * HCI command.
	 * If NULL is returned, then no bt_enable command should be sent to the
	 * chip.
	 */
	enable_cmd = get_bt_enable_cmd(true);
	if (!enable_cmd) {
		/* The chip is enabled by default */
		BT_DBG("New enable_state: ENABLE_BT_ENABLED");
		btcg2900_info->enable_state = ENABLE_BT_ENABLED;
		return 0;
	}

	/* Set the HCI state before sending command to chip. */
	BT_DBG("New enable_state: ENABLE_WAITING_BT_ENABLED_CC");
	btcg2900_info->enable_state = ENABLE_WAITING_BT_ENABLED_CC;

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
		BT_ERR("Could not enable CG2900 BT core (%d)",
		       info->enable_state);
		err = -EACCES;
		BT_DBG("New enable_state: ENABLE_BT_DISABLED");
		btcg2900_info->enable_state = ENABLE_BT_DISABLED;
		goto handle_error;
	}

	return 0;

handle_error:
	remove_bt_users(info);
	clear_bit(HCI_RUNNING, &(hdev->flags));
	return err;

}

/**
 * btcg2900_close() - Close HCI interface.
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
static int btcg2900_close(struct hci_dev *hdev)
{
	struct btcg2900_info *info = NULL;
	struct sk_buff *enable_cmd;

	BT_DBG("btcg2900_close");

	if (!hdev) {
		BT_ERR(NAME "NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct btcg2900_info *)hdev->driver_data;
	if (!info) {
		BT_ERR(NAME "NULL supplied for driver_data");
		return -EINVAL;
	}

	if (!test_and_clear_bit(HCI_RUNNING, &(hdev->flags))) {
		BT_ERR(NAME "Device already closed!");
		return -EBUSY;
	}

	/* Do not do this if there is an reset ongoing */
	if (btcg2900_info->reset_state == RESET_ACTIVATED)
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
		BT_DBG("New enable_state: ENABLE_BT_ENABLED");
		btcg2900_info->enable_state = ENABLE_BT_ENABLED;
		goto remove_users;
	}

	/* Set the HCI state before sending command to chip */
	BT_DBG("New enable_state: ENABLE_WAITING_BT_DISABLED_CC");
	btcg2900_info->enable_state = ENABLE_WAITING_BT_DISABLED_CC;

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
		BT_ERR("Could not disable CG2900 BT core.");
		BT_DBG("New enable_state: ENABLE_BT_ENABLED");
		btcg2900_info->enable_state = ENABLE_BT_ENABLED;
	}

remove_users:
	/* Finally deregister all users and free allocated data */
	remove_bt_users(info);
	return 0;
}

/**
 * btcg2900_send() - Send packet to device.
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
static int btcg2900_send(struct sk_buff *skb)
{
	struct hci_dev *hdev;
	struct btcg2900_info *info;
	int err = 0;

	if (!skb) {
		BT_ERR(NAME "NULL supplied for skb");
		return -EINVAL;
	}

	hdev = (struct hci_dev *)(skb->dev);
	if (!hdev) {
		BT_ERR(NAME "NULL supplied for hdev");
		return -EINVAL;
	}

	info = (struct btcg2900_info *)hdev->driver_data;
	if (!info) {
		BT_ERR(NAME "NULL supplied for info");
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
		BT_ERR(NAME "Trying to transmit unsupported packet type"
		       " (0x%.2X)", bt_cb(skb)->pkt_type);
		err = -EOPNOTSUPP;
		break;
	};

	return err;
}

/**
 * btcg2900_destruct() - Destruct HCI interface.
 * @hdev:	HCI device being destructed.
 */
static void btcg2900_destruct(struct hci_dev *hdev)
{
	BT_DBG("btcg2900_destruct");

	if (!btcg2900_info)
		return;

	/*
	 * When destruct is called it means that the Bluetooth stack is done
	 * with the HCI device and we can now free it.
	 * Normally we do this only when removing the whole module through
	 * btcg2900_remove(), but when being reset we free the device here and
	 * we then set the reset state so that the reset handler can allocate a
	 * new HCI device and then register it to the Bluetooth stack.
	 */
	if (btcg2900_info->reset_state == RESET_ACTIVATED) {
		if (btcg2900_info->hdev)
			hci_free_dev(btcg2900_info->hdev);
		BT_DBG("New reset_state: RESET_UNREGISTERED");
		btcg2900_info->reset_state = RESET_UNREGISTERED;
		wake_up_interruptible(&hci_wait_queue);
	}
}

/**
 * register_bluetooth() - Initialize module.
 *
 * Alloc, init, and register HCI device to BlueZ.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation fails.
 *   Error codes from hci_register_dev.
 */
static int register_bluetooth(void)
{
	int err;
	static struct cg2900_bt_platform_data *pf_data;

	pf_data = dev_get_platdata(&btcg2900_info->pdev->dev);

	btcg2900_info->hdev = hci_alloc_dev();
	if (!btcg2900_info->hdev) {
		BT_ERR("Could not allocate mem for CG2900 BT driver");
		return -ENOMEM;
	}

	SET_HCIDEV_DEV(btcg2900_info->hdev, &btcg2900_info->pdev->dev);
	if (pf_data) {
		btcg2900_info->hdev->bus = pf_data->bus;
	} else {
		BT_DBG(NAME "Missing platform data. Defaulting to UART");
		btcg2900_info->hdev->bus = HCI_UART;
	}
	btcg2900_info->hdev->driver_data = btcg2900_info;
	btcg2900_info->hdev->owner = THIS_MODULE;
	btcg2900_info->hdev->open = btcg2900_open;
	btcg2900_info->hdev->close = btcg2900_close;
	btcg2900_info->hdev->send = btcg2900_send;
	btcg2900_info->hdev->destruct = btcg2900_destruct;

	err = hci_register_dev(btcg2900_info->hdev);
	if (err) {
		BT_ERR(NAME "Can not register HCI device (%d)", err);
		hci_free_dev(btcg2900_info->hdev);
	}

	BT_DBG("New enable_state: ENABLE_IDLE");
	btcg2900_info->enable_state = ENABLE_IDLE;
	BT_DBG("New reset_state: RESET_IDLE");
	btcg2900_info->reset_state = RESET_IDLE;

	return err;
}

/**
 * btcg2900_probe() - Initialize module.
 *
 * Allocate and initialize private data. Register to Bluetooth stack.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation fails.
 *   Error codes from register_bluetooth.
 */
static int __devinit btcg2900_probe(struct platform_device *pdev)
{
	int err;

	BT_INFO("btcg2900_probe");

	/* Initialize private data. */
	btcg2900_info = kzalloc(sizeof(*btcg2900_info), GFP_KERNEL);
	if (!btcg2900_info) {
		BT_ERR("Could not alloc btcg2900_info struct.");
		return -ENOMEM;
	}

	btcg2900_info->pdev = pdev;

	/* Init and register hdev */
	err = register_bluetooth();
	if (err) {
		BT_ERR("HCI Device registration error (%d)", err);
		kfree(btcg2900_info);
		btcg2900_info = NULL;
		return err;
	}

	return 0;
}

/**
 * btcg2900_remove() - Remove module.
 */
static int __devexit btcg2900_remove(struct platform_device *pdev)
{
	int err = 0;

	BT_INFO("btcg2900_remove");

	if (!btcg2900_info)
		return 0;

	if (!btcg2900_info->hdev)
		goto finished;

	err = hci_unregister_dev(btcg2900_info->hdev);
	if (err)
		BT_ERR("Can not unregister HCI device (%d)", err);
	hci_free_dev(btcg2900_info->hdev);

finished:
	kfree(btcg2900_info);
	btcg2900_info = NULL;
	return err;
}

static struct platform_driver btcg2900_driver = {
	.driver = {
		.name	= "cg2900-bt",
		.owner	= THIS_MODULE,
	},
	.probe	= btcg2900_probe,
	.remove	= __devexit_p(btcg2900_remove),
};

/**
 * btcg2900_init() - Initialize module.
 *
 * Registers platform driver.
 */
static int __init btcg2900_init(void)
{
	BT_INFO("btcg2900_init");
	return platform_driver_register(&btcg2900_driver);
}

/**
 * btcg2900_exit() - Remove module.
 *
 * Unregisters platform driver.
 */
static void __exit btcg2900_exit(void)
{
	BT_INFO("btcg2900_exit");
	platform_driver_unregister(&btcg2900_driver);
}

module_init(btcg2900_init);
module_exit(btcg2900_exit);

MODULE_AUTHOR("Par-Gunnar Hjalmdahl ST-Ericsson");
MODULE_AUTHOR("Henrik Possung ST-Ericsson");
MODULE_AUTHOR("Josef Kindberg ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Linux Bluetooth HCI H:4 Driver for ST-Ericsson controller");
