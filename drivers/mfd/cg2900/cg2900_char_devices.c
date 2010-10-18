/*
 * drivers/mfd/cg2900/cg2900_char_devices.c
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
 * Linux Bluetooth HCI H:4 Driver for ST-Ericsson connectivity controller.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/mfd/cg2900.h>

#include "cg2900_core.h"
#include "cg2900_debug.h"

#define NAME					"CharDev "

/* Ioctls */
#define CG2900_CHAR_DEV_IOCTL_RESET		_IOW('U', 210, int)
#define CG2900_CHAR_DEV_IOCTL_CHECK4RESET	_IOR('U', 212, int)
#define CG2900_CHAR_DEV_IOCTL_GET_REVISION	_IOR('U', 213, int)
#define CG2900_CHAR_DEV_IOCTL_GET_SUB_VER	_IOR('U', 214, int)

#define CG2900_CHAR_DEV_IOCTL_EVENT_RESET	1
#define CG2900_CHAR_DEV_IOCTL_EVENT_CLOSED	2

/* Internal type definitions */

/**
 * enum char_reset_state - Reset state.
 * @CG2900_CHAR_IDLE:	Idle state.
 * @CG2900_CHAR_RESET:	Reset state.
 */
enum char_reset_state {
	CG2900_CHAR_IDLE,
	CG2900_CHAR_RESET
};

/**
 * struct char_dev_user - Stores device information.
 * @dev:		Registered CG2900 Core device.
 * @miscdev:	Registered device struct.
 * @name:		Name of device.
 * @rx_queue:		Data queue.
 * @rx_wait_queue:	Wait queue.
 * @reset_wait_queue:	Reset Wait queue.
 * @reset_state:	Reset state.
 * @read_mutex:		Read mutex.
 * @write_mutex:	Write mutex.
 * @list:		List header for inserting into device list.
 */
struct char_dev_user {
	struct cg2900_device	*dev;
	struct miscdevice	*miscdev;
	char			*name;
	struct sk_buff_head	rx_queue;
	wait_queue_head_t	rx_wait_queue;
	wait_queue_head_t	reset_wait_queue;
	enum char_reset_state	reset_state;
	struct mutex		read_mutex;
	struct mutex		write_mutex;
	struct list_head	list;
};

/**
 * struct char_info - Stores all current users.
 * @open_mutex:	Open mutex (used for both open and release).
 * @dev_users:	List of char dev users.
 */
struct char_info {
	struct mutex		open_mutex;
	struct list_head	dev_users;
};

static struct char_info *char_info;

/**
 * char_dev_read_cb() - Handle data received from controller.
 * @dev:	Device receiving data.
 * @skb:	Buffer with data coming from controller.
 *
 * The char_dev_read_cb() function handles data received from STE-CG2900 driver.
 */
static void char_dev_read_cb(struct cg2900_device *dev, struct sk_buff *skb)
{
	struct char_dev_user *char_dev = (struct char_dev_user *)dev->user_data;

	CG2900_INFO("char_dev_read_cb");

	if (!char_dev) {
		CG2900_ERR("No char dev! Exiting");
		kfree_skb(skb);
		return;
	}

	skb_queue_tail(&char_dev->rx_queue, skb);

	wake_up_interruptible(&char_dev->rx_wait_queue);
}

/**
 * char_dev_reset_cb() - Handle reset from controller.
 * @dev:	Device resetting.
 *
 * The char_dev_reset_cb() function handles reset from the CG2900 driver.
 */
static void char_dev_reset_cb(struct cg2900_device *dev)
{
	struct char_dev_user *char_dev = (struct char_dev_user *)dev->user_data;

	CG2900_INFO("char_dev_reset_cb");

	if (!char_dev) {
		CG2900_ERR("char_dev == NULL");
		return;
	}

	char_dev->reset_state = CG2900_CHAR_RESET;
	/*
	 * The device will be freed by CG2900 Core when this function is
	 * finished.
	 */
	char_dev->dev = NULL;

	wake_up_interruptible(&char_dev->rx_wait_queue);
	wake_up_interruptible(&char_dev->reset_wait_queue);
}

static struct cg2900_callbacks char_cb = {
	.read_cb = char_dev_read_cb,
	.reset_cb = char_dev_reset_cb
};

/**
 * char_dev_open() - Open char device.
 * @inode:	Device driver information.
 * @filp:	Pointer to the file struct.
 *
 * The char_dev_open() function opens the char device.
 *
 * Returns:
 *   0 if there is no error.
 *   -EACCES if device was already registered to driver or if registration
 *   failed.
 */
static int char_dev_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	int minor;
	struct char_dev_user *dev = NULL;
	struct char_dev_user *tmp;
	struct list_head *cursor;

	mutex_lock(&char_info->open_mutex);

	minor = iminor(inode);

	/* Find the device for this file */
	list_for_each(cursor, &char_info->dev_users) {
		tmp = list_entry(cursor, struct char_dev_user, list);
		if (tmp->miscdev->minor == minor) {
			dev = tmp;
			break;
		}
	}
	if (!dev) {
		CG2900_ERR("Could not identify device in inode");
		err = -EINVAL;
		goto error_handling;
	}

	filp->private_data = dev;

	CG2900_INFO("char_dev_open %s", dev->name);

	if (dev->dev) {
		CG2900_ERR("Device already registered to CG2900 Driver");
		err = -EACCES;
		goto error_handling;
	}
	/* First initiate wait queues for this device. */
	init_waitqueue_head(&dev->rx_wait_queue);
	init_waitqueue_head(&dev->reset_wait_queue);

	dev->reset_state = CG2900_CHAR_IDLE;

	/* Register to CG2900 Driver */
	dev->dev = cg2900_register_user(dev->name, &char_cb);
	if (dev->dev)
		dev->dev->user_data = dev;
	else {
		CG2900_ERR("Couldn't register to CG2900 for H:4 channel %s",
			   dev->name);
		err = -EACCES;
	}

error_handling:
	mutex_unlock(&char_info->open_mutex);
	return err;
}

/**
 * char_dev_release() - Release char device.
 * @inode:	Device driver information.
 * @filp:	Pointer to the file struct.
 *
 * The char_dev_release() function release the char device.
 *
 * Returns:
 *   0 if there is no error.
 *   -EBADF if NULL pointer was supplied in private data.
 */
static int char_dev_release(struct inode *inode, struct file *filp)
{
	int err = 0;
	struct char_dev_user *dev = (struct char_dev_user *)filp->private_data;

	CG2900_INFO("char_dev_release");

	if (!dev) {
		CG2900_ERR("Calling with NULL pointer");
		return -EBADF;
	}

	mutex_lock(&char_info->open_mutex);
	mutex_lock(&dev->read_mutex);
	mutex_lock(&dev->write_mutex);

	if (dev->reset_state == CG2900_CHAR_IDLE)
		cg2900_deregister_user(dev->dev);

	dev->dev = NULL;
	filp->private_data = NULL;
	wake_up_interruptible(&dev->rx_wait_queue);
	wake_up_interruptible(&dev->reset_wait_queue);

	mutex_unlock(&dev->write_mutex);
	mutex_unlock(&dev->read_mutex);
	mutex_unlock(&char_info->open_mutex);

	return err;
}

/**
 * char_dev_read() - Queue and copy buffer to user.
 * @filp:	Pointer to the file struct.
 * @buf:	Received buffer.
 * @count:	Size of buffer.
 * @f_pos:	Position in buffer.
 *
 * The char_dev_read() function queues and copy the received buffer to
 * the user space char device. If no data is available this function will block.
 *
 * Returns:
 *   Bytes successfully read (could be 0).
 *   -EBADF if NULL pointer was supplied in private data.
 *   -EFAULT if copy_to_user fails.
 *   Error codes from wait_event_interruptible.
 */
static ssize_t char_dev_read(struct file *filp, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	struct char_dev_user *dev = (struct char_dev_user *)filp->private_data;
	struct sk_buff *skb;
	int bytes_to_copy;
	int err = 0;

	CG2900_INFO("char_dev_read");

	if (!dev) {
		CG2900_ERR("Calling with NULL pointer");
		return -EBADF;
	}
	mutex_lock(&dev->read_mutex);

	if (skb_queue_empty(&dev->rx_queue)) {
		err = wait_event_interruptible(dev->rx_wait_queue,
				(!(skb_queue_empty(&dev->rx_queue))) ||
				(CG2900_CHAR_RESET == dev->reset_state) ||
				(dev->dev == NULL));
		if (err) {
			CG2900_ERR("Failed to wait for event");
			goto error_handling;
		}
	}

	if (!dev->dev) {
		CG2900_DBG("dev is empty - return with negative bytes");
		err = -EBADF;
		goto error_handling;
	}

	skb = skb_dequeue(&dev->rx_queue);
	if (!skb) {
		CG2900_DBG("skb queue is empty - return with zero bytes");
		bytes_to_copy = 0;
		goto finished;
	}

	bytes_to_copy = min(count, skb->len);

	err = copy_to_user(buf, skb->data, bytes_to_copy);
	if (err) {
		skb_queue_head(&dev->rx_queue, skb);
		err = -EFAULT;
		goto error_handling;
	}

	skb_pull(skb, bytes_to_copy);

	if (skb->len > 0)
		skb_queue_head(&dev->rx_queue, skb);
	else
		kfree_skb(skb);

	goto finished;

error_handling:
	mutex_unlock(&dev->read_mutex);
	return (ssize_t)err;
finished:
	mutex_unlock(&dev->read_mutex);
	return bytes_to_copy;
}

/**
 * char_dev_write() - Copy buffer from user and write to CG2900 driver.
 * @filp:	Pointer to the file struct.
 * @buf:	Write buffer.
 * @count:	Size of the buffer write.
 * @f_pos:	Position of buffer.
 *
 * Returns:
 *   Bytes successfully written (could be 0).
 *   -EBADF if NULL pointer was supplied in private data.
 *   -EFAULT if copy_from_user fails.
 */
static ssize_t char_dev_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct sk_buff *skb;
	struct char_dev_user *dev = (struct char_dev_user *)filp->private_data;
	int err = 0;

	CG2900_INFO("char_dev_write");

	if (!dev) {
		CG2900_ERR("Calling with NULL pointer");
		return -EBADF;
	}
	mutex_lock(&dev->write_mutex);

	skb = cg2900_alloc_skb(count, GFP_ATOMIC);
	if (!skb) {
		CG2900_ERR("Couldn't allocate sk_buff with length %d", count);
		goto error_handling;
	}

	if (copy_from_user(skb_put(skb, count), buf, count)) {
		kfree_skb(skb);
		err = -EFAULT;
		goto error_handling;
	}

	err = cg2900_write(dev->dev, skb);
	if (err) {
		CG2900_ERR("cg2900_write failed (%d)", err);
		kfree_skb(skb);
		goto error_handling;
	}

	mutex_unlock(&dev->write_mutex);
	return count;

error_handling:
	mutex_unlock(&dev->write_mutex);
	return err;
}

/**
 * char_dev_unlocked_ioctl() - Handle IOCTL call to the interface.
 * @filp:	Pointer to the file struct.
 * @cmd:	IOCTL command.
 * @arg:	IOCTL argument.
 *
 * Returns:
 *   0 if there is no error.
 *   -EBADF if NULL pointer was supplied in private data.
 *   -EINVAL if supplied cmd is not supported.
 *   For cmd CG2900_CHAR_DEV_IOCTL_CHECK4RESET 0x01 is returned if device is
 *   reset and 0x02 is returned if device is closed.
 */
static long char_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	struct char_dev_user *dev = (struct char_dev_user *)filp->private_data;
	struct cg2900_rev_data rev_data;
	int err = 0;

	CG2900_INFO("char_dev_unlocked_ioctl cmd %d for %s", cmd, dev->name);
	CG2900_DBG("DIR: %d, TYPE: %d, NR: %d, SIZE: %d",
		     _IOC_DIR(cmd), _IOC_TYPE(cmd), _IOC_NR(cmd),
		     _IOC_SIZE(cmd));

	switch (cmd) {
	case CG2900_CHAR_DEV_IOCTL_RESET:
		if (!dev) {
			err = -EBADF;
			goto error_handling;
		}
		CG2900_INFO("ioctl reset command for device %s", dev->name);
		err = cg2900_reset(dev->dev);
		break;

	case CG2900_CHAR_DEV_IOCTL_CHECK4RESET:
		if (!dev) {
			CG2900_INFO("ioctl check for reset command for device");
			/* Return positive value if closed */
			err = CG2900_CHAR_DEV_IOCTL_EVENT_CLOSED;
		} else if (dev->reset_state == CG2900_CHAR_RESET) {
			CG2900_INFO("ioctl check for reset command for device "
				    "%s", dev->name);
			/* Return positive value if reset */
			err = CG2900_CHAR_DEV_IOCTL_EVENT_RESET;
		}
		break;

	case CG2900_CHAR_DEV_IOCTL_GET_REVISION:
		CG2900_INFO("ioctl check for local revision info");
		if (cg2900_get_local_revision(&rev_data)) {
			CG2900_DBG("Read revision data revision %d "
				   "sub_version %d",
				   rev_data.revision, rev_data.sub_version);
			err = rev_data.revision;
		} else {
			CG2900_DBG("No revision data available");
			err = -EIO;
		}
		break;

	case CG2900_CHAR_DEV_IOCTL_GET_SUB_VER:
		CG2900_INFO("ioctl check for local sub-version info");
		if (cg2900_get_local_revision(&rev_data)) {
			CG2900_DBG("Read revision data revision %d "
				   "sub_version %d",
				   rev_data.revision, rev_data.sub_version);
			err = rev_data.sub_version;
		} else {
			CG2900_DBG("No revision data available");
			err = -EIO;
		}
		break;

	default:
		CG2900_ERR("Unknown ioctl command %08X", cmd);
		err = -EINVAL;
		break;
	};

error_handling:
	return err;
}

/**
 * char_dev_poll() - Handle POLL call to the interface.
 * @filp:	Pointer to the file struct.
 * @wait:	Poll table supplied to caller.
 *
 * Returns:
 *   Mask of current set POLL values
 */
static unsigned int char_dev_poll(struct file *filp, poll_table *wait)
{
	struct char_dev_user *dev = (struct char_dev_user *)filp->private_data;
	unsigned int mask = 0;

	if (!dev) {
		CG2900_DBG("Device not open");
		return POLLERR | POLLRDHUP;
	}

	poll_wait(filp, &dev->reset_wait_queue, wait);
	poll_wait(filp, &dev->rx_wait_queue, wait);

	if (!dev->dev)
		mask |= POLLERR | POLLRDHUP;
	else
		mask |= POLLOUT; /* We can TX unless there is an error */

	if (!(skb_queue_empty(&dev->rx_queue)))
		mask |= POLLIN | POLLRDNORM;

	if (CG2900_CHAR_RESET == dev->reset_state)
		mask |= POLLPRI;

	return mask;
}

/*
 * struct char_dev_fops - Char devices file operations.
 * @read:		Function that reads from the char device.
 * @write:		Function that writes to the char device.
 * @unlocked_ioctl:	Function that performs IO operations with
 *			the char device.
 * @poll:		Function that checks if there are possible operations
 *			with the char device.
 * @open:		Function that opens the char device.
 * @release:		Function that release the char device.
 */
static const struct file_operations char_dev_fops = {
	.read		= char_dev_read,
	.write		= char_dev_write,
	.unlocked_ioctl	= char_dev_unlocked_ioctl,
	.poll		= char_dev_poll,
	.open		= char_dev_open,
	.release	= char_dev_release
};

/**
 * setup_dev() - Set up the char device structure for device.
 * @parent:	Parent device pointer.
 * @name:	Name of registered device.
 *
 * The setup_dev() function sets up the char_dev structure for this device.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL if NULL pointer has been supplied.
 *   Error codes from cdev_add and device_create.
 */
static int setup_dev(struct device *parent, char *name)
{
	int err = 0;
	struct char_dev_user *dev_usr;

	CG2900_INFO(NAME "setup_dev");

	dev_usr = kzalloc(sizeof(*dev_usr), GFP_KERNEL);
	if (!dev_usr) {
		CG2900_ERR("Couldn't allocate dev_usr");
		return -ENOMEM;
	}

	/* Store device name */
	dev_usr->name = name;

	dev_usr->miscdev = kzalloc(sizeof(*(dev_usr->miscdev)),
				       GFP_KERNEL);
	if (!dev_usr->miscdev) {
		CG2900_ERR("Couldn't allocate char_dev");
		err = -ENOMEM;
		goto err_free_usr;
	}

	/* Prepare miscdevice struct before registering the device */
	dev_usr->miscdev->minor = MISC_DYNAMIC_MINOR;
	dev_usr->miscdev->name = name;
	dev_usr->miscdev->fops = &char_dev_fops;
	dev_usr->miscdev->parent = parent;

	err = misc_register(dev_usr->miscdev);
	if (err) {
		CG2900_ERR("Error %d registering misc dev!", err);
		goto err_free_dev;
	}

	CG2900_INFO("Added char device %s with major 0x%X and minor 0x%X",
		    name, MAJOR(dev_usr->miscdev->this_device->devt),
		    MINOR(dev_usr->miscdev->this_device->devt));

	mutex_init(&dev_usr->read_mutex);
	mutex_init(&dev_usr->write_mutex);

	skb_queue_head_init(&dev_usr->rx_queue);

	list_add_tail(&dev_usr->list, &char_info->dev_users);
	return 0;

err_free_dev:
	kfree(dev_usr->miscdev);
	dev_usr->miscdev = NULL;
err_free_usr:
	kfree(dev_usr);
	return err;
}

/**
 * remove_dev() - Remove char device structure for device.
 * @dev_usr:	Char device user.
 *
 * The remove_dev() function releases the char_dev structure for this device.
 */
static void remove_dev(struct char_dev_user *dev_usr)
{
	CG2900_INFO(NAME "remove_dev");

	if (!dev_usr)
		return;

	skb_queue_purge(&dev_usr->rx_queue);

	mutex_destroy(&dev_usr->read_mutex);
	mutex_destroy(&dev_usr->write_mutex);

	/* Remove device node in file system. */
	misc_deregister(dev_usr->miscdev);
	kfree(dev_usr->miscdev);
	dev_usr->miscdev = NULL;

	kfree(dev_usr);
}

/**
 * cg2900_char_probe() - Initialize char device module.
 * @pdev:	Platform device.
 *
 * Returns:
 *   0 if success.
 *   -ENOMEM if allocation fails.
 *   -EACCES if device already have been initiated.
 */
static int __init cg2900_char_probe(struct platform_device *pdev)
{
	struct device *parent;

	CG2900_INFO("cg2900_char_probe");

	if (char_info) {
		CG2900_ERR("Char devices already initiated");
		return -EACCES;
	}

	parent = pdev->dev.parent;

	/* Initialize private data. */
	char_info = kzalloc(sizeof(*char_info), GFP_ATOMIC);
	if (!char_info) {
		CG2900_ERR("Could not alloc char_info struct.");
		return -ENOMEM;
	}

	mutex_init(&char_info->open_mutex);
	INIT_LIST_HEAD(&char_info->dev_users);

	setup_dev(parent, CG2900_BT_CMD);
	setup_dev(parent, CG2900_BT_ACL);
	setup_dev(parent, CG2900_BT_EVT);
	setup_dev(parent, CG2900_FM_RADIO);
	setup_dev(parent, CG2900_GNSS);
	setup_dev(parent, CG2900_DEBUG);
	setup_dev(parent, CG2900_STE_TOOLS);
	setup_dev(parent, CG2900_HCI_LOGGER);
	setup_dev(parent, CG2900_US_CTRL);
	setup_dev(parent, CG2900_BT_AUDIO);
	setup_dev(parent, CG2900_FM_RADIO_AUDIO);
	setup_dev(parent, CG2900_CORE);

	return 0;
}

/**
 * cg2900_char_remove() - Release the char device module.
 * @pdev:	Platform device.
 *
 * Returns:
 *   0 if success (always success).
 */
static int __exit cg2900_char_remove(struct platform_device *pdev)
{
	struct list_head *cursor, *next;
	struct char_dev_user *tmp;

	CG2900_INFO("cg2900_char_remove");

	if (!char_info)
		return 0;

	list_for_each_safe(cursor, next, &char_info->dev_users) {
		tmp = list_entry(cursor, struct char_dev_user, list);
		list_del(cursor);
		remove_dev(tmp);
	}

	mutex_destroy(&char_info->open_mutex);

	kfree(char_info);
	char_info = NULL;
	return 0;
}

static struct platform_driver cg2900_char_driver = {
	.driver = {
		.name	= "cg2900-chardev",
		.owner	= THIS_MODULE,
	},
	.probe	= cg2900_char_probe,
	.remove	= __exit_p(cg2900_char_remove),
};

/**
 * cg2900_char_init() - Initialize module.
 *
 * Registers platform driver.
 */
static int __init cg2900_char_init(void)
{
	CG2900_INFO("cg2900_char_init");
	return platform_driver_register(&cg2900_char_driver);
}

/**
 * cg2900_char_exit() - Remove module.
 *
 * Unregisters platform driver.
 */
static void __exit cg2900_char_exit(void)
{
	CG2900_INFO("cg2900_char_exit");
	platform_driver_unregister(&cg2900_char_driver);
}

module_init(cg2900_char_init);
module_exit(cg2900_char_exit);

MODULE_AUTHOR("Henrik Possung ST-Ericsson");
MODULE_AUTHOR("Par-Gunnar Hjalmdahl ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ST-Ericsson CG2900 Char Devices Driver");
