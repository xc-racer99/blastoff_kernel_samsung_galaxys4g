/*
 * drivers/mfd/cg2900/cg2900_uart.c
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
 * Linux Bluetooth UART Driver for ST-Ericsson CG2900 connectivity controller.
 */

#include <asm/byteorder.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/mfd/cg2900.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>

#include "cg2900_chip.h"
#include "cg2900_core.h"
#include "cg2900_debug.h"
#include "hci_defines.h"

/* Workqueues' names */
#define UART_WQ_NAME		"cg2900_uart_wq"
#define UART_NAME		"cg2900_uart"

/* Standardized Bluetooth command channels */
#define HCI_BT_CMD_H4_CHANNEL	0x01
#define HCI_BT_ACL_H4_CHANNEL	0x02
#define HCI_BT_EVT_H4_CHANNEL	0x04

/* H4 channels specific for CG2900 */
#define HCI_FM_RADIO_H4_CHANNEL	0x08
#define HCI_GNSS_H4_CHANNEL	0x09

/* Timers used in milliseconds */
#define UART_TX_TIMEOUT		100
#define UART_RESP_TIMEOUT	1000

/* State-setting defines */
#define SET_BAUD_STATE(__new_state) \
	CG2900_SET_STATE("baud_rate_state", uart_info->baud_rate_state, \
			 __new_state)
#define SET_SLEEP_STATE(__new_state) \
	CG2900_SET_STATE("sleep_state", uart_info->sleep_state, __new_state)

/* Number of bytes to reserve at start of sk_buffer when receiving packet */
#define RX_SKB_RESERVE		8
/* Max size of received packet (not including reserved bytes) */
#define RX_SKB_MAX_SIZE		1024

/* Max size of bytes we can receive on the UART */
#define UART_RECEIVE_ROOM	65536

/* Size of the header in the different packets */
#define HCI_BT_EVT_HDR_SIZE	2
#define HCI_BT_ACL_HDR_SIZE	4
#define HCI_FM_RADIO_HDR_SIZE	1
#define HCI_GNSS_HDR_SIZE	3

/* Position of length field in the different packets */
#define HCI_EVT_LEN_POS		2
#define HCI_ACL_LEN_POS		3
#define FM_RADIO_LEN_POS	1
#define GNSS_LEN_POS		2

/* Bytes in the command Hci_Cmd_ST_Set_Uart_Baud_Rate */
#define SET_BAUD_RATE_LSB	0x09
#define SET_BAUD_RATE_MSB	0xFC
#define SET_BAUD_RATE_PAYL_LEN	0x01
#define SET_BAUD_RATE_LEN	0x04
#define BAUD_RATE_57600		0x03
#define BAUD_RATE_115200	0x02
#define BAUD_RATE_230400	0x01
#define BAUD_RATE_460800	0x00
#define BAUD_RATE_921600	0x20
#define BAUD_RATE_2000000	0x25
#define BAUD_RATE_3000000	0x27
#define BAUD_RATE_4000000	0x2B

/* Baud rate defines */
#define ZERO_BAUD_RATE		0
#define DEFAULT_BAUD_RATE	115200
#define HIGH_BAUD_RATE		3000000

/* HCI TTY line discipline value */
#ifndef N_HCI
#define N_HCI			15
#endif

/* IOCTLs for UART */
#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)
#define HCIUARTSETFD		_IOW('U', 203, int)


/* UART break control parameters */
#define TTY_BREAK_ON		(-1)
#define TTY_BREAK_OFF		(0)

/**
 * enum uart_rx_state - UART RX-state for UART.
 * @W4_PACKET_TYPE:	Waiting for packet type.
 * @W4_EVENT_HDR:	Waiting for BT event header.
 * @W4_ACL_HDR:		Waiting for BT ACL header.
 * @W4_FM_RADIO_HDR:	Waiting for FM header.
 * @W4_GNSS_HDR:	Waiting for GNSS header.
 * @W4_DATA:		Waiting for data in rest of the packet (after header).
 */
enum uart_rx_state {
	W4_PACKET_TYPE,
	W4_EVENT_HDR,
	W4_ACL_HDR,
	W4_FM_RADIO_HDR,
	W4_GNSS_HDR,
	W4_DATA
};

/**
  * enum sleep_state - Sleep-state for UART.
  * @CHIP_AWAKE:  Chip is awake.
  * @CHIP_FALLING_ASLEEP:  Chip is falling asleep.
  * @CHIP_ASLEEP: Chip is asleep.
  */
enum sleep_state {
	CHIP_AWAKE,
	CHIP_FALLING_ASLEEP,
	CHIP_ASLEEP
};

/**
 * enum baud_rate_change_state - Baud rate-state for UART.
 * @BAUD_IDLE:		No baud rate change is ongoing.
 * @BAUD_SENDING_RESET:	HCI reset has been sent. Waiting for command complete
 *			event.
 * @BAUD_START:		Set baud rate cmd scheduled for sending.
 * @BAUD_SENDING:	Set baud rate cmd sending in progress.
 * @BAUD_WAITING:	Set baud rate cmd sent, waiting for command complete
 *			event.
 * @BAUD_SUCCESS:	Baud rate change has succeeded.
 * @BAUD_FAIL:		Baud rate change has failed.
 */
enum baud_rate_change_state {
	BAUD_IDLE,
	BAUD_SENDING_RESET,
	BAUD_START,
	BAUD_SENDING,
	BAUD_WAITING,
	BAUD_SUCCESS,
	BAUD_FAIL
};

/**
 * struct uart_work_struct - Work structure for UART module.
 * @work:	Work structure.
 * @data:	Pointer to private data.
 *
 * This structure is used to pack work for work queue.
 */
struct uart_work_struct{
	struct work_struct	work;
	void			*data;
};

/**
 * struct test_char_dev_info - Main UART info structure.
 * @wq:			UART work queue.
 * @tx_queue:		TX queue for sending data to chip.
 * @tty:		TTY info structure.
 * @rx_lock:		RX spin lock.
 * @rx_state:		Current RX state.
 * @rx_count:		Number of bytes left to receive.
 * @rx_skb:		SK_buffer to store the received data into.
 * @tx_mutex:		TX mutex.
 * @baud_rate_state:	UART baud rate change state.
 * @baud_rate:		Current baud rate setting.
 * @sleep_state:	UART sleep state.
 * @timer:		UART timer (for chip sleep).
 * @fd:			File object to device.
 * @sleep_state_lock:	Used to protect chip state.
 * @sleep_allowed:	Indicate if tty has functions needed for sleep mode.
 */
struct uart_info {
	struct workqueue_struct		*wq;
	struct sk_buff_head		tx_queue;
	struct tty_struct		*tty;
	spinlock_t			rx_lock;
	enum uart_rx_state		rx_state;
	unsigned long			rx_count;
	struct sk_buff			*rx_skb;
	struct mutex			tx_mutex;
	enum baud_rate_change_state	baud_rate_state;
	int				baud_rate;
	enum sleep_state		sleep_state;
	struct timer_list		timer;
	struct file			*fd;
	struct mutex			sleep_state_lock;
	bool				sleep_allowed;
	struct device			*dev;
};

static struct uart_info *uart_info;

/* Module parameters */
static int uart_default_baud = DEFAULT_BAUD_RATE;
static int uart_high_baud = HIGH_BAUD_RATE;

static DECLARE_WAIT_QUEUE_HEAD(uart_wait_queue);

static void update_timer(void);

/**
 * is_chip_flow_off() - Check if chip has set flow off.
 * @tty:	Pointer to tty.
 *
 * Returns:
 *   true - chip flows off.
 *   false - chip flows on.
 */
static bool is_chip_flow_off(struct tty_struct *tty)
{
	int lines;

	lines = tty->ops->tiocmget(tty, uart_info->fd);

	if (lines & TIOCM_CTS)
		return false;
	else
		return true;
}

/**
 * create_work_item() - Create work item and add it to the work queue.
 * @wq:		work queue struct where the work will be added.
 * @work_func:	Work function.
 * @data:	Private data for the work.
 *
 * Returns:
 *   0 if there is no error.
 *   -EBUSY if not possible to queue work.
 *   -ENOMEM if allocation fails.
 */
static int create_work_item(struct workqueue_struct *wq, work_func_t work_func,
			    void *data)
{
	struct uart_work_struct *new_work;
	int err;

	new_work = kmalloc(sizeof(*new_work), GFP_ATOMIC);
	if (!new_work) {
		CG2900_ERR("Failed to alloc memory for uart_work_struct!");
		return -ENOMEM;
	}

	new_work->data = data;
	INIT_WORK(&new_work->work, work_func);

	err = queue_work(wq, &new_work->work);
	if (!err) {
		CG2900_ERR("Failed to queue work_struct because it's already "
			   "in the queue!");
		kfree(new_work);
		return -EBUSY;
	}

	return 0;
}

/**
 * set_tty_baud() - Called to set specific baud in TTY.
 * @tty:	Tty device.
 * @baud:	Baud to set.
 *
 * Returns:
 *   true - baudrate set with success.
 *   false - baundrate set failure.
 */
static bool set_tty_baud(struct tty_struct *tty, int baud)
{
	struct ktermios *old_termios;
	bool retval = true;

	old_termios = kmalloc(sizeof(*old_termios), GFP_ATOMIC);
	if (!old_termios) {
		CG2900_ERR("Could not allocate termios");
		return false;
	}

	mutex_lock(&(tty->termios_mutex));
	/* Start by storing the old termios. */
	memcpy(old_termios, tty->termios, sizeof(*old_termios));

	/* Let's mark that CG2900 driver uses c_ispeed and c_ospeed fields. */
	tty->termios->c_cflag |= BOTHER;

	tty_encode_baud_rate(tty, baud, baud);

	/* Finally inform the driver */
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, old_termios);
	else {
		CG2900_ERR("Can not set new baudrate.");
		/* Copy back the old termios to restore old setting. */
		memcpy(tty->termios, old_termios, sizeof(*old_termios));
		retval = false;
	}

	tty->termios->c_cflag &= ~BOTHER;

	mutex_unlock(&(tty->termios_mutex));
	kfree(old_termios);

	return retval;
}

/**
 * handle_cts_irq() - Called to handle CTS interrupt in work context.
 * @work:	work which needs to be done.
 *
 * The handle_cts_irq() function is a work handler called if interrupt on CTS
 * occurred. It updates the sleep timer which will wake up the transport.
 */
static void handle_cts_irq(struct work_struct *work)
{
	/* Restart timer and disable interrupt. */
	update_timer();
}

/**
 * cts_interrupt() - Called to handle CTS interrupt.
 * @irq:	Interrupt that occurred.
 * @dev_id:	Device ID where interrupt occurred (not used).
 *
 * The handle_cts_irq() function is called if interrupt on CTS occurred.
 * It disables the interrupt and starts a new work thread to handle
 * the interrupt.
 */
static irqreturn_t cts_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(irq);

	/* Create work and leave IRQ context. */
	(void)create_work_item(uart_info->wq, handle_cts_irq, NULL);

	return IRQ_HANDLED;
}

/**
 * set_cts_irq() - Enable interrupt on CTS.
 *
 * Returns:
 *   0 if there is no error.
 *   Error codes from request_irq and disable_uart.
 */
static int set_cts_irq(void)
{
	int err;
	struct cg2900_platform_data *pf_data;

	pf_data = (struct cg2900_platform_data *)uart_info->dev->platform_data;

	/* First disable the UART so we can use IRQ on the GPIOs */
	if (pf_data->uart.disable_uart) {
		err = pf_data->uart.disable_uart();
		if (err) {
			CG2900_ERR("Could not disable UART (%d)", err);
			goto error;
		}
	}

	/* Set IRQ on CTS. */
	err = request_irq(pf_data->uart.cts_irq,
			  cts_interrupt,
			  IRQF_TRIGGER_FALLING,
			  UART_NAME,
			  NULL);
	if (err) {
		CG2900_ERR("Could not request CTS IRQ (%d)", err);
		goto error;
	}

	return 0;

error:
	if (pf_data->uart.enable_uart)
		(void)pf_data->uart.enable_uart();
	return err;
}

/**
 * unset_cts_irq() - Disable interrupt on CTS.
 */
static void unset_cts_irq(void)
{
	int err = 0;
	struct cg2900_platform_data *pf_data;

	pf_data = (struct cg2900_platform_data *)uart_info->dev->platform_data;

	/* Free CTS interrupt and restore UART settings. */
	free_irq(pf_data->uart.cts_irq, NULL);

	if (pf_data->uart.enable_uart) {
		err = pf_data->uart.enable_uart();
		if (err)
			CG2900_ERR("Unable to enable UART Hardware (%d)", err);
	}
}

/**
 * update_timer() - Updates or starts the sleep timer.
 *
 * Updates or starts the sleep timer used to detect when there are no current
 * data transmissions.
 */
static void update_timer(void)
{
	unsigned long timeout_jiffies = cg2900_get_sleep_timeout();
	struct tty_struct *tty;

	if (!timeout_jiffies || !uart_info->fd || !uart_info->sleep_allowed)
		return;

	mutex_lock(&(uart_info->sleep_state_lock));
	/*
	 * This function indicates data is transmitted.
	 * Therefore see to that the chip is awake.
	 */
	if (CHIP_AWAKE == uart_info->sleep_state)
		goto finished;

	tty = uart_info->tty;

	if (CHIP_ASLEEP == uart_info->sleep_state) {
		/* Disable IRQ only when it was enabled. */
		unset_cts_irq();
		(void)set_tty_baud(tty, uart_info->baud_rate);
	}
	/* Set FLOW on. */
	tty_unthrottle(tty);

	/* Unset BREAK. */
	CG2900_DBG("Clear break");
	tty->ops->break_ctl(tty, TTY_BREAK_OFF);

	SET_SLEEP_STATE(CHIP_AWAKE);

finished:
	mutex_unlock(&(uart_info->sleep_state_lock));
	/*
	 * If timer is running restart it. If not, start it.
	 * All this is handled by mod_timer().
	 */
	mod_timer(&(uart_info->timer), jiffies + timeout_jiffies);
}

/**
 * sleep_timer_expired() - Called when sleep timer expires.
 * @data:	Value supplied when starting the timer.
 *
 * The sleep_timer_expired() function is called if there are no ongoing data
 * transmissions. It tries to put the chip in sleep mode.
 *
 */
static void sleep_timer_expired(unsigned long data)
{
	unsigned long timeout_jiffies = cg2900_get_sleep_timeout();
	struct tty_struct *tty;

	if (!timeout_jiffies || !uart_info->sleep_allowed || !uart_info->fd)
		return;

	mutex_lock(&(uart_info->sleep_state_lock));

	tty = uart_info->tty;

	switch (uart_info->sleep_state) {
	case CHIP_FALLING_ASLEEP:
		if (!is_chip_flow_off(tty))
			goto run_timer;

		/* Flow OFF. */
		tty_throttle(tty);

		/*
		 * Set baud zero.
		 * This cause shut off UART clock as well.
		 */
		(void)set_tty_baud(tty, ZERO_BAUD_RATE);

		if (set_cts_irq() < 0) {
			CG2900_ERR("Can not set interrupt on CTS.");
			(void)set_tty_baud(tty, uart_info->baud_rate);
			tty_unthrottle(tty);
			SET_SLEEP_STATE(CHIP_AWAKE);
			goto error;
		}

		SET_SLEEP_STATE(CHIP_ASLEEP);
		break;
	case CHIP_AWAKE:

		CG2900_DBG("Set break");
		tty->ops->break_ctl(tty, TTY_BREAK_ON);

		SET_SLEEP_STATE(CHIP_FALLING_ASLEEP);
		goto run_timer;

	case CHIP_ASLEEP: /* Fallthrough. */
	default:
		CG2900_ERR("Chip already sleeps.");
		break;
	}

	mutex_unlock(&(uart_info->sleep_state_lock));

	return;

run_timer:
	mutex_unlock(&(uart_info->sleep_state_lock));
	mod_timer(&(uart_info->timer), jiffies + timeout_jiffies);
	return;
error:
	/* Disable sleep mode.*/
	CG2900_ERR("Disable sleep mode.");
	uart_info->sleep_allowed = false;
	uart_info->fd = NULL;
	mutex_unlock(&(uart_info->sleep_state_lock));
}

/**
 * is_set_baud_rate_cmd() - Checks if data contains set baud rate hci cmd.
 * @data:	Pointer to data array to check.
 *
 * Returns:
 *   true - if cmd found;
 *   false - otherwise.
 */
static bool is_set_baud_rate_cmd(const char *data)
{
	bool cmd_match = false;

	if ((data[0] == HCI_BT_CMD_H4_CHANNEL) &&
	    (data[1] == SET_BAUD_RATE_LSB) &&
	    (data[2] == SET_BAUD_RATE_MSB) &&
	    (data[3] == SET_BAUD_RATE_PAYL_LEN)) {
		cmd_match = true;
	}
	return cmd_match;
}

/**
 * is_bt_cmd_complete_no_param() - Checks if data contains command complete event for a certain command.
 * @skb:	sk_buffer containing the data including H:4 header.
 * @cmd_lsb:	Command LSB.
 * @cmd_msb:	Command MSB.
 *
 * Returns:
 *   true - If this is the command complete we were looking for;
 *   false - otherwise.
 */
static bool is_bt_cmd_complete_no_param(struct sk_buff *skb, u8 cmd_lsb,
					u8 cmd_msb)
{
	if ((HCI_BT_EVT_H4_CHANNEL == skb->data[0]) &&
	    (HCI_BT_EVT_CMD_COMPLETE == skb->data[1]) &&
	    (HCI_BT_CMD_COMPLETE_NO_PARAM_LEN == skb->data[2]) &&
	    (cmd_lsb == skb->data[4]) &&
	    (cmd_msb == skb->data[5]))
		return true;

	return false;
}

/**
 * alloc_rx_skb() - Alloc an sk_buff structure for receiving data from controller.
 * @size:	Size in number of octets.
 * @priority:	Allocation priority, e.g. GFP_KERNEL.
 *
 * Returns:
 *   Pointer to sk_buff structure.
 */
static struct sk_buff *alloc_rx_skb(unsigned int size, gfp_t priority)
{
	struct sk_buff *skb;

	/* Allocate the SKB and reserve space for the header */
	skb = alloc_skb(size + RX_SKB_RESERVE, priority);
	if (skb)
		skb_reserve(skb, RX_SKB_RESERVE);

	return skb;
}

/**
 * finish_setting_baud_rate() - Handles sending the ste baud rate hci cmd.
 * @tty:	Pointer to a tty_struct used to communicate with tty driver.
 *
 * finish_setting_baud_rate() makes sure that the set baud rate cmd has
 * been really sent out on the wire and then switches the tty driver to new
 * baud rate.
 */
static void finish_setting_baud_rate(struct tty_struct *tty)
{
	/*
	 * Give the tty driver time to send data and proceed. If it hasn't
	 * been sent we can't do much about it anyway.
	 */
	schedule_timeout_interruptible(msecs_to_jiffies(UART_TX_TIMEOUT));

	/*
	 * Now set the termios struct to the new baudrate. Start by storing
	 * the old termios.
	 */
	if (set_tty_baud(tty, uart_info->baud_rate)) {
		CG2900_DBG("Setting termios to new baud rate");
		SET_BAUD_STATE(BAUD_WAITING);
	} else
		SET_BAUD_STATE(BAUD_IDLE);

	tty_unthrottle(tty);
}

/**
 * alloc_set_baud_rate_cmd() - Allocates new sk_buff and fills in the change baud rate hci cmd.
 * @baud:	(in/out) Requested new baud rate. Updated to default baud rate
 *		upon invalid value.
 *
 * Returns:
 *   Pointer to allocated sk_buff if successful;
 *   NULL otherwise.
 */
static struct sk_buff *alloc_set_baud_rate_cmd(int *baud)
{
	struct sk_buff *skb;
	u8 data[SET_BAUD_RATE_LEN];
	u8 *h4;

	skb = cg2900_alloc_skb(SET_BAUD_RATE_LEN, GFP_ATOMIC);
	if (!skb) {
		CG2900_ERR("Failed to alloc skb!");
		return NULL;
	}

	/* Create the Hci_Cmd_ST_Set_Uart_Baud_Rate packet */
	data[0] = SET_BAUD_RATE_LSB;
	data[1] = SET_BAUD_RATE_MSB;
	data[2] = SET_BAUD_RATE_PAYL_LEN;

	switch (*baud) {
	case 57600:
		data[3] = BAUD_RATE_57600;
		break;
	case 115200:
		data[3] = BAUD_RATE_115200;
		break;
	case 230400:
		data[3] = BAUD_RATE_230400;
		break;
	case 460800:
		data[3] = BAUD_RATE_460800;
		break;
	case 921600:
		data[3] = BAUD_RATE_921600;
		break;
	case 2000000:
		data[3] = BAUD_RATE_2000000;
		break;
	case 3000000:
		data[3] = BAUD_RATE_3000000;
		break;
	case 4000000:
		data[3] = BAUD_RATE_4000000;
		break;
	default:
		CG2900_ERR("Invalid speed requested (%d), using 115200 bps "
			   "instead\n", *baud);
		data[3] = BAUD_RATE_115200;
		*baud = 115200;
		break;
	};

	memcpy(skb_put(skb, SET_BAUD_RATE_LEN), data, SET_BAUD_RATE_LEN);
	h4 = skb_push(skb, HCI_H4_SIZE);
	*h4 = HCI_BT_CMD_H4_CHANNEL;

	return skb;
}

/**
 * work_do_transmit() - Transmit data packet to connectivity controller over UART.
 * @work:	Pointer to work info structure. Contains uart_info structure
 *		pointer.
 */
static void work_do_transmit(struct work_struct *work)
{
	struct sk_buff *skb;
	struct tty_struct *tty;
	struct uart_work_struct *current_work;

	if (!work) {
		CG2900_ERR("work == NULL");
		return;
	}

	/* Restart timer. */
	update_timer();

	current_work = container_of(work, struct uart_work_struct, work);

	if (uart_info->tty)
		tty = uart_info->tty;
	else {
		CG2900_ERR("Important structs not allocated!");
		goto finished;
	}

	mutex_lock(&uart_info->tx_mutex);

	/* Retrieve the first packet in the queue */
	skb = skb_dequeue(&uart_info->tx_queue);
	while (skb) {
		int len;

		/*
		 * Tell TTY that there is data waiting and call the write
		 * function.
		 */
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		len = tty->ops->write(tty, skb->data, skb->len);
		CG2900_INFO("Written %d bytes to UART of %d bytes in packet",
			    len, skb->len);

		/*
		 * If it's set baud rate cmd set correct baud state and after
		 * sending is finished inform the tty driver about the new
		 * baud rate.
		 */
		if ((BAUD_START == uart_info->baud_rate_state) &&
		    (is_set_baud_rate_cmd(skb->data))) {
			CG2900_INFO("UART set baud rate cmd found.");
			SET_BAUD_STATE(BAUD_SENDING);
		}

		/* Remove the bytes written from the sk_buffer */
		skb_pull(skb, len);

		/*
		 * If there is more data in this sk_buffer, put it at the start
		 * of the list and exit the loop
		 */
		if (skb->len) {
			skb_queue_head(&uart_info->tx_queue, skb);
			break;
		}
		/*
		 * No more data in the sk_buffer. Free it and get next packet in
		 * queue.
		 * Check if set baud rate cmd is in sending progress, if so call
		 * proper function to handle that cmd since it requires special
		 * attention.
		 */
		if (BAUD_SENDING == uart_info->baud_rate_state)
			finish_setting_baud_rate(tty);

		kfree_skb(skb);
		skb = skb_dequeue(&uart_info->tx_queue);
	}

	mutex_unlock(&uart_info->tx_mutex);

finished:
	kfree(current_work);
}

/**
 * work_hw_deregistered() - Handle HW deregistered.
 * @work: Reference to work data.
 */
static void work_hw_deregistered(struct work_struct *work)
{
	struct uart_work_struct *current_work;
	int err;

	if (!work) {
		CG2900_ERR("work == NULL");
		return;
	}

	current_work = container_of(work, struct uart_work_struct, work);

	/* Purge any stored sk_buffers */
	skb_queue_purge(&uart_info->tx_queue);
	if (uart_info->rx_skb) {
		kfree_skb(uart_info->rx_skb);
		uart_info->rx_skb = NULL;
	}

	err = cg2900_deregister_trans_driver();
	if (err)
		CG2900_ERR("Could not deregister UART from Core (%d)", err);

	kfree(current_work);
}

/**
 * set_baud_rate() - Sets new baud rate for the UART.
 * @baud:	New baud rate.
 *
 * This function first sends the HCI command
 * Hci_Cmd_ST_Set_Uart_Baud_Rate. It then changes the baud rate in HW, and
 * finally it waits for the Command Complete event for the
 * Hci_Cmd_ST_Set_Uart_Baud_Rate command.
 *
 * Returns:
 *   0 if there is no error.
 *   -EALREADY if baud rate change is already in progress.
 *   -EFAULT if one or more of the UART related structs is not allocated.
 *   -ENOMEM if skb allocation has failed.
 *   -EPERM if setting the new baud rate has failed.
 *   Error codes generated by create_work_item.
 */
static int set_baud_rate(int baud)
{
	struct tty_struct *tty = NULL;
	int err = 0;
	struct sk_buff *skb;
	int old_baud_rate;

	CG2900_INFO("set_baud_rate (%d baud)", baud);

	if (uart_info->baud_rate_state != BAUD_IDLE) {
		CG2900_ERR("Trying to set new baud rate before old setting "
			   "is finished");
		return -EALREADY;
	}

	if (uart_info->tty)
		tty = uart_info->tty;
	else {
		CG2900_ERR("Important structs not allocated!");
		return -EFAULT;
	}

	tty_throttle(tty);

	/*
	 * Store old baud rate so that we can restore it if something goes
	 * wrong.
	 */
	old_baud_rate = uart_info->baud_rate;

	skb = alloc_set_baud_rate_cmd(&baud);
	if (!skb) {
		CG2900_ERR("alloc_set_baud_rate_cmd failed");
		return -ENOMEM;
	}

	SET_BAUD_STATE(BAUD_START);
	uart_info->baud_rate = baud;

	/* Queue the sk_buffer... */
	skb_queue_tail(&uart_info->tx_queue, skb);

	/* ... and call the common UART TX function */
	err = create_work_item(uart_info->wq, work_do_transmit, NULL);
	if (err) {
		CG2900_ERR("Failed to send change baud rate cmd, freeing "
			   "skb.");
		skb = skb_dequeue_tail(&uart_info->tx_queue);
		SET_BAUD_STATE(BAUD_IDLE);
		uart_info->baud_rate = old_baud_rate;
		kfree_skb(skb);
		return err;
	}

	CG2900_DBG("Set baud rate cmd scheduled for sending.");

	/*
	 * Now wait for the command complete.
	 * It will come at the new baudrate.
	 */
	wait_event_interruptible_timeout(uart_wait_queue,
				((BAUD_SUCCESS == uart_info->baud_rate_state) ||
				 (BAUD_FAIL    == uart_info->baud_rate_state)),
				 msecs_to_jiffies(UART_RESP_TIMEOUT));
	if (BAUD_SUCCESS == uart_info->baud_rate_state)
		CG2900_DBG("Baudrate changed to %d baud", baud);
	else {
		CG2900_ERR("Failed to set new baudrate (%d)",
			   uart_info->baud_rate_state);
		err = -EPERM;
	}

	/* Finally flush the TTY so we are sure that is no bad data there */
	if (tty->ops->flush_buffer) {
		CG2900_DBG("Flushing TTY after baud rate change");
		tty->ops->flush_buffer(tty);
	}

	/* Finished. Set state to IDLE */
	SET_BAUD_STATE(BAUD_IDLE);

	return err;
}

/**
 * uart_open() - Open the CG2900 UART for data transfers.
 * @dev:	Transport device information.
 *
 * Returns:
 *   0 if there is no error,
 *   -EACCES if write to transport failed,
 *   -EIO if chip did not answer to commands.
 */
static int uart_open(struct cg2900_trans_dev *dev)
{
	u8 data[HCI_BT_RESET_LEN + HCI_H4_SIZE];
	struct tty_struct *tty;
	int bytes_written;

	/*
	 * Chip has just been started up. It has a system to autodetect
	 * exact baud rate and transport to use. There are only a few commands
	 * it will recognize and HCI Reset is one of them.
	 * We therefore start with sending that before actually changing
	 * baud rate.
	 *
	 * Create the Hci_Reset packet
	 */
	data[0] = HCI_BT_CMD_H4_CHANNEL;
	data[1] = HCI_BT_RESET_CMD_LSB;
	data[2] = HCI_BT_RESET_CMD_MSB;
	data[3] = HCI_BT_RESET_PARAM_LEN;

	/* Get the TTY info and send the packet */
	tty = uart_info->tty;
	SET_BAUD_STATE(BAUD_SENDING_RESET);
	CG2900_DBG("Sending HCI reset before baud rate change");
	bytes_written = tty->ops->write(tty, data,
					HCI_BT_RESET_LEN + HCI_H4_SIZE);
	if (bytes_written != HCI_BT_RESET_LEN + HCI_H4_SIZE) {
		CG2900_ERR("Only wrote %d bytes", bytes_written);
		SET_BAUD_STATE(BAUD_IDLE);
		return -EACCES;
	}

	/*
	 * Wait for command complete. If error, exit without changing
	 * baud rate.
	 */
	wait_event_interruptible_timeout(uart_wait_queue,
					BAUD_IDLE == uart_info->baud_rate_state,
					msecs_to_jiffies(UART_RESP_TIMEOUT));
	if (BAUD_IDLE != uart_info->baud_rate_state) {
		CG2900_ERR("Failed to send HCI Reset");
		SET_BAUD_STATE(BAUD_IDLE);
		return -EIO;
	}

	/* Just return if there will be no change of baud rate */
	if (uart_default_baud != uart_high_baud)
		return set_baud_rate(uart_high_baud);
	else
		return 0;
}

/**
 * uart_set_chip_power() - Enable or disable the CG2900.
 * @chip_on:	true if chip shall be enabled, false otherwise.
 */
static void uart_set_chip_power(bool chip_on)
{
	int uart_baudrate = uart_default_baud;
	struct tty_struct *tty;
	struct cg2900_platform_data *pf_data;

	CG2900_INFO("uart_set_chip_power: %s",
		    (chip_on ? "ENABLE" : "DISABLE"));

	if (uart_info->tty)
		tty = uart_info->tty;
	else {
		CG2900_ERR("Important structs not allocated!");
		return;
	}

	pf_data = (struct cg2900_platform_data *)uart_info->dev->platform_data;

	if (chip_on) {
		if (pf_data->enable_chip)
			pf_data->enable_chip();
	} else {
		if (pf_data->disable_chip)
			pf_data->disable_chip();
		/*
		 * Setting baud rate to 0 will tell UART driver to shut off its
		 * clocks.
		 */
		uart_baudrate = ZERO_BAUD_RATE;
	}

	/*
	 * Now we have to set the digital baseband UART
	 * to default baudrate if chip is ON or to zero baudrate if
	 * chip is turning OFF.
	 */
	 (void)set_tty_baud(tty, uart_baudrate);
}

/**
 * uart_close() - Close the CG2900 UART for data transfers.
 * @dev:	Transport device information.
 *
 * Returns:
 *   0 if there is no error.
 */
static int uart_close(struct cg2900_trans_dev *dev)
{
	/* The chip is already shut down. Power off the chip. */
	uart_set_chip_power(false);

	return 0;
}

/**
 * uart_write() - Transmit data to CG2900 over UART.
 * @dev:	Transport device information.
 * @skb:	SK buffer to transmit.
 *
 * Returns:
 *   0 if there is no error.
 *   Errors from create_work_item.
 */
static int uart_write(struct cg2900_trans_dev *dev, struct sk_buff *skb)
{
	int err;

	/* Delete sleep timer. */
	(void)del_timer(&uart_info->timer);

	CG2900_DBG_DATA_CONTENT("uart_write", skb->data, skb->len);

	/* Queue the sk_buffer... */
	skb_queue_tail(&uart_info->tx_queue, skb);

	/* ...and start TX operation */
	err = create_work_item(uart_info->wq, work_do_transmit, NULL);
	if (err)
		CG2900_ERR("Failed to create work item (%d) uart_tty_wakeup",
			   err);

	return err;
}

/**
 * send_skb_to_core() - Sends packet received from UART to CG2900 Core.
 * @skb:	Received data packet.
 *
 * This function checks if UART is waiting for Command complete event,
 * see set_baud_rate.
 * If it is waiting it checks if it is the expected packet and the status.
 * If not is passes the packet to CG2900 Core.
 */
static void send_skb_to_core(struct sk_buff *skb)
{
	u8 status;

	if (!skb) {
		CG2900_ERR("Received NULL as skb");
		return;
	}

	if (BAUD_WAITING == uart_info->baud_rate_state) {
		/*
		 * Should only really be one packet received now:
		 * the CmdComplete for the SetBaudrate command
		 * Let's see if this is the packet we are waiting for.
		 */
		if (!is_bt_cmd_complete_no_param(skb, SET_BAUD_RATE_LSB,
						 SET_BAUD_RATE_MSB)) {
			/*
			 * Received other event. Should not really happen,
			 * but pass the data to CG2900 Core anyway.
			 */
			CG2900_DBG_DATA("Sending packet to CG2900 Core while "
					"waiting for CmdComplete");
			cg2900_data_from_chip(skb);
			return;
		}

		/*
		 * We have received complete event for our baud rate
		 * change command
		 */
		status = skb->data[HCI_BT_EVT_CMD_COMPL_STATUS_POS +
				   HCI_H4_SIZE];
		if (HCI_BT_ERROR_NO_ERROR == status) {
			CG2900_DBG("Received baud rate change complete "
				   "event OK");
			SET_BAUD_STATE(BAUD_SUCCESS);
		} else {
			CG2900_ERR("Received baud rate change complete event "
				   "with status 0x%X", status);
			SET_BAUD_STATE(BAUD_FAIL);
		}
		wake_up_interruptible(&uart_wait_queue);
		kfree_skb(skb);
	} else if (BAUD_SENDING_RESET == uart_info->baud_rate_state) {
		/*
		 * Should only really be one packet received now:
		 * the CmdComplete for the Reset command
		 * Let's see if this is the packet we are waiting for.
		 */
		if (!is_bt_cmd_complete_no_param(skb, HCI_BT_RESET_CMD_LSB,
						 HCI_BT_RESET_CMD_MSB)) {
			/*
			 * Received other event. Should not really happen,
			 * but pass the data to CG2900 Core anyway.
			 */
			CG2900_DBG_DATA("Sending packet to CG2900 Core while "
					"waiting for CmdComplete");
			cg2900_data_from_chip(skb);
			return;
		}

		/*
		 * We have received complete event for our baud rate
		 * change command
		 */
		status = skb->data[HCI_BT_EVT_CMD_COMPL_STATUS_POS +
				   HCI_H4_SIZE];
		if (HCI_BT_ERROR_NO_ERROR == status) {
			CG2900_DBG("Received HCI reset complete event OK");
			/*
			 * Go back to BAUD_IDLE since this was not really
			 * baud rate change but just a preparation of the chip
			 * to be ready to receive commands.
			 */
			SET_BAUD_STATE(BAUD_IDLE);
		} else {
			CG2900_ERR("Received HCI reset complete event with "
				   "status 0x%X", status);
			SET_BAUD_STATE(BAUD_FAIL);
		}
		wake_up_interruptible(&uart_wait_queue);
		kfree_skb(skb);
	} else {
		/* Just pass data to CG2900 Core */
		cg2900_data_from_chip(skb);
	}
}

static struct cg2900_trans_callbacks uart_cb = {
	.open = uart_open,
	.close = uart_close,
	.write = uart_write,
	.set_chip_power = uart_set_chip_power
};

/**
 * check_data_len() - Check number of bytes to receive.
 * @len:	Number of bytes left to receive.
 */
static void check_data_len(int len)
{
	/* First get number of bytes left in the sk_buffer */
	register int room = skb_tailroom(uart_info->rx_skb);

	if (!len) {
		/* No data left to receive. Transmit to CG2900 Core */
		send_skb_to_core(uart_info->rx_skb);
	} else if (len > room) {
		CG2900_ERR("Data length is too large (%d > %d)", len, room);
		kfree_skb(uart_info->rx_skb);
	} else {
		/*
		 * "Normal" case. Switch to data receiving state and store
		 * data length.
		 */
		uart_info->rx_state = W4_DATA;
		uart_info->rx_count = len;
		return;
	}

	uart_info->rx_state = W4_PACKET_TYPE;
	uart_info->rx_skb   = NULL;
	uart_info->rx_count = 0;
}

/**
 * uart_receive_skb() - Handles received UART data.
 * @data:	Data received
 * @count:	Number of bytes received
 *
 * The uart_receive_skb() function handles received UART data and puts it
 * together to one complete packet.
 *
 * Returns:
 *   Number of bytes not handled, i.e. 0 = no error.
 */
static int uart_receive_skb(const u8 *data, int count)
{
	const u8 *r_ptr;
	u8 *w_ptr;
	int len;
	struct hci_event_hdr	*evt;
	struct hci_acl_hdr	*acl;
	union fm_leg_evt_or_irq	*fm;
	struct gnss_hci_hdr	*gnss;
	u8 *tmp;

	r_ptr = data;
	/* Continue while there is data left to handle */
	while (count) {
		/*
		 * If we have already received a packet we know how many bytes
		 * there are left.
		 */
		if (!uart_info->rx_count)
			goto check_h4_header;

		/* First copy received data into the skb_rx */
		len = min_t(unsigned int, uart_info->rx_count, count);
		memcpy(skb_put(uart_info->rx_skb, len), r_ptr, len);
		/* Update counters from the length and step the data pointer */
		uart_info->rx_count -= len;
		count -= len;
		r_ptr += len;

		if (uart_info->rx_count)
			/*
			 * More data to receive to current packet. Break and
			 * wait for next data on the UART.
			 */
			break;

		/* Handle the different states */
		tmp = uart_info->rx_skb->data + CG2900_SKB_RESERVE;
		switch (uart_info->rx_state) {
		case W4_DATA:
			/*
			 * Whole data packet has been received.
			 * Transmit it to CG2900 Core.
			 */
			send_skb_to_core(uart_info->rx_skb);

			uart_info->rx_state = W4_PACKET_TYPE;
			uart_info->rx_skb = NULL;
			continue;

		case W4_EVENT_HDR:
			evt = (struct hci_event_hdr *)tmp;
			check_data_len(evt->plen);
			/* Header read. Continue with next bytes */
			continue;

		case W4_ACL_HDR:
			acl = (struct hci_acl_hdr *)tmp;
			check_data_len(le16_to_cpu(acl->dlen));
			/* Header read. Continue with next bytes */
			continue;

		case W4_FM_RADIO_HDR:
			fm = (union fm_leg_evt_or_irq *)tmp;
			check_data_len(fm->param_length);
			/* Header read. Continue with next bytes */
			continue;

		case W4_GNSS_HDR:
			gnss = (struct gnss_hci_hdr *)tmp;
			check_data_len(le16_to_cpu(gnss->plen));
			/* Header read. Continue with next bytes */
			continue;

		default:
			CG2900_ERR("Bad state indicating memory overwrite "
				   "(0x%X)", (u8)(uart_info->rx_state));
			break;
		}

check_h4_header:
		/* Check which H:4 packet this is and update RX states */
		if (*r_ptr == HCI_BT_EVT_H4_CHANNEL) {
			uart_info->rx_state = W4_EVENT_HDR;
			uart_info->rx_count = HCI_BT_EVT_HDR_SIZE;
		} else if (*r_ptr == HCI_BT_ACL_H4_CHANNEL) {
			uart_info->rx_state = W4_ACL_HDR;
			uart_info->rx_count = HCI_BT_ACL_HDR_SIZE;
		} else if (*r_ptr == HCI_FM_RADIO_H4_CHANNEL) {
			uart_info->rx_state = W4_FM_RADIO_HDR;
			uart_info->rx_count = HCI_FM_RADIO_HDR_SIZE;
		} else if (*r_ptr == HCI_GNSS_H4_CHANNEL) {
			uart_info->rx_state = W4_GNSS_HDR;
			uart_info->rx_count = HCI_GNSS_HDR_SIZE;
		} else {
			CG2900_ERR("Unknown HCI packet type 0x%X", (u8)*r_ptr);
			r_ptr++;
			count--;
			continue;
		}

		/*
		 * Allocate packet. We do not yet know the size and therefore
		 * allocate max size.
		 */
		uart_info->rx_skb = alloc_rx_skb(RX_SKB_MAX_SIZE, GFP_ATOMIC);
		if (!uart_info->rx_skb) {
			CG2900_ERR("Can't allocate memory for new packet");
			uart_info->rx_state = W4_PACKET_TYPE;
			uart_info->rx_count = 0;
			return 0;
		}

		/* Write the H:4 header first in the sk_buffer */
		w_ptr = skb_put(uart_info->rx_skb, 1);
		*w_ptr = *r_ptr;

		/* First byte (H4 header) read. Goto next byte */
		r_ptr++;
		count--;
	}

	return count;
}

/**
 * uart_tty_open() - Called when UART line discipline changed to N_HCI.
 * @tty:	Pointer to associated TTY instance data.
 *
 * Returns:
 *   0 if there is no error.
 *   Errors from cg2900_register_trans_driver.
 */
static int uart_tty_open(struct tty_struct *tty)
{
	int err;

	CG2900_INFO("uart_tty_open");

	/* Set the structure pointers and set the UART receive room */
	uart_info->tty = tty;
	tty->disc_data = NULL;
	tty->receive_room = UART_RECEIVE_ROOM;

	/*
	 * Flush any pending characters in the driver and line discipline.
	 * Don't use ldisc_ref here as the open path is before the ldisc is
	 * referencable.
	 */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);

	tty_driver_flush_buffer(tty);

	/* Tell CG2900 Core that UART is connected */
	err = cg2900_register_trans_driver(&uart_cb, NULL, &uart_info->dev);
	if (err)
		CG2900_ERR("Could not register transport driver (%d)", err);

	if (tty->ops->tiocmget && tty->ops->break_ctl)
		uart_info->sleep_allowed = true;
	else
		CG2900_ERR("Sleep mode not available.");

	return err;

}

/**
 * uart_tty_close() - Close UART tty.
 * @tty:	Pointer to associated TTY instance data.
 *
 * The uart_tty_close() function is called when the line discipline is changed
 * to something else, the TTY is closed, or the TTY detects a hangup.
 */
static void uart_tty_close(struct tty_struct *tty)
{
	int err;

	CG2900_INFO("uart_tty_close");

	BUG_ON(!uart_info);
	BUG_ON(!uart_info->wq);

	err = create_work_item(uart_info->wq, work_hw_deregistered, NULL);
	if (err)
		CG2900_ERR("Failed to create work item (%d) "
			   "work_hw_deregistered", err);
}

/**
 * uart_tty_wakeup() - Callback function for transmit wake up.
 * @tty:	Pointer to associated TTY instance data.
 *
 * The uart_tty_wakeup() callback function is called when low level
 * device driver can accept more send data.
 */
static void uart_tty_wakeup(struct tty_struct *tty)
{
	int err;

	CG2900_INFO("uart_tty_wakeup");

	/*
	 * Clear the TTY_DO_WRITE_WAKEUP bit that is set in
	 * work_do_transmit().
	 */
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	if (tty != uart_info->tty)
		return;

	/* Delete sleep timer. */
	(void)del_timer(&uart_info->timer);

	/* Start TX operation */
	err = create_work_item(uart_info->wq, work_do_transmit, NULL);
	if (err)
		CG2900_ERR("Failed to create work item (%d) uart_tty_wakeup",
			   err);
}

/**
 * uart_tty_receive() - Called by TTY low level driver when receive data is available.
 * @tty:	Pointer to TTY instance data
 * @data:	Pointer to received data
 * @flags:	Pointer to flags for data
 * @count:	Count of received data in bytes
 */
static void uart_tty_receive(struct tty_struct *tty, const u8 *data,
			     char *flags, int count)
{
	CG2900_INFO("uart_tty_receive");

	if (tty != uart_info->tty)
		return;

	CG2900_DBG_DATA("Received data with length = %d and first byte 0x%02X",
			count, data[0]);
	CG2900_DBG_DATA_CONTENT("uart_tty_receive", data, count);

	/* Restart data timer */
	update_timer();
	spin_lock(&uart_info->rx_lock);
	uart_receive_skb(data, count);
	spin_unlock(&uart_info->rx_lock);

}

/**
 * uart_tty_ioctl() -  Process IOCTL system call for the TTY device.
 * @tty:   Pointer to TTY instance data.
 * @file:  Pointer to open file object for device.
 * @cmd:   IOCTL command code.
 * @arg:   Argument for IOCTL call (cmd dependent).
 *
 * Returns:
 *   0 if there is no error.
 *   -EBADF if supplied TTY struct is not correct.
 *   Error codes from n_tty_iotcl_helper.
 */
static int uart_tty_ioctl(struct tty_struct *tty, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int err = 0;

	CG2900_INFO("uart_tty_ioctl cmd %d", cmd);
	CG2900_DBG("DIR: %d, TYPE: %d, NR: %d, SIZE: %d", _IOC_DIR(cmd),
		   _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd));



	switch (cmd) {
	case HCIUARTSETFD:
		/* Save file object to device. */
		if (!uart_info->fd)
			uart_info->fd = file;
		else
			CG2900_DBG("Cannot store file object to device.");
		break;
	case HCIUARTSETPROTO: /* Fallthrough */
	case HCIUARTGETPROTO:
	case HCIUARTGETDEVICE:
		/*
		 * We don't do anything special here, but we have to show we
		 * handle it.
		 */
		break;

	default:
		err = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;
	};

	return err;
}

/*
 * We don't provide read/write/poll interface for user space.
 */
static ssize_t uart_tty_read(struct tty_struct *tty, struct file *file,
			     unsigned char __user *buf, size_t nr)
{
	CG2900_INFO("uart_tty_read");
	return 0;
}

static ssize_t uart_tty_write(struct tty_struct *tty, struct file *file,
			      const unsigned char *data, size_t count)
{
	CG2900_INFO("uart_tty_write");
	return count;
}

static unsigned int uart_tty_poll(struct tty_struct *tty, struct file *filp,
				  poll_table *wait)
{
	return 0;
}

/* Generic functions */

/* The uart_ldisc structure is used when registering to the UART framework. */
static struct tty_ldisc_ops uart_ldisc = {
	.magic        = TTY_LDISC_MAGIC,
	.name         = "n_hci",
	.open         = uart_tty_open,
	.close        = uart_tty_close,
	.read         = uart_tty_read,
	.write        = uart_tty_write,
	.ioctl        = uart_tty_ioctl,
	.poll         = uart_tty_poll,
	.receive_buf  = uart_tty_receive,
	.write_wakeup = uart_tty_wakeup,
	.owner        = THIS_MODULE
};

/**
 * cg2900_uart_probe() - Initialize CG2900 UART resources.
 * @pdev:	Platform device.
 *
 * This function initializes the module and registers to the UART framework.
 *
 * Returns:
 *   0 if success.
 *   -ENOMEM for failed alloc or structure creation.
 *   -ECHILD for failed work queue creation.
 *   Error codes generated by tty_register_ldisc.
 */
static int __init cg2900_uart_probe(struct platform_device *pdev)
{
	int err = 0;

	CG2900_INFO("cg2900_uart_probe");

	uart_info = kzalloc(sizeof(*uart_info), GFP_KERNEL);
	if (!uart_info) {
		CG2900_ERR("Couldn't allocate uart_info");
		return -ENOMEM;
	}

	uart_info->sleep_state = CHIP_AWAKE;
	skb_queue_head_init(&uart_info->tx_queue);
	mutex_init(&uart_info->tx_mutex);
	spin_lock_init(&uart_info->rx_lock);
	mutex_init(&(uart_info->sleep_state_lock));

	/* Init UART TX work queue */
	uart_info->wq = create_singlethread_workqueue(UART_WQ_NAME);
	if (!uart_info->wq) {
		CG2900_ERR("Could not create workqueue");
		err = -ECHILD; /* No child processes */
		goto error_handling_wq;
	}
	init_timer(&uart_info->timer);
	uart_info->timer.function = sleep_timer_expired;
	uart_info->timer.expires  = jiffies + cg2900_get_sleep_timeout();

	/* Register the tty discipline. We will see what will be used. */
	err = tty_register_ldisc(N_HCI, &uart_ldisc);
	if (err) {
		CG2900_ERR("HCI line discipline registration failed. (0x%X)",
			   err);
		goto error_handling_register;
	}

	goto finished;

error_handling_register:
	destroy_workqueue(uart_info->wq);
error_handling_wq:
	mutex_destroy(&uart_info->tx_mutex);
	kfree(uart_info);
	uart_info = NULL;
finished:
	return err;
}

/**
 * cg2900_uart_remove() - Release CG2900 UART resources.
 * @pdev:	Platform device.
 *
 * Returns:
 *   0 if success.
 *   Error codes generated by tty_unregister_ldisc.
 */
static int __exit cg2900_uart_remove(struct platform_device *pdev)
{
	int err;

	CG2900_INFO("cg2900_uart_remove");

	/* Release tty registration of line discipline */
	err = tty_unregister_ldisc(N_HCI);
	if (err)
		CG2900_ERR("Can't unregister HCI line discipline (%d)", err);

	if (!uart_info)
		return err;

	destroy_workqueue(uart_info->wq);
	mutex_destroy(&uart_info->tx_mutex);

	kfree(uart_info);
	uart_info = NULL;
	return err;
}

static struct platform_driver cg2900_uart_driver = {
	.driver = {
		.name	= "cg2900-uart",
		.owner	= THIS_MODULE,
	},
	.probe	= cg2900_uart_probe,
	.remove	= __exit_p(cg2900_uart_remove),
};

/**
 * cg2900_uart_init() - Initialize module.
 *
 * Registers platform driver.
 */
static int __init cg2900_uart_init(void)
{
	CG2900_INFO("cg2900_uart_init");
	return platform_driver_register(&cg2900_uart_driver);
}

/**
 * cg2900_uart_exit() - Remove module.
 *
 * Unregisters platform driver.
 */
static void __exit cg2900_uart_exit(void)
{
	CG2900_INFO("cg2900_uart_exit");
	platform_driver_unregister(&cg2900_uart_driver);
}

module_init(cg2900_uart_init);
module_exit(cg2900_uart_exit);

module_param(uart_default_baud, int, S_IRUGO);
MODULE_PARM_DESC(uart_default_baud,
		 "Default UART baud rate, e.g. 115200. If not set 115200 will "
		 "be used.");

module_param(uart_high_baud, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(uart_high_baud,
		 "High speed UART baud rate, e.g. 4000000.  If not set 3000000 "
		 "will be used.");

MODULE_AUTHOR("Par-Gunnar Hjalmdahl ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ST-Ericsson CG2900 UART Driver");
