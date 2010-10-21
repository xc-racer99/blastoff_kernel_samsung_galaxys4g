/*
 * drivers/mfd/cg2900/cg2900_audio.c
 *
 * Copyright (C) ST-Ericsson SA 2010
 * Authors:
 * Par-Gunnar Hjalmdahl (par-gunnar.p.hjalmdahl@stericsson.com) for ST-Ericsson.
 * Kjell Andersson (kjell.k.andersson@stericsson.com) for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Linux Bluetooth Audio Driver for ST-Ericsson CG2900 controller.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/mfd/cg2900.h>
#include <linux/mfd/cg2900_audio.h>

#include "cg2900_debug.h"
#include "hci_defines.h"
#include "cg2900_chip.h"

/* Char device op codes */
#define OP_CODE_SET_DAI_CONF			0x01
#define OP_CODE_GET_DAI_CONF			0x02
#define OP_CODE_CONFIGURE_ENDPOINT		0x03
#define OP_CODE_START_STREAM			0x04
#define OP_CODE_STOP_STREAM			0x05

/* Device names */
#define DEVICE_NAME				"cg2900_audio"

/* Type of channel used */
#define BT_CHANNEL_USED				0x00
#define FM_CHANNEL_USED				0x01

#define MAX_NBR_OF_USERS			10
#define FIRST_USER				1

#define SET_RESP_STATE(__state_var, __new_state) \
	CG2900_SET_STATE("resp_state", __state_var, __new_state)

#define DEFAULT_SCO_HANDLE			0x0008

/* Use a timeout of 5 seconds when waiting for a command response */
#define RESP_TIMEOUT				5000

/* Used to select proper API, ignoring subrevisions etc */
enum chip_revision {
	CHIP_REV_PG1,
	CHIP_REV_PG2
};

/**
 * enum chip_resp_state - State when communicating with the CG2900 controller.
 * @IDLE:		No outstanding packets to the controller.
 * @WAITING:		Packet has been sent to the controller. Waiting for
 *			response.
 * @RESP_RECEIVED:	Response from controller has been received but not yet
 *			handled.
 */
enum chip_resp_state {
	IDLE,
	WAITING,
	RESP_RECEIVED
};

/**
 * enum main_state - Main state for the CG2900 Audio driver.
 * @OPENED:	Audio driver has registered to CG2900 Core.
 * @CLOSED:	Audio driver is not registered to CG2900 Core.
 * @RESET:	A reset of CG2900 Core has occurred and no user has re-opened
 *		the audio driver.
 */
enum main_state {
	OPENED,
	CLOSED,
	RESET
};

/**
 * struct char_dev_info - CG2900 character device info structure.
 * @session:		Stored session for the char device.
 * @stored_data:	Data returned when executing last command, if any.
 * @stored_data_len:	Length of @stored_data in bytes.
 * @management_mutex:	Mutex for handling access to char dev management.
 * @rw_mutex:		Mutex for handling access to char dev writes and reads.
 */
struct char_dev_info {
	int		session;
	u8		*stored_data;
	int		stored_data_len;
	struct mutex	management_mutex;
	struct mutex	rw_mutex;
};

/**
 * struct audio_user - CG2900 audio user info structure.
 * @session:	Stored session for the char device.
 * @resp_state:	State for controller communications.
 */
struct audio_user {
	int			session;
	enum chip_resp_state	resp_state;
};

/**
 * struct endpoint_list - List for storing endpoint configuration nodes.
 * @ep_list:		Pointer to first node in list.
 * @management_mutex:	Mutex for handling access to list.
 */
struct endpoint_list {
	struct list_head	ep_list;
	struct mutex		management_mutex;
};

/**
 * struct endpoint_config_node - Node for storing endpoint configuration.
 * @list:		list_head struct.
 * @endpoint_id:	Endpoint ID.
 * @config:		Stored configuration for this endpoint.
 */
struct endpoint_config_node {
	struct list_head			list;
	enum cg2900_audio_endpoint_id		endpoint_id;
	union cg2900_endpoint_config_union	config;
};

/**
 * struct audio_info - Main CG2900 Audio driver info structure.
 * @state:			Current state of the CG2900 Audio driver.
 * @revision:			Chip revision, used to select API.
 * @dev:			The misc device created by this driver.
 * @dev_bt:			CG2900 Core device registered by this driver for
 *				the BT audio channel.
 * @dev_fm:			CG2900 Core device registered by this driver for
 *				the FM audio channel.
 * @management_mutex:		Mutex for handling access to CG2900 Audio driver
 *				management.
 * @bt_mutex:			Mutex for handling access to BT audio channel.
 * @fm_mutex:			Mutex for handling access to FM audio channel.
 * @nbr_of_users_active:	Number of sessions open in the CG2900 Audio
 *				driver.
 * @bt_queue:			Received BT events.
 * @fm_queue:			Received FM events.
 * @audio_sessions:		Pointers to currently opened sessions (maps
 *				session ID to user info).
 * @i2s_config:			DAI I2S configuration.
 * @i2s_pcm_config:		DAI PCM_I2S configuration.
 * @i2s_config_known:		@true if @i2s_config has been set,
 *				@false otherwise.
 * @i2s_pcm_config_known:	@true if @i2s_pcm_config has been set,
 *				@false otherwise.
 * @endpoints:			List containing the endpoint configurations.
 * @stream_ids:			Bitmask for in-use stream ids (only used with
 *				PG2 chip API).
 */
struct audio_info {
	enum main_state			state;
	enum chip_revision		revision;
	struct miscdevice		dev;
	struct cg2900_device		*dev_bt;
	struct cg2900_device		*dev_fm;
	struct mutex			management_mutex;
	struct mutex			bt_mutex;
	struct mutex			fm_mutex;
	int				nbr_of_users_active;
	struct sk_buff_head		bt_queue;
	struct sk_buff_head		fm_queue;
	struct audio_user		*audio_sessions[MAX_NBR_OF_USERS];
	struct cg2900_dai_conf_i2s	i2s_config;
	struct cg2900_dai_conf_i2s_pcm	i2s_pcm_config;
	bool				i2s_config_known;
	bool				i2s_pcm_config_known;
	struct endpoint_list		endpoints;
	u32				stream_ids;
};

/**
 * struct audio_cb_info - Callback info structure registered in @user_data.
 * @channel:	Stores if this device handles BT or FM events.
 * @user:	Audio user currently awaiting data on the channel.
 */
struct audio_cb_info {
	int			channel;
	struct audio_user	*user;
};

/* cg2900_audio wait queues */
static DECLARE_WAIT_QUEUE_HEAD(wq_bt);
static DECLARE_WAIT_QUEUE_HEAD(wq_fm);

static struct audio_info *audio_info;

static struct audio_cb_info cb_info_bt = {
	.channel = BT_CHANNEL_USED,
	.user = NULL
};
static struct audio_cb_info cb_info_fm = {
	.channel = FM_CHANNEL_USED,
	.user = NULL
};

/*
 *	Internal conversion functions
 *
 *	Since the CG2900 apis uses several different ways to encode the
 *	same parameter in different cases, we have to use translator
 *	functions.
 */

/**
 * session_config_sample_rate() - Convert sample rate to format used in VS_Set_SessionConfiguration.
 * @rate: Sample rate in API encoding.
 */
static u8 session_config_sample_rate(enum cg2900_endpoint_sample_rate rate)
{
	static const u8 codes[] = {
		[ENDPOINT_SAMPLE_RATE_8_KHZ]    = CG2900_BT_SESSION_RATE_8K,
		[ENDPOINT_SAMPLE_RATE_16_KHZ]   = CG2900_BT_SESSION_RATE_16K,
		[ENDPOINT_SAMPLE_RATE_44_1_KHZ] = CG2900_BT_SESSION_RATE_44_1K,
		[ENDPOINT_SAMPLE_RATE_48_KHZ]   = CG2900_BT_SESSION_RATE_48K
	};

	return codes[rate];
}

/**
 * mc_i2s_sample_rate() - Convert sample rate to format used in VS_Port_Config for I2S.
 * @rate: Sample rate in API encoding.
 */
static u8 mc_i2s_sample_rate(enum cg2900_dai_sample_rate rate)
{
	static const u8 codes[] = {
		[SAMPLE_RATE_8]    = CG2900_MC_I2S_SAMPLE_RATE_8,
		[SAMPLE_RATE_16]   = CG2900_MC_I2S_SAMPLE_RATE_16,
		[SAMPLE_RATE_44_1] = CG2900_MC_I2S_SAMPLE_RATE_44_1,
		[SAMPLE_RATE_48]   = CG2900_MC_I2S_SAMPLE_RATE_48
	};

	return codes[rate];
}

/**
 * mc_pcm_sample_rate() - Convert sample rate to format used in VS_Port_Config for PCM/I2S.
 * @rate: Sample rate in API encoding.
 */
static u8 mc_pcm_sample_rate(enum cg2900_dai_sample_rate rate)
{
	static const u8 codes[] = {
		[SAMPLE_RATE_8]    = CG2900_MC_PCM_SAMPLE_RATE_8,
		[SAMPLE_RATE_16]   = CG2900_MC_PCM_SAMPLE_RATE_16,
		[SAMPLE_RATE_44_1] = CG2900_MC_PCM_SAMPLE_RATE_44_1,
		[SAMPLE_RATE_48]   = CG2900_MC_PCM_SAMPLE_RATE_48
	};

	return codes[rate];
}

/**
 * mc_i2s_channel_select() - Convert channel selection to format used in VS_Port_Config.
 * @sel: Channel selection in API encoding.
 */
static u8 mc_i2s_channel_select(enum cg2900_dai_channel_sel sel)
{
	static const u8 codes[] = {
		[CHANNEL_SELECTION_RIGHT] = CG2900_MC_I2S_RIGHT_CHANNEL,
		[CHANNEL_SELECTION_LEFT]  = CG2900_MC_I2S_LEFT_CHANNEL,
		[CHANNEL_SELECTION_BOTH]  = CG2900_MC_I2S_BOTH_CHANNELS
	};
	return codes[sel];
}

/**
 * get_fs_duration() - Convert framesync-enumeration to real value.
 * @duration: Framsync duration (API encoding).
 *
 * Returns:
 * Duration in bits.
 */
static u16 get_fs_duration(enum cg2900_dai_fs_duration duration)
{
	static const u16 values[] = {
		[SYNC_DURATION_8] = 8,
		[SYNC_DURATION_16] = 16,
		[SYNC_DURATION_24] = 24,
		[SYNC_DURATION_32] = 32,
		[SYNC_DURATION_48] = 48,
		[SYNC_DURATION_50] = 50,
		[SYNC_DURATION_64] = 64,
		[SYNC_DURATION_75] = 75,
		[SYNC_DURATION_96] = 96,
		[SYNC_DURATION_125] = 125,
		[SYNC_DURATION_128] = 128,
		[SYNC_DURATION_150] = 150,
		[SYNC_DURATION_192] = 192,
		[SYNC_DURATION_250] = 250,
		[SYNC_DURATION_256] = 256,
		[SYNC_DURATION_300] = 300,
		[SYNC_DURATION_384] = 384,
		[SYNC_DURATION_500] = 500,
		[SYNC_DURATION_512] = 512,
		[SYNC_DURATION_600] = 600,
		[SYNC_DURATION_768] = 768
	};
	return values[duration];
}

/**
 * mc_i2s_role() - Convert master/slave encoding to format for I2S-ports.
 * @mode: Master/slave in API encoding.
 */
static u8 mc_i2s_role(enum cg2900_dai_mode mode)
{
	if (mode == DAI_MODE_SLAVE)
		return CG2900_I2S_MODE_SLAVE;
	else
		return CG2900_I2S_MODE_MASTER;
}

/**
 * mc_pcm_role() - Convert master/slave encoding to format for PCM/I2S-port.
 * @mode: Master/slave in API encoding.
 */
static u8 mc_pcm_role(enum cg2900_dai_mode mode)
{
	if (mode == DAI_MODE_SLAVE)
		return CG2900_PCM_MODE_SLAVE;
	else
		return CG2900_PCM_MODE_MASTER;
}

/**
 * fm_get_conversion() - Convert sample rate to convert up/down used in X_Set_Control FM commands.
 * @srate: Sample rate.
 */
static u16 fm_get_conversion(enum cg2900_endpoint_sample_rate srate)
{
	if (srate >= ENDPOINT_SAMPLE_RATE_44_1_KHZ)
		return CG2900_FM_CMD_SET_CTRL_CONV_UP;
	else
		return CG2900_FM_CMD_SET_CTRL_CONV_DOWN;
}

/*
 *	Internal helper functions
 */

/**
 * read_cb() - Handle data received from STE connectivity driver.
 * @dev:	Device receiving data.
 * @skb:	Buffer with data coming form device.
 */
static void read_cb(struct cg2900_device *dev, struct sk_buff *skb)
{
	struct audio_cb_info *cb_info;

	CG2900_INFO("CG2900 Audio: read_cb");

	if (!dev) {
		CG2900_ERR("NULL supplied as dev");
		return;
	}

	if (!skb) {
		CG2900_ERR("NULL supplied as skb");
		return;
	}

	cb_info = (struct audio_cb_info *)dev->user_data;
	if (!cb_info) {
		CG2900_ERR("NULL supplied as cb_info");
		return;
	}
	if (!(cb_info->user)) {
		CG2900_ERR("NULL supplied as cb_info->user");
		return;
	}

	/* Mark that packet has been received */
	SET_RESP_STATE(cb_info->user->resp_state, RESP_RECEIVED);

	/* Handle packet depending on channel */
	if (cb_info->channel == BT_CHANNEL_USED) {
		skb_queue_tail(&(audio_info->bt_queue), skb);
		wake_up_interruptible(&wq_bt);
	} else if (cb_info->channel == FM_CHANNEL_USED) {
		skb_queue_tail(&(audio_info->fm_queue), skb);
		wake_up_interruptible(&wq_fm);
	} else {
		/* Unhandled channel; free the packet */
		CG2900_ERR("Received callback on bad channel %d",
			   cb_info->channel);
		kfree_skb(skb);
	}
}

/**
 * reset_cb() - Reset callback function.
 * @dev:        CG2900_Core device resetting.
 */
static void reset_cb(struct cg2900_device *dev)
{
	CG2900_INFO("CG2900 Audio: reset_cb");
	mutex_lock(&audio_info->management_mutex);
	audio_info->nbr_of_users_active = 0;
	audio_info->state = RESET;
	mutex_unlock(&audio_info->management_mutex);
}

static struct cg2900_callbacks cg2900_cb = {
	.read_cb = read_cb,
	.reset_cb = reset_cb
};

/**
 * get_session_user() - Check that supplied session is within valid range.
 * @session:	Session ID.
 *
 * Returns:
 *   Audio_user if there is no error.
 *   NULL for bad session ID.
 */
static struct audio_user *get_session_user(int session)
{
	struct audio_user *audio_user;

	if (session < FIRST_USER || session >= MAX_NBR_OF_USERS) {
		CG2900_ERR("Calling with invalid session %d", session);
		return NULL;
	}

	audio_user = audio_info->audio_sessions[session];
	if (!audio_user)
		CG2900_ERR("Calling with non-opened session %d", session);
	return audio_user;
}

/**
 * del_endpoint_private() - Deletes an endpoint from @list.
 * @endpoint_id:	Endpoint ID.
 * @list:		List of endpoints.
 *
 * Deletes an endpoint from the supplied endpoint list.
 * This function is not protected by any semaphore.
 */
static void del_endpoint_private(enum cg2900_audio_endpoint_id endpoint_id,
				 struct endpoint_list *list)
{
	struct list_head *cursor, *next;
	struct endpoint_config_node *tmp;

	list_for_each_safe(cursor, next, &(list->ep_list)) {
		tmp = list_entry(cursor, struct endpoint_config_node, list);
		if (tmp->endpoint_id == endpoint_id) {
			list_del(cursor);
			kfree(tmp);
		}
	}
}

/**
 * add_endpoint() - Add endpoint node to @list.
 * @ep_config:	Endpoint configuration.
 * @list:	List of endpoints.
 *
 * Add endpoint node to the supplied list and copies supplied config to node.
 * If a node already exists for the supplied endpoint, the old node is removed
 * and replaced by the new node.
 */
static void add_endpoint(struct cg2900_endpoint_config *ep_config,
			 struct endpoint_list *list)
{
	struct endpoint_config_node *item;

	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (!item) {
		CG2900_ERR("Failed to alloc memory!");
		return;
	}

	/* Store values */
	item->endpoint_id = ep_config->endpoint_id;
	memcpy(&(item->config), &(ep_config->config), sizeof(item->config));

	mutex_lock(&(list->management_mutex));

	/*
	 * Check if endpoint ID already exist in list.
	 * If that is the case, remove it.
	 */
	if (!list_empty(&(list->ep_list)))
		del_endpoint_private(ep_config->endpoint_id, list);

	list_add_tail(&(item->list), &(list->ep_list));

	mutex_unlock(&(list->management_mutex));
}

/**
 * find_endpoint() - Finds endpoint identified by @endpoint_id in @list.
 * @endpoint_id:	Endpoint ID.
 * @list:		List of endpoints.
 *
 * Returns:
 *   Endpoint configuration if there is no error.
 *   NULL if no configuration can be found for @endpoint_id.
 */
static union cg2900_endpoint_config_union *
find_endpoint(enum cg2900_audio_endpoint_id endpoint_id,
	      struct endpoint_list *list)
{
	struct list_head *cursor, *next;
	struct endpoint_config_node *tmp;
	struct endpoint_config_node *ret_ep = NULL;

	mutex_lock(&list->management_mutex);
	list_for_each_safe(cursor, next, &(list->ep_list)) {
		tmp = list_entry(cursor, struct endpoint_config_node, list);
		if (tmp->endpoint_id == endpoint_id) {
			ret_ep = tmp;
			break;
		}
	}
	mutex_unlock(&list->management_mutex);

	if (ret_ep)
		return &(ret_ep->config);
	else
		return NULL;
}

/**
 * flush_endpoint_list() - Deletes all stored endpoints in @list.
 * @list:	List of endpoints.
 */
static void flush_endpoint_list(struct endpoint_list *list)
{
	struct list_head *cursor, *next;
	struct endpoint_config_node *tmp;

	mutex_lock(&list->management_mutex);
	list_for_each_safe(cursor, next, &(list->ep_list)) {
		tmp = list_entry(cursor, struct endpoint_config_node, list);
		list_del(cursor);
		kfree(tmp);
	}
	mutex_unlock(&list->management_mutex);
}

/**
 * new_stream_id() - Allocate a new stream id.
 *
 * Returns:
 *  0-127 new valid id.
 *  -ENOMEM if no id is available.
 */
static s8 new_stream_id(void)
{
	int r;

	mutex_lock(&audio_info->management_mutex);

	r = find_first_zero_bit(&audio_info->stream_ids,
				8 * sizeof(audio_info->stream_ids));

	if (r >= 8 * sizeof(audio_info->stream_ids)) {
		r = -ENOMEM;
		goto out;
	}

	audio_info->stream_ids |= (0x01u << r);

out:
	mutex_unlock(&audio_info->management_mutex);
	return r;
}

/**
 * release_stream_id() - Release a stream id.
 * @id: stream to release.
 */
static void release_stream_id(u8 id)
{
	if (id >= 8 * sizeof(audio_info->stream_ids))
		return;

	mutex_lock(&audio_info->management_mutex);
	audio_info->stream_ids &= ~(0x01u << id);
	mutex_unlock(&audio_info->management_mutex);
}

/**
 * receive_fm_write_response() - Wait for and handle the response to an FM Legacy WriteCommand request.
 * @audio_user:	Audio user to check for.
 * @command:	FM command to wait for.
 *
 * This function first waits (up to 5 seconds) for a response to an FM
 * write command and when one arrives, it checks that it is the one we
 * are waiting for and also that no error has occurred.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -EIO for other errors.
 */
static int receive_fm_write_response(struct audio_user *audio_user,
				     u16 command)
{
	int err = 0;
	struct sk_buff *skb;
	struct fm_leg_cmd_cmpl *pkt;
	u16 rsp_cmd;

	/*
	 * Wait for callback to receive command complete and then wake us up
	 * again.
	 */
	if (0 >= wait_event_interruptible_timeout(wq_fm,
			audio_user->resp_state == RESP_RECEIVED,
			msecs_to_jiffies(RESP_TIMEOUT))) {
		/* We timed out or an error occurred */
		CG2900_ERR("Error occurred while waiting for return packet.");
		return -ECOMM;
	}

	/* OK, now we should have received answer. Let's check it. */
	skb = skb_dequeue_tail(&audio_info->fm_queue);
	if (!skb) {
		CG2900_ERR("No skb in queue when it should be there");
		return -EIO;
	}

	pkt = (struct fm_leg_cmd_cmpl *)skb->data;

	/* Check if we received the correct event */
	if (pkt->opcode != CG2900_FM_GEN_ID_LEGACY) {
		CG2900_ERR("Received unknown FM packet. 0x%X %X %X %X %X",
			   skb->data[0], skb->data[1], skb->data[2],
			   skb->data[3], skb->data[4]);
		err = -EIO;
		goto error_handling_free_skb;
	}

	/* FM Legacy Command complete event */
	rsp_cmd = cg2900_get_fm_cmd_id(le16_to_cpu(pkt->response_head));

	if (pkt->fm_function != CG2900_FM_CMD_PARAM_WRITECOMMAND ||
	    rsp_cmd != command) {
		CG2900_ERR("Received unexpected packet func 0x%X cmd 0x%04X",
			   pkt->fm_function, rsp_cmd);
		err = -EIO;
		goto error_handling_free_skb;
	}

	if (pkt->cmd_status != CG2900_FM_CMD_STATUS_COMMAND_SUCCEEDED) {
		CG2900_ERR("FM Command failed (%d)", pkt->cmd_status);
		err = -EIO;
		goto error_handling_free_skb;
	}
	/* Operation succeeded. We are now done */

error_handling_free_skb:
	kfree_skb(skb);
	return err;
}

/**
 * receive_bt_cmd_complete() - Wait for and handle an BT Command Complete event.
 * @audio_user:	Audio user to check for.
 * @rsp:	Opcode of BT command to wait for.
 * @data:	Pointer to buffer if any received data should be stored (except
 *		status).
 * @data_len:	Length of @data in bytes.
 *
 * This function first waits for BT Command Complete event (up to 5 seconds)
 * and when one arrives, it checks that it is the one we are waiting for and
 * also that no error has occurred.
 * If @data is supplied it also copies received data into @data.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -EIO for other errors.
 */
static int receive_bt_cmd_complete(struct audio_user *audio_user, u16 rsp,
				   void *data, int data_len)
{
	int err = 0;
	struct sk_buff *skb;
	struct bt_cmd_cmpl_event *evt;
	u16 opcode;

	/*
	 * Wait for callback to receive command complete and then wake us up
	 * again.
	 */
	if (0 >= wait_event_interruptible_timeout(wq_bt,
					audio_user->resp_state == RESP_RECEIVED,
					msecs_to_jiffies(RESP_TIMEOUT))) {
		/* We timed out or an error occurred */
		CG2900_ERR("Error occurred while waiting for return packet.");
		return -ECOMM;
	}

	/* OK, now we should have received answer. Let's check it. */
	skb = skb_dequeue_tail(&audio_info->bt_queue);
	if (!skb) {
		CG2900_ERR("No skb in queue when it should be there");
		return -EIO;
	}

	evt = (struct bt_cmd_cmpl_event *)skb->data;
	if (evt->eventcode != HCI_BT_EVT_CMD_COMPLETE) {
		CG2900_ERR("We did not receive the event we expected (0x%X)",
			   evt->eventcode);
		err = -EIO;
		goto error_handling_free_skb;
	}

	opcode = le16_to_cpu(evt->opcode);
	if (opcode != rsp) {
		CG2900_ERR("Received cmd complete for unexpected command: "
			   "0x%04X", opcode);
		err = -EIO;
		goto error_handling_free_skb;
	}

	if (evt->status != HCI_BT_ERROR_NO_ERROR) {
		CG2900_ERR("Received command complete with err %d",
			   evt->status);
		err = -EIO;
		goto error_handling_free_skb;
	}

	/* Copy the rest of the parameters if a buffer has been supplied.
	 * The caller must have set the length correctly.
	 */
	if (data)
		memcpy(data, evt->data, data_len);

	/* Operation succeeded. We are now done */

error_handling_free_skb:
	kfree_skb(skb);
	return err;
}

/**
 * send_vs_session_ctrl() - Formats an sends a CG2900_BT_VS_SESSION_CTRL command.
 * @user:          Audio user this command belongs to.
 * @stream_handle: Handle to stream.
 * @command:       Command to execute on stream, should be one of
 *                 CG2900_BT_SESSION_START, CG2900_BT_SESSION_STOP,
 *                 CG2900_BT_SESSION_PAUSE, CG2900_BT_SESSION_RESUME.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the bt_mutex held.
 *
 * Returns:
 *  0 if there is no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_vs_session_ctrl(struct audio_user *user,
				u8 stream_handle, u8 command)
{
	int err = 0;
	struct bt_vs_session_ctrl_cmd *pkt;
	struct sk_buff *skb;

	CG2900_INFO("BT: HCI_VS_Session_Control");

	skb = cg2900_alloc_skb(sizeof(*pkt), GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		return -ENOMEM;
	}

	/* Enter data into the skb */
	pkt = (struct bt_vs_session_ctrl_cmd *) skb_put(skb, sizeof(*pkt));

	pkt->opcode  = cpu_to_le16(CG2900_BT_VS_SESSION_CTRL);
	pkt->plen    = BT_PARAM_LEN(sizeof(*pkt));
	pkt->id      = stream_handle;
	pkt->control = command; /* Start/stop etc */

	cb_info_bt.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished;
	}

	err = receive_bt_cmd_complete(user, CG2900_BT_VS_SESSION_CTRL,
				      NULL, 0);
finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * send_vs_session_config() - Formats an sends a CG2900_BT_VS_SESSION_CONFIG command.
 * @user:          Audio user this command belongs to.
 * @config_stream: Custom function for configuring the stream.
 * @priv_data:     Private data passed to @config_stream untouched.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the bt_mutex held.
 *
 * Space is allocated for one stream and a custom function is used to
 * fill in the stream configuration.
 *
 * Returns:
 *  0-255 stream handle if no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_vs_session_config(struct audio_user *user,
	void(*config_stream)(void *, struct session_config_stream *),
	void *priv_data)
{
	int err = 0;
	struct sk_buff *skb;
	struct bt_vs_session_config_cmd *pkt;
	u8 session_id;

	CG2900_INFO("BT: HCI_VS_Set_Session_Configuration");

	skb = cg2900_alloc_skb(sizeof(*pkt), GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		return -ENOMEM;
	}

	pkt = (struct bt_vs_session_config_cmd *)skb_put(skb, sizeof(*pkt));
	/* zero the packet so we don't have to set all reserved fields */
	memset(pkt, 0, sizeof(*pkt));

	/* Common parameters */
	pkt->opcode    = cpu_to_le16(CG2900_BT_VS_SET_SESSION_CONFIG);
	pkt->plen      = BT_PARAM_LEN(sizeof(*pkt));
	pkt->n_streams = 1; /* 1 stream configuration supplied */

	/* Let the custom-function fill in the rest */
	config_stream(priv_data, &pkt->stream);

	cb_info_bt.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished;
	}

	err = receive_bt_cmd_complete(user,
				      CG2900_BT_VS_SET_SESSION_CONFIG,
				      &session_id, sizeof(session_id));
	/* Return session id/stream handle if success */
	if (!err)
		err = session_id;

finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * send_fm_write_1_param() - Formats and sends an FM legacy write command with one parameter.
 * @user:    Audio user this command belongs to.
 * @command: Command.
 * @param:   Parameter for command.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the fm_mutex held.
 *
 * Returns:
 *  0 if there is no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_fm_write_1_param(struct audio_user *user,
				 u16 command, u16 param)
{
	int err = 0;
	struct sk_buff *skb;
	struct fm_leg_cmd *cmd;
	size_t len;

	/* base package + one parameter */
	len = sizeof(*cmd) + sizeof(cmd->fm_cmd.data[0]);

	skb = cg2900_alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		return -ENOMEM;
	}

	cmd = (struct fm_leg_cmd *)skb_put(skb, len);

	cmd->length      = CG2900_FM_CMD_PARAM_LEN(len);
	cmd->opcode      = CG2900_FM_GEN_ID_LEGACY;
	cmd->read_write  = CG2900_FM_CMD_LEG_PARAM_WRITE;
	cmd->fm_function = CG2900_FM_CMD_PARAM_WRITECOMMAND;
	/* one parameter - builtin assumption for this function */
	cmd->fm_cmd.head    = cpu_to_le16(cg2900_make_fm_cmd_id(command, 1));
	cmd->fm_cmd.data[0] = cpu_to_le16(param);

	cb_info_fm.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_fm, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished;
	}

	err = receive_fm_write_response(user, command);
finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * send_vs_stream_ctrl() - Formats an sends a CG2900_MC_VS_STREAM_CONTROL command.
 * @user:	Audio user this command belongs to.
 * @stream:	Stream id.
 * @command:	Start/stop etc.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the bt_mutex held.
 *
 * While the HCI command allows for multiple streams in one command,
 * this function only handles one.
 *
 * Returns:
 *  0 if there is no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_vs_stream_ctrl(struct audio_user *user, u8 stream, u8 command)
{
	int err = 0;
	struct sk_buff *skb;
	struct mc_vs_stream_ctrl_cmd *cmd;
	size_t len;
	u8 vs_err;

	/* basic length + one stream */
	len = sizeof(*cmd) + sizeof(cmd->stream[0]);

	skb = cg2900_alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		return -ENOMEM;
	}

	cmd = (struct mc_vs_stream_ctrl_cmd *)skb_put(skb, len);

	cmd->opcode  = cpu_to_le16(CG2900_MC_VS_STREAM_CONTROL);
	cmd->plen    = BT_PARAM_LEN(len);
	cmd->command = command;

	/* one stream */
	cmd->n_streams  = 1;
	cmd->stream[0] = stream;

	cb_info_bt.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished;
	}

	/* All commands in PG2 API returns one byte with extra status */
	err = receive_bt_cmd_complete(user,
				      CG2900_MC_VS_STREAM_CONTROL,
				      &vs_err, sizeof(vs_err));
	if (err)
		CG2900_DBG("VS_STREAM_CONTROL - failed with error %02x",
			   vs_err);

finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * send_vs_create_stream() - Formats an sends a CG2900_MC_VS_CREATE_STREAM command.
 * @user:	Audio user this command belongs to.
 * @inport:	Stream id.
 * @outport:	Start/stop etc.
 * @order:	Activation order.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the bt_mutex held.
 *
 * Returns:
 *  0 if there is no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_vs_create_stream(struct audio_user *user, u8 inport,
				 u8 outport, u8 order)
{
	int err = 0;
	struct sk_buff *skb;
	struct mc_vs_create_stream_cmd *cmd;
	s8 id;
	u8 vs_err;

	id = new_stream_id();
	if (id < 0) {
		CG2900_ERR("No free stream id");
		err = -EIO;
		goto finished;
	}

	skb = cg2900_alloc_skb(sizeof(*cmd), GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		err = -ENOMEM;
		goto finished_release_id;
	}

	cmd = (struct mc_vs_create_stream_cmd *)skb_put(skb, sizeof(*cmd));

	cmd->opcode  = cpu_to_le16(CG2900_MC_VS_CREATE_STREAM);
	cmd->plen    = BT_PARAM_LEN(sizeof(*cmd));
	cmd->id      = (u8)id;
	cmd->inport  = inport;
	cmd->outport = outport;
	cmd->order   = order;

	cb_info_bt.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished_release_id;
	}

	/* All commands in PG2 API returns one byte with extra status */
	err = receive_bt_cmd_complete(user,
				      CG2900_MC_VS_CREATE_STREAM,
				      &vs_err, sizeof(vs_err));
	if (err) {
		CG2900_DBG("VS_CREATE_STREAM - failed with error %02x",
			   vs_err);
		goto finished_release_id;
	}

	err = id;
	goto finished;

finished_release_id:
	release_stream_id(id);
finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * send_vs_port_cfg() - Formats an sends a CG2900_MC_VS_PORT_CONFIG command.
 * @user:	Audio user this command belongs to.
 * @port:	Port id to configure.
 * @cfg:	Pointer to specific configuration.
 * @cfglen:	Length of configuration.
 *
 * Packs and sends a command packet and waits for the response. Must
 * be called with the bt_mutex held.
 *
 * Returns:
 *  0 if there is no error.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int send_vs_port_cfg(struct audio_user *user, u8 port,
			    const void *cfg, size_t cfglen)
{
	int err = 0;
	struct sk_buff *skb;
	struct mc_vs_port_cfg_cmd *cmd;
	void *ptr;
	u8 vs_err;

	skb = cg2900_alloc_skb(sizeof(*cmd) + cfglen, GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		return -ENOMEM;
	}

	/* Fill in common part */
	cmd = (struct mc_vs_port_cfg_cmd *) skb_put(skb, sizeof(*cmd));
	cmd->opcode = cpu_to_le16(CG2900_MC_VS_PORT_CONFIG);
	cmd->plen = BT_PARAM_LEN(sizeof(*cmd) + cfglen);
	cmd->type = port;

	/* Copy specific configuration */
	ptr = skb_put(skb, cfglen);
	memcpy(ptr, cfg, cfglen);

	/* Send */
	cb_info_bt.user = user;
	SET_RESP_STATE(user->resp_state, WAITING);

	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		kfree_skb(skb);
		goto finished;
	}

	/* All commands in PG2 API returns one byte with extra status */
	err = receive_bt_cmd_complete(user, CG2900_MC_VS_PORT_CONFIG,
				      &vs_err, sizeof(vs_err));
	if (err)
		CG2900_DBG("VS_PORT_CONFIG - failed with error %02x",
			   vs_err);

finished:
	SET_RESP_STATE(user->resp_state, IDLE);
	return err;
}

/**
 * set_dai_config_pg1() - Internal implementation of @cg2900_audio_set_dai_config for PG1 hardware.
 * @audio_user:	Pointer to audio user struct.
 * @config:	Pointer to the configuration to set.
 *
 * Sets the Digital Audio Interface (DAI) configuration for PG1
 * hardware. This is and internal function and basic
 * argument-verification should have been done by the caller.
 *
 * Returns:
 *  0 if there is no error.
 *  -EACCESS if port is not supported.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int set_dai_config_pg1(struct audio_user *audio_user,
			      struct cg2900_dai_config *config)
{
	int err = 0;
	struct cg2900_dai_conf_i2s_pcm *i2s_pcm;
	struct sk_buff *skb = NULL;
	struct bt_vs_set_hw_cfg_cmd_i2s *i2s_cmd;
	struct bt_vs_set_hw_cfg_cmd_pcm *pcm_cmd;

	/*
	 * Use mutex to assure that only ONE command is sent at any time on
	 * each channel.
	 */
	mutex_lock(&audio_info->bt_mutex);

	/* Allocate the sk_buffer. The length is actually a max length since
	 * length varies depending on logical transport.
	 */
	skb = cg2900_alloc_skb(CG2900_BT_LEN_VS_SET_HARDWARE_CONFIG,
			       GFP_KERNEL);
	if (!skb) {
		CG2900_ERR("Could not allocate skb");
		err = -ENOMEM;
		goto finished_unlock_mutex;
	}

	/* Fill in hci-command according to received configuration */
	switch (config->port) {
	case PORT_0_I2S:
		i2s_cmd = (struct bt_vs_set_hw_cfg_cmd_i2s *)
			skb_put(skb, sizeof(*i2s_cmd));

		i2s_cmd->opcode = cpu_to_le16(CG2900_BT_VS_SET_HARDWARE_CONFIG);
		i2s_cmd->plen   = BT_PARAM_LEN(sizeof(*i2s_cmd));

		i2s_cmd->vp_type = PORT_PROTOCOL_I2S;
		i2s_cmd->port_id = 0x00; /* First/only I2S port */
		i2s_cmd->half_period = config->conf.i2s.half_period;

		i2s_cmd->master_slave = mc_i2s_role(config->conf.i2s.mode);

		/* Store the new configuration */
		mutex_lock(&audio_info->management_mutex);
		memcpy(&(audio_info->i2s_config), &(config->conf.i2s),
		       sizeof(config->conf.i2s));
		audio_info->i2s_config_known = true;
		mutex_unlock(&audio_info->management_mutex);
		break;

	case PORT_1_I2S_PCM:
		pcm_cmd = (struct bt_vs_set_hw_cfg_cmd_pcm *)
			skb_put(skb, sizeof(*pcm_cmd));

		pcm_cmd->opcode = cpu_to_le16(CG2900_BT_VS_SET_HARDWARE_CONFIG);
		pcm_cmd->plen   = BT_PARAM_LEN(sizeof(*pcm_cmd));

		i2s_pcm = &config->conf.i2s_pcm;

		/*
		 * PG1 chips don't support I2S over the PCM/I2S bus,
		 * and PG2 chips don't use this command
		 */
		if (i2s_pcm->protocol != PORT_PROTOCOL_PCM) {
			CG2900_ERR("I2S not supported over the PCM/I2S bus");
			err = -EACCES;
			goto error_handling_free_skb;
		}

		pcm_cmd->vp_type = PORT_PROTOCOL_PCM;
		pcm_cmd->port_id = 0x00; /* First/only PCM port */

		HWCONFIG_PCM_SET_MODE(pcm_cmd, mc_pcm_role(i2s_pcm->mode));

		HWCONFIG_PCM_SET_DIR(pcm_cmd, 0, i2s_pcm->slot_0_dir);
		HWCONFIG_PCM_SET_DIR(pcm_cmd, 1, i2s_pcm->slot_1_dir);
		HWCONFIG_PCM_SET_DIR(pcm_cmd, 2, i2s_pcm->slot_2_dir);
		HWCONFIG_PCM_SET_DIR(pcm_cmd, 3, i2s_pcm->slot_3_dir);

		pcm_cmd->bit_clock = i2s_pcm->clk;
		pcm_cmd->frame_len =
			cpu_to_le16(get_fs_duration(i2s_pcm->duration));

		/* Store the new configuration */
		mutex_lock(&audio_info->management_mutex);
		memcpy(&(audio_info->i2s_pcm_config), &(config->conf.i2s_pcm),
		       sizeof(config->conf.i2s_pcm));
		audio_info->i2s_pcm_config_known = true;
		mutex_unlock(&audio_info->management_mutex);
		break;

	default:
		CG2900_ERR("Unknown port configuration %d", config->port);
		err = -EACCES;
		goto error_handling_free_skb;
	};

	cb_info_bt.user = audio_user;
	SET_RESP_STATE(audio_user->resp_state, WAITING);

	/* Send packet to controller */
	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		goto error_handling_free_skb;
	}

	err = receive_bt_cmd_complete(audio_user,
				      CG2900_BT_VS_SET_HARDWARE_CONFIG,
				      NULL, 0);

	goto finished_unlock_mutex;

error_handling_free_skb:
	kfree_skb(skb);
finished_unlock_mutex:
	SET_RESP_STATE(audio_user->resp_state, IDLE);
	mutex_unlock(&audio_info->bt_mutex);
	return err;
}

/**
 * set_dai_config_pg2() - Internal implementation of @cg2900_audio_set_dai_config for PG2 hardware.
 * @audio_user:	Pointer to audio user struct.
 * @config:	Pointer to the configuration to set.
 *
 * Sets the Digital Audio Interface (DAI) configuration for PG2
 * hardware. This is an internal function and basic
 * argument-verification should have been done by the caller.
 *
 * Returns:
 *  0 if there is no error.
 *  -EACCESS if port is not supported.
 *  -ENOMEM if not possible to allocate packet.
 *  -ECOMM if no response was received.
 *  -EIO for other errors.
 */
static int set_dai_config_pg2(struct audio_user *audio_user,
			      struct cg2900_dai_config *config)
{
	int err = 0;
	struct cg2900_dai_conf_i2s *i2s;
	struct cg2900_dai_conf_i2s_pcm *i2s_pcm;

	struct mc_vs_port_cfg_i2s i2s_cfg;
	struct mc_vs_port_cfg_pcm_i2s pcm_cfg;

	/*
	 * Use mutex to assure that only ONE command is sent at any time on
	 * each channel.
	 */
	mutex_lock(&audio_info->bt_mutex);

	switch (config->port) {
	case PORT_0_I2S:
		i2s = &config->conf.i2s;

		memset(&i2s_cfg, 0, sizeof(i2s_cfg)); /* just to be safe  */

		/* master/slave */
		PORTCFG_I2S_SET_ROLE(i2s_cfg, mc_i2s_role(i2s->mode));

		PORTCFG_I2S_SET_HALFPERIOD(i2s_cfg, i2s->half_period);
		PORTCFG_I2S_SET_CHANNELS(i2s_cfg,
			mc_i2s_channel_select(i2s->channel_sel));
		PORTCFG_I2S_SET_SRATE(i2s_cfg,
			mc_i2s_sample_rate(i2s->sample_rate));
		switch (i2s->word_width) {
		case WORD_WIDTH_16:
			PORTCFG_I2S_SET_WORDLEN(i2s_cfg, CG2900_MC_I2S_WORD_16);
			break;
		case WORD_WIDTH_32:
			PORTCFG_I2S_SET_WORDLEN(i2s_cfg, CG2900_MC_I2S_WORD_32);
			break;
		}

		/* Store the new configuration */
		mutex_lock(&audio_info->management_mutex);
		memcpy(&(audio_info->i2s_config), &(config->conf.i2s),
		       sizeof(config->conf.i2s));
		audio_info->i2s_config_known = true;
		mutex_unlock(&audio_info->management_mutex);

		/* Send */
		err = send_vs_port_cfg(audio_user, CG2900_MC_PORT_I2S,
				       &i2s_cfg, sizeof(i2s_cfg));
		break;

	case PORT_1_I2S_PCM:
		i2s_pcm = &config->conf.i2s_pcm;

		memset(&pcm_cfg, 0, sizeof(pcm_cfg)); /* just to be safe  */

		/* master/slave */
		PORTCFG_PCM_SET_ROLE(pcm_cfg, mc_pcm_role(i2s_pcm->mode));

		/* set direction for all 4 slots */
		PORTCFG_PCM_SET_DIR(pcm_cfg, 0, i2s_pcm->slot_0_dir);
		PORTCFG_PCM_SET_DIR(pcm_cfg, 1, i2s_pcm->slot_1_dir);
		PORTCFG_PCM_SET_DIR(pcm_cfg, 2, i2s_pcm->slot_2_dir);
		PORTCFG_PCM_SET_DIR(pcm_cfg, 3, i2s_pcm->slot_3_dir);

		/* set used SCO slots, other use cases not supported atm */
		PORTCFG_PCM_SET_SCO_USED(pcm_cfg, 0, i2s_pcm->slot_0_used);
		PORTCFG_PCM_SET_SCO_USED(pcm_cfg, 1, i2s_pcm->slot_1_used);
		PORTCFG_PCM_SET_SCO_USED(pcm_cfg, 2, i2s_pcm->slot_2_used);
		PORTCFG_PCM_SET_SCO_USED(pcm_cfg, 3, i2s_pcm->slot_3_used);

		/* slot starts */
		pcm_cfg.slot_start[0] = i2s_pcm->slot_0_start;
		pcm_cfg.slot_start[1] = i2s_pcm->slot_1_start;
		pcm_cfg.slot_start[2] = i2s_pcm->slot_2_start;
		pcm_cfg.slot_start[3] = i2s_pcm->slot_3_start;

		/* audio/voice sample-rate ratio */
		PORTCFG_PCM_SET_RATIO(pcm_cfg, i2s_pcm->ratio);

		/* PCM or I2S mode */
		PORTCFG_PCM_SET_MODE(pcm_cfg, i2s_pcm->protocol);

		pcm_cfg.frame_len = i2s_pcm->duration;

		PORTCFG_PCM_SET_BITCLK(pcm_cfg, i2s_pcm->clk);
		PORTCFG_PCM_SET_SRATE(pcm_cfg,
			mc_pcm_sample_rate(i2s_pcm->sample_rate));

		/* Store the new configuration */
		mutex_lock(&audio_info->management_mutex);
		memcpy(&(audio_info->i2s_pcm_config), &(config->conf.i2s_pcm),
		       sizeof(config->conf.i2s_pcm));
		audio_info->i2s_pcm_config_known = true;
		mutex_unlock(&audio_info->management_mutex);

		/* Send */
		err = send_vs_port_cfg(audio_user, CG2900_MC_PORT_PCM_I2S,
				       &pcm_cfg, sizeof(pcm_cfg));
		break;

	default:
		CG2900_ERR("Unknown port configuration %d", config->port);
		err = -EACCES;
	};

	mutex_unlock(&audio_info->bt_mutex);
	return err;
}

/**
 * struct i2s_fm_stream_config_priv - Helper struct for stream i2s-fm streams.
 * @fm_config:	FM endpoint configuration.
 * @rx:		true for FM-RX, false for FM-TX.
 */
struct i2s_fm_stream_config_priv {
	struct cg2900_endpoint_config_fm	*fm_config;
	bool					rx;

};

/**
 * config_i2s_fm_stream() - Callback for @send_vs_session_config.
 * @_priv:	Pointer to a @i2s_fm_stream_config_priv struct.
 * @cfg:	Pointer to stream config block in command packet.
 *
 * Fills in stream configuration for I2S-FM RX/TX.
 */

static void config_i2s_fm_stream(void *_priv,
				 struct session_config_stream *cfg)
{
	struct i2s_fm_stream_config_priv *priv = _priv;
	struct session_config_vport *fm;
	struct session_config_vport *i2s;

	cfg->media_type = CG2900_BT_SESSION_MEDIA_TYPE_AUDIO;

	if (audio_info->i2s_config.channel_sel == CHANNEL_SELECTION_BOTH)
		SESSIONCFG_SET_CHANNELS(cfg, CG2900_BT_MEDIA_CONFIG_STEREO);
	else
		SESSIONCFG_SET_CHANNELS(cfg, CG2900_BT_MEDIA_CONFIG_MONO);

	SESSIONCFG_I2S_SET_SRATE(cfg,
		session_config_sample_rate(priv->fm_config->sample_rate));

	cfg->codec_type = CG2900_CODEC_TYPE_NONE;
	/* codec mode and parameters not used  */

	if (priv->rx) {
		fm  = &cfg->inport;  /* FM is input */
		i2s = &cfg->outport; /* I2S is output */
	} else {
		i2s = &cfg->inport;  /* I2S is input */
		fm  = &cfg->outport; /* FM is output */
	}

	fm->type = CG2900_BT_VP_TYPE_FM;

	i2s->type = CG2900_BT_VP_TYPE_I2S;
	i2s->i2s.index   = CG2900_BT_SESSION_I2S_INDEX_I2S;
	i2s->i2s.channel = audio_info->i2s_config.channel_sel;
}

/**
 * conn_start_i2s_to_fm_rx() - Start an audio stream connecting FM RX to I2S.
 * @audio_user:		Audio user to check for.
 * @stream_handle:	[out] Pointer where to store the stream handle.
 *
 * This function sets up an FM RX to I2S stream.
 * It does this by first setting the output mode and then the configuration of
 * the External Sample Rate Converter.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -ENOMEM upon allocation errors.
 *   -EIO for other errors.
 */
static int conn_start_i2s_to_fm_rx(struct audio_user *audio_user,
				   unsigned int *stream_handle)
{
	int err = 0;
	union cg2900_endpoint_config_union *fm_config;

	fm_config = find_endpoint(ENDPOINT_FM_RX,
				  &(audio_info->endpoints));
	if (!fm_config) {
		CG2900_ERR("FM RX not configured before stream start");
		return -EIO;
	}

	if (!(audio_info->i2s_config_known)) {
		CG2900_ERR("I2S DAI not configured before stream start");
		return -EIO;
	}

	/*
	 * Use mutex to assure that only ONE command is sent at any
	 * time on each channel.
	 */
	mutex_lock(&audio_info->fm_mutex);
	mutex_lock(&audio_info->bt_mutex);

	/*
	 * Now set the output mode of the External Sample Rate Converter by
	 * sending HCI_Write command with AUP_EXT_SetMode.
	 */
	err = send_fm_write_1_param(audio_user,
				    CG2900_FM_CMD_ID_AUP_EXT_SET_MODE,
				    CG2900_FM_CMD_AUP_EXT_SET_MODE_PARALLEL);
	if (err)
		goto finished_unlock_mutex;

	/*
	 * Now configure the External Sample Rate Converter by sending
	 * HCI_Write command with AUP_EXT_SetControl.
	 */
	err = send_fm_write_1_param(
		audio_user, CG2900_FM_CMD_ID_AUP_EXT_SET_CTRL,
		fm_get_conversion(fm_config->fm.sample_rate));
	if (err)
		goto finished_unlock_mutex;

	/* Set up the stream */
	if (audio_info->revision == CHIP_REV_PG1) {
		struct i2s_fm_stream_config_priv stream_priv;

		/* Now send HCI_VS_Set_Session_Configuration command */
		stream_priv.fm_config = &fm_config->fm;
		stream_priv.rx = true;
		err = send_vs_session_config(audio_user, config_i2s_fm_stream,
					     &stream_priv);
	} else {
		struct mc_vs_port_cfg_fm fm_cfg;

		memset(&fm_cfg, 0, sizeof(fm_cfg));

		/* Configure port FM RX */
		/* Expects 0-3 - same as user API - so no conversion needed */
		PORTCFG_FM_SET_SRATE(fm_cfg, (u8)fm_config->fm.sample_rate);

		err = send_vs_port_cfg(audio_user, CG2900_MC_PORT_FM_RX_1,
				       &fm_cfg, sizeof(fm_cfg));
		if (err)
			goto finished_unlock_mutex;

		/* CreateStream */
		err = send_vs_create_stream(audio_user,
					    CG2900_MC_PORT_FM_RX_1,
					    CG2900_MC_PORT_I2S,
					    0); /* chip doesn't care */
	}

	if (err < 0)
		goto finished_unlock_mutex;

	/* Store the stream handle (used for start and stop stream) */
	*stream_handle = (u8)err;
	CG2900_DBG("stream_handle set to %d", *stream_handle);

	/* Now start the stream */
	if (audio_info->revision == CHIP_REV_PG1)
		err = send_vs_session_ctrl(audio_user, *stream_handle,
					   CG2900_BT_SESSION_START);
	else
		err = send_vs_stream_ctrl(audio_user, *stream_handle,
					  CG2900_MC_STREAM_START);

finished_unlock_mutex:
	SET_RESP_STATE(audio_user->resp_state, IDLE);
	mutex_unlock(&audio_info->bt_mutex);
	mutex_unlock(&audio_info->fm_mutex);
	return err;
}

/**
 * conn_start_i2s_to_fm_tx() - Start an audio stream connecting FM TX to I2S.
 * @audio_user:		Audio user to check for.
 * @stream_handle:	[out] Pointer where to store the stream handle.
 *
 * This function sets up an I2S to FM TX stream.
 * It does this by first setting the Audio Input source and then setting the
 * configuration and input source of BT sample rate converter.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -ENOMEM upon allocation errors.
 *   -EIO for other errors.
 */
static int conn_start_i2s_to_fm_tx(struct audio_user *audio_user,
				   unsigned int *stream_handle)
{
	int err = 0;
	union cg2900_endpoint_config_union *fm_config;

	fm_config = find_endpoint(ENDPOINT_FM_TX, &(audio_info->endpoints));
	if (!fm_config) {
		CG2900_ERR("FM TX not configured before stream start");
		return -EIO;
	}

	if (!(audio_info->i2s_config_known)) {
		CG2900_ERR("I2S DAI not configured before stream start");
		return -EIO;
	}

	/*
	 * Use mutex to assure that only ONE command is sent at any time
	 * on each channel.
	 */
	mutex_lock(&audio_info->fm_mutex);
	mutex_lock(&audio_info->bt_mutex);

	/*
	 * Select Audio Input Source by sending HCI_Write command with
	 * AIP_SetMode.
	 */
	CG2900_DBG("FM: AIP_SetMode");
	err = send_fm_write_1_param(audio_user, CG2900_FM_CMD_ID_AIP_SET_MODE,
				    CG2900_FM_CMD_AIP_SET_MODE_INPUT_DIG);
	if (err)
		goto finished_unlock_mutex;

	/*
	 * Now configure the BT sample rate converter by sending HCI_Write
	 * command with AIP_BT_SetControl.
	 */
	CG2900_DBG("FM: AIP_BT_SetControl");
	err = send_fm_write_1_param(
		audio_user, CG2900_FM_CMD_ID_AIP_BT_SET_CTRL,
		fm_get_conversion(fm_config->fm.sample_rate));
	if (err)
		goto finished_unlock_mutex;

	/*
	 * Now set input of the BT sample rate converter by sending HCI_Write
	 * command with AIP_BT_SetMode.
	 */
	CG2900_DBG("FM: AIP_BT_SetMode");
	err = send_fm_write_1_param(audio_user,
				    CG2900_FM_CMD_ID_AIP_BT_SET_MODE,
				    CG2900_FM_CMD_AIP_BT_SET_MODE_INPUT_PAR);
	if (err)
		goto finished_unlock_mutex;

	/* Set up the stream */
	if (audio_info->revision == CHIP_REV_PG1) {
		struct i2s_fm_stream_config_priv stream_priv;

		/* Now send HCI_VS_Set_Session_Configuration command */
		stream_priv.fm_config = &fm_config->fm;
		stream_priv.rx = false;
		err = send_vs_session_config(audio_user, config_i2s_fm_stream,
					     &stream_priv);
	} else {
		struct mc_vs_port_cfg_fm fm_cfg;

		memset(&fm_cfg, 0, sizeof(fm_cfg));

		/* Configure port FM TX */
		/* Expects 0-3 - same as user API - so no conversion needed */
		PORTCFG_FM_SET_SRATE(fm_cfg, (u8)fm_config->fm.sample_rate);

		err = send_vs_port_cfg(audio_user, CG2900_MC_PORT_FM_TX,
				       &fm_cfg, sizeof(fm_cfg));
		if (err)
			goto finished_unlock_mutex;

		/* CreateStream */
		err = send_vs_create_stream(audio_user,
					    CG2900_MC_PORT_I2S,
					    CG2900_MC_PORT_FM_TX,
					    0); /* chip doesn't care */
	}

	if (err < 0)
		goto finished_unlock_mutex;

	/* Store the stream handle (used for start and stop stream) */
	*stream_handle = (u8)err;
	CG2900_DBG("stream_handle set to %d", *stream_handle);

	/* Now start the stream */
	if (audio_info->revision == CHIP_REV_PG1)
		err = send_vs_session_ctrl(audio_user, *stream_handle,
					   CG2900_BT_SESSION_START);
	else
		err = send_vs_stream_ctrl(audio_user, *stream_handle,
					  CG2900_MC_STREAM_START);

finished_unlock_mutex:
	SET_RESP_STATE(audio_user->resp_state, IDLE);
	mutex_unlock(&audio_info->bt_mutex);
	mutex_unlock(&audio_info->fm_mutex);
	return err;
}

/**
 * config_pcm_sco_stream() - Callback for @send_vs_session_config.
 * @_priv:	Pointer to a @cg2900_endpoint_config_sco_in_out struct.
 * @cfg:	Pointer to stream config block in command packet.
 *
 * Fills in stream configuration for PCM-SCO.
 */
static void config_pcm_sco_stream(void *_priv,
				  struct session_config_stream *cfg)
{
	struct cg2900_endpoint_config_sco_in_out *sco_ep = _priv;

	cfg->media_type = CG2900_BT_SESSION_MEDIA_TYPE_AUDIO;

	SESSIONCFG_SET_CHANNELS(cfg, CG2900_BT_MEDIA_CONFIG_MONO);
	SESSIONCFG_I2S_SET_SRATE(cfg,
		session_config_sample_rate(sco_ep->sample_rate));

	cfg->codec_type = CG2900_CODEC_TYPE_NONE;
	/* codec mode and parameters not used  */

	cfg->inport.type = CG2900_BT_VP_TYPE_BT_SCO;
	cfg->inport.sco.acl_handle = cpu_to_le16(DEFAULT_SCO_HANDLE);

	cfg->outport.type = CG2900_BT_VP_TYPE_PCM;
	cfg->outport.pcm.index = CG2900_BT_SESSION_PCM_INDEX_PCM_I2S;

	SESSIONCFG_PCM_SET_USED(cfg->outport, 0,
				audio_info->i2s_pcm_config.slot_0_used);
	SESSIONCFG_PCM_SET_USED(cfg->outport, 1,
				audio_info->i2s_pcm_config.slot_1_used);
	SESSIONCFG_PCM_SET_USED(cfg->outport, 2,
				audio_info->i2s_pcm_config.slot_2_used);
	SESSIONCFG_PCM_SET_USED(cfg->outport, 3,
				audio_info->i2s_pcm_config.slot_3_used);

	cfg->outport.pcm.slot_start[0] =
		audio_info->i2s_pcm_config.slot_0_start;
	cfg->outport.pcm.slot_start[1] =
		audio_info->i2s_pcm_config.slot_1_start;
	cfg->outport.pcm.slot_start[2] =
		audio_info->i2s_pcm_config.slot_2_start;
	cfg->outport.pcm.slot_start[3] =
		audio_info->i2s_pcm_config.slot_3_start;
}

/**
 * conn_start_pcm_to_sco() - Start an audio stream connecting Bluetooth (e)SCO to PCM_I2S.
 * @audio_user:		Audio user to check for.
 * @stream_handle:	[out] Pointer where to store the stream handle.
 *
 * This function sets up a BT to_from PCM_I2S stream. It does this by
 * first setting the Session configuration and then starting the Audio
 * Stream.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -ENOMEM upon allocation errors.
 *   Errors from @cg2900_write
 *   -EIO for other errors.
 */
static int conn_start_pcm_to_sco(struct audio_user *audio_user,
				 unsigned int *stream_handle)
{
	int err = 0;
	union cg2900_endpoint_config_union *bt_config;

	bt_config = find_endpoint(ENDPOINT_BT_SCO_INOUT,
				  &(audio_info->endpoints));
	if (!bt_config) {
		CG2900_ERR("BT not configured before stream start");
		return -EIO;
	}

	if (!(audio_info->i2s_pcm_config_known)) {
		CG2900_ERR("I2S_PCM DAI not configured before stream start");
		return -EIO;
	}

	/*
	 * Use mutex to assure that only ONE command is sent at any time on each
	 * channel.
	 */
	mutex_lock(&audio_info->bt_mutex);

	/* Set up the stream */
	if (audio_info->revision == CHIP_REV_PG1) {
		err = send_vs_session_config(audio_user, config_pcm_sco_stream,
					     &bt_config->sco);
	} else {
		struct mc_vs_port_cfg_sco sco_cfg;

		/* zero codec params etc */
		memset(&sco_cfg, 0, sizeof(sco_cfg));
		sco_cfg.acl_id = DEFAULT_SCO_HANDLE;
		PORTCFG_SCO_SET_WBS(sco_cfg, 0); /* No WBS yet */
		PORTCFG_SCO_SET_CODEC(sco_cfg, CG2900_CODEC_TYPE_NONE);

		err = send_vs_port_cfg(audio_user, CG2900_MC_PORT_BT_SCO,
				       &sco_cfg, sizeof(sco_cfg));
		if (err)
			goto finished_unlock_mutex;

		/* CreateStream */
		err = send_vs_create_stream(audio_user,
					    CG2900_MC_PORT_PCM_I2S,
					    CG2900_MC_PORT_BT_SCO,
					    0); /* chip doesn't care */
	}

	if (err < 0)
		goto finished_unlock_mutex;

	/* Store the stream handle (used for start and stop stream) */
	*stream_handle = (u8)err;
	CG2900_DBG("stream_handle set to %d", *stream_handle);

	/* Now start the stream by sending HCI_VS_Session_Control command */
	err = send_vs_session_ctrl(audio_user, *stream_handle,
				   CG2900_BT_SESSION_START);

finished_unlock_mutex:
	SET_RESP_STATE(audio_user->resp_state, IDLE);
	mutex_unlock(&audio_info->bt_mutex);
	return err;
}

/**
 * conn_stop_stream() - Stops an audio stream defined by @stream_handle.
 * @audio_user:		Audio user to check for.
 * @stream_handle:	Handle of the audio stream.
 *
 * This function is used to stop an audio stream defined by a stream
 * handle. It does this by first stopping the stream and then
 * resetting the session/stream.
 *
 * Returns:
 *   0 if there is no error.
 *   -ECOMM if no response was received.
 *   -ENOMEM upon allocation errors.
 *   Errors from @cg2900_write.
 *   -EIO for other errors.
 */
static int conn_stop_stream(struct audio_user *audio_user,
			    unsigned int stream_handle)
{
	int err = 0;
	struct sk_buff *skb;
	u16 opcode;

	/*
	 * Use mutex to assure that only ONE command is sent at any
	 * time on each channel.
	 */
	mutex_lock(&audio_info->bt_mutex);

	/* Now stop the stream */
	if (audio_info->revision == CHIP_REV_PG1)
		err = send_vs_session_ctrl(audio_user, stream_handle,
					   CG2900_BT_SESSION_STOP);
	else
		err = send_vs_stream_ctrl(audio_user, stream_handle,
					  CG2900_MC_STREAM_STOP);
	if (err)
		goto finished_unlock_mutex;

	/* Now delete the stream - format command... */
	if (audio_info->revision == CHIP_REV_PG1) {
		struct bt_vs_reset_session_cfg_cmd *cmd;

		CG2900_DBG("BT: HCI_VS_Reset_Session_Configuration");

		skb = cg2900_alloc_skb(sizeof(*cmd), GFP_KERNEL);
		if (!skb) {
			CG2900_ERR("Could not allocate skb");
			err = -ENOMEM;
			goto finished_unlock_mutex;
		}

		cmd = (struct bt_vs_reset_session_cfg_cmd *)
			skb_put(skb, sizeof(*cmd));

		opcode = CG2900_BT_VS_RESET_SESSION_CONFIG;
		cmd->opcode = cpu_to_le16(opcode);
		cmd->plen   = BT_PARAM_LEN(sizeof(*cmd));
		cmd->id     = (u8)stream_handle;
	} else {
		struct mc_vs_delete_stream_cmd *cmd;

		CG2900_DBG("BT: HCI_VS_Delete_Stream");

		skb = cg2900_alloc_skb(sizeof(*cmd), GFP_KERNEL);
		if (!skb) {
			CG2900_ERR("Could not allocate skb");
			err = -ENOMEM;
			goto finished_unlock_mutex;
		}

		cmd = (struct mc_vs_delete_stream_cmd *)
			skb_put(skb, sizeof(*cmd));

		opcode = CG2900_MC_VS_DELETE_STREAM;
		cmd->opcode = cpu_to_le16(opcode);
		cmd->plen   = BT_PARAM_LEN(sizeof(*cmd));
		cmd->stream = (u8)stream_handle;
	}

	/* ...and send it */
	cb_info_bt.user = audio_user;
	SET_RESP_STATE(audio_user->resp_state, WAITING);

	err = cg2900_write(audio_info->dev_bt, skb);
	if (err) {
		CG2900_ERR("Error occurred while transmitting skb (%d)", err);
		goto error_handling_free_skb;
	}

	/* wait for response */
	if (audio_info->revision == CHIP_REV_PG1) {
		err = receive_bt_cmd_complete(audio_user, opcode, NULL, 0);
	} else {
		u8 vs_err;

		/* All commands in PG2 API returns one byte extra status */
		err = receive_bt_cmd_complete(audio_user, opcode,
					      &vs_err, sizeof(vs_err));

		if (err)
			CG2900_DBG("VS_DELETE_STREAM - failed with error %02x",
				   vs_err);
		else
			release_stream_id(stream_handle);

	}

	goto finished_unlock_mutex;

error_handling_free_skb:
	kfree_skb(skb);
finished_unlock_mutex:
	SET_RESP_STATE(audio_user->resp_state, IDLE);
	mutex_unlock(&audio_info->bt_mutex);
	return err;
}

/**
 * cg2900_audio_open() - Opens a session to the ST-Ericsson CG2900 Audio control interface.
 * @session:	[out] Address where to store the session identifier.
 *		Allocated by caller, must not be NULL.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -ENOMEM upon allocation failure.
 *   -EMFILE if no more user session could be opened.
 *   -EIO upon failure to register to CG2900.
 */
int cg2900_audio_open(unsigned int *session)
{
	int err = 0;
	int i;

	CG2900_INFO("cg2900_audio_open");

	if (!session) {
		CG2900_ERR("NULL supplied as session.");
		return -EINVAL;
	}

	mutex_lock(&audio_info->management_mutex);

	*session = 0;

	/*
	 * First find a free session to use and allocate the session structure.
	 */
	for (i = FIRST_USER;
	     i < MAX_NBR_OF_USERS && audio_info->audio_sessions[i];
	     i++)
		; /* Just loop until found or end reached */

	if (i >= MAX_NBR_OF_USERS) {
		CG2900_ERR("Couldn't find free user");
		err = -EMFILE;
		goto finished;
	}

	audio_info->audio_sessions[i] =
			kzalloc(sizeof(*(audio_info->audio_sessions[0])),
				GFP_KERNEL);
	if (!audio_info->audio_sessions[i]) {
		CG2900_ERR("Could not allocate user");
		err = -ENOMEM;
		goto finished;
	}
	CG2900_DBG("Found free session %d", i);
	*session = i;
	audio_info->nbr_of_users_active++;

	SET_RESP_STATE(audio_info->audio_sessions[*session]->resp_state, IDLE);
	audio_info->audio_sessions[*session]->session = *session;

	if (audio_info->nbr_of_users_active == 1) {
		struct cg2900_rev_data rev_data;

		/*
		 * First user so register to CG2900 Core.
		 * First the BT audio device.
		 */
		audio_info->dev_bt = cg2900_register_user(CG2900_BT_AUDIO,
							  &cg2900_cb);
		if (!audio_info->dev_bt) {
			CG2900_ERR("Failed to register BT audio channel");
			err = -EIO;
			goto error_handling;
		}

		/* Store the callback info structure */
		audio_info->dev_bt->user_data = &cb_info_bt;

		/* Then the FM audio device */
		audio_info->dev_fm = cg2900_register_user(CG2900_FM_RADIO_AUDIO,
							  &cg2900_cb);
		if (!audio_info->dev_fm) {
			CG2900_ERR("Failed to register FM audio channel");
			err = -EIO;
			goto error_handling;
		}

		/* Store the callback info structure */
		audio_info->dev_fm->user_data = &cb_info_fm;

		/* Read chip revision data */
		if (!cg2900_get_local_revision(&rev_data)) {
			CG2900_ERR("Couldn't retrieve revision data");
			err = -EIO;
			goto error_handling;
		}

		/* Decode revision data */
		switch (rev_data.revision) {
		case CG2900_PG1_REV:
		case CG2900_PG1_SPECIAL_REV:
			audio_info->revision = CHIP_REV_PG1;
			break;

		case CG2900_PG2_REV:
			audio_info->revision = CHIP_REV_PG2;
			break;

		default:
			CG2900_ERR("Chip rev 0x%04X sub 0x%04X not supported",
				   rev_data.revision, rev_data.sub_version);
			err = -EIO;
			goto error_handling;
		}

		audio_info->state = OPENED;
	}

	goto finished;

error_handling:
	if (audio_info->dev_fm) {
		cg2900_deregister_user(audio_info->dev_fm);
		audio_info->dev_fm = NULL;
	}
	if (audio_info->dev_bt) {
		cg2900_deregister_user(audio_info->dev_bt);
		audio_info->dev_bt = NULL;
	}
	audio_info->nbr_of_users_active--;
	kfree(audio_info->audio_sessions[*session]);
	audio_info->audio_sessions[*session] = NULL;
finished:
	mutex_unlock(&audio_info->management_mutex);
	return err;
}
EXPORT_SYMBOL(cg2900_audio_open);

/**
 * cg2900_audio_close() - Closes an opened session to the ST-Ericsson CG2900 audio control interface.
 * @session:	[in_out] Pointer to session identifier to close.
 *		Will be 0 after this call.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -EIO if driver has not been opened.
 *   -EACCES if session has not opened.
 */
int cg2900_audio_close(unsigned int *session)
{
	int err = 0;
	struct audio_user *audio_user;

	CG2900_INFO("cg2900_audio_close");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	if (!session) {
		CG2900_ERR("NULL pointer supplied");
		return -EINVAL;
	}

	audio_user = get_session_user(*session);
	if (!audio_user) {
		CG2900_ERR("Invalid session ID");
		return -EINVAL;
	}

	mutex_lock(&audio_info->management_mutex);

	if (!(audio_info->audio_sessions[*session])) {
		CG2900_ERR("Session %d not opened", *session);
		err = -EACCES;
		goto err_unlock_mutex;
	}

	kfree(audio_info->audio_sessions[*session]);
	audio_info->audio_sessions[*session] = NULL;
	audio_info->nbr_of_users_active--;

	if (audio_info->nbr_of_users_active == 0) {
		/* No more sessions open. Deregister from CG2900 Core */
		cg2900_deregister_user(audio_info->dev_fm);
		cg2900_deregister_user(audio_info->dev_bt);
		audio_info->state = CLOSED;
	}

	*session = 0;

err_unlock_mutex:
	mutex_unlock(&audio_info->management_mutex);
	return err;
}
EXPORT_SYMBOL(cg2900_audio_close);

/**
 * cg2900_audio_set_dai_config() -  Sets the Digital Audio Interface configuration.
 * @session:	Session identifier this call is related to.
 * @config:	Pointer to the configuration to set.
 *		Allocated by caller, must not be NULL.
 *
 * Sets the Digital Audio Interface (DAI) configuration. The DAI is the external
 * interface between the combo chip and the platform.
 * For example the PCM or I2S interface.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -EIO if driver has not been opened.
 *   -ENOMEM upon allocation failure.
 *   -EACCES if trying to set unsupported configuration.
 *   Errors from @receive_bt_cmd_complete.
 */
int cg2900_audio_set_dai_config(unsigned int session,
				struct cg2900_dai_config *config)
{
	int err = 0;
	struct audio_user *audio_user;
	struct cg2900_rev_data rev_data;

	CG2900_INFO("cg2900_audio_set_dai_config");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	audio_user = get_session_user(session);
	if (!audio_user)
		return -EINVAL;

	if (!cg2900_get_local_revision(&rev_data)) {
		CG2900_ERR("Couldn't retrieve revision data");
		return -EIO;
	}

	/* Different commands are used for PG1 and PG2 */
	switch (rev_data.revision) {
	case CG2900_PG1_REV:
	case CG2900_PG1_SPECIAL_REV:
		err = set_dai_config_pg1(audio_user, config);
		break;

	case CG2900_PG2_REV:
		err = set_dai_config_pg2(audio_user, config);
		break;

	default:
		CG2900_ERR("Chip rev 0x%04X sub 0x%04X not supported",
			   rev_data.revision, rev_data.sub_version);
		err = -EIO;
	}

	return err;
}
EXPORT_SYMBOL(cg2900_audio_set_dai_config);

/**
 * cg2900_audio_get_dai_config() - Gets the current Digital Audio Interface configuration.
 * @session:	Session identifier this call is related to.
 * @config:	[out] Pointer to the configuration to get.
 *		Allocated by caller, must not be NULL.
 *
 * Gets the current Digital Audio Interface configuration. Currently this method
 * can only be called after some one has called
 * cg2900_audio_set_dai_config(), there is today no way of getting
 * the static settings file parameters from this method.
 * Note that the @port parameter within @config must be set when calling this
 * function so that the ST-Ericsson CG2900 Audio driver will know which
 * configuration to return.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -EIO if driver has not been opened or configuration has not been set.
 */
int cg2900_audio_get_dai_config(unsigned int session,
				struct cg2900_dai_config *config)
{
	int err = 0;
	struct audio_user *audio_user;

	CG2900_INFO("cg2900_audio_get_dai_config");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	if (!config) {
		CG2900_ERR("NULL supplied as config structure");
		return -EINVAL;
	}

	audio_user = get_session_user(session);
	if (!audio_user)
		return -EINVAL;

	/*
	 * Return DAI configuration based on the received port.
	 * If port has not been configured return error.
	 */
	switch (config->port) {
	case PORT_0_I2S:
		mutex_lock(&audio_info->management_mutex);
		if (audio_info->i2s_config_known)
			memcpy(&(config->conf.i2s),
			       &(audio_info->i2s_config),
			       sizeof(config->conf.i2s));
		else
			err = -EIO;
		mutex_unlock(&audio_info->management_mutex);
		break;

	case PORT_1_I2S_PCM:
		mutex_lock(&audio_info->management_mutex);
		if (audio_info->i2s_pcm_config_known)
			memcpy(&(config->conf.i2s_pcm),
			       &(audio_info->i2s_pcm_config),
			       sizeof(config->conf.i2s_pcm));
		else
			err = -EIO;
		mutex_unlock(&audio_info->management_mutex);
		break;

	default:
		CG2900_ERR("Unknown port configuration %d", config->port);
		err = -EIO;
		break;
	};

	return err;
}
EXPORT_SYMBOL(cg2900_audio_get_dai_config);

/**
 * cg2900_audio_config_endpoint() - Configures one endpoint in the combo chip's audio system.
 * @session:	Session identifier this call is related to.
 * @config:	Pointer to the endpoint's configuration structure.
 *
 * Configures one endpoint in the combo chip's audio system.
 * Supported @endpoint_id values are:
 *  * ENDPOINT_BT_SCO_INOUT
 *  * ENDPOINT_BT_A2DP_SRC
 *  * ENDPOINT_FM_RX
 *  * ENDPOINT_FM_TX
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -EIO if driver has not been opened.
 *   -EACCES if supplied cg2900_dai_config struct contains not supported
 *   endpoint_id.
 */
int cg2900_audio_config_endpoint(unsigned int session,
				 struct cg2900_endpoint_config *config)
{
	struct audio_user *audio_user;

	CG2900_INFO("cg2900_audio_config_endpoint");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	if (!config) {
		CG2900_ERR("NULL supplied as configuration structure");
		return -EINVAL;
	}

	audio_user = get_session_user(session);
	if (!audio_user)
		return -EINVAL;

	switch (config->endpoint_id) {
	case ENDPOINT_BT_SCO_INOUT:
	case ENDPOINT_BT_A2DP_SRC:
	case ENDPOINT_FM_RX:
	case ENDPOINT_FM_TX:
		add_endpoint(config, &(audio_info->endpoints));
		break;

	case ENDPOINT_PORT_0_I2S:
	case ENDPOINT_PORT_1_I2S_PCM:
	case ENDPOINT_SLIMBUS_VOICE:
	case ENDPOINT_SLIMBUS_AUDIO:
	case ENDPOINT_BT_A2DP_SNK:
	case ENDPOINT_ANALOG_OUT:
	case ENDPOINT_DSP_AUDIO_IN:
	case ENDPOINT_DSP_AUDIO_OUT:
	case ENDPOINT_DSP_VOICE_IN:
	case ENDPOINT_DSP_VOICE_OUT:
	case ENDPOINT_DSP_TONE_IN:
	case ENDPOINT_BURST_BUFFER_IN:
	case ENDPOINT_BURST_BUFFER_OUT:
	case ENDPOINT_MUSIC_DECODER:
	case ENDPOINT_HCI_AUDIO_IN:
	default:
		CG2900_ERR("Unknown endpoint_id %d", config->endpoint_id);
		return -EACCES;
	}

	return 0;
}
EXPORT_SYMBOL(cg2900_audio_config_endpoint);

static bool is_dai_port(enum cg2900_audio_endpoint_id ep)
{
	/* These are the only supported ones */
	return (ep == ENDPOINT_PORT_0_I2S) || (ep == ENDPOINT_PORT_1_I2S_PCM);
}

/**
 * cg2900_audio_start_stream() - Connects two endpoints and starts the audio stream.
 * @session:		Session identifier this call is related to.
 * @ep_1:		One of the endpoints, no relation to direction or role.
 * @ep_2:		The other endpoint, no relation to direction or role.
 * @stream_handle:	Pointer where to store the stream handle.
 *			Allocated by caller, must not be NULL.
 *
 * Connects two endpoints and starts the audio stream.
 * Note that the endpoints need to be configured before the stream is started;
 * DAI endpoints, such as ENDPOINT_PORT_0_I2S, are
 * configured through @cg2900_audio_set_dai_config() while other
 * endpoints are configured through @cg2900_audio_config_endpoint().
 *
 * Supported @endpoint_id values are:
 *  * ENDPOINT_PORT_0_I2S
 *  * ENDPOINT_PORT_1_I2S_PCM
 *  * ENDPOINT_BT_SCO_INOUT
 *  * ENDPOINT_FM_RX
 *  * ENDPOINT_FM_TX
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter or unsupported configuration.
 *   -EIO if driver has not been opened.
 *   Errors from @conn_start_i2s_to_fm_rx, @conn_start_i2s_to_fm_tx, and
 *   @conn_start_pcm_to_sco.
 */
int cg2900_audio_start_stream(unsigned int session,
			      enum cg2900_audio_endpoint_id ep_1,
			      enum cg2900_audio_endpoint_id ep_2,
			      unsigned int *stream_handle)
{
	int err;
	struct audio_user *audio_user;

	CG2900_INFO("cg2900_audio_start_stream");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	audio_user = get_session_user(session);
	if (!audio_user)
		return -EINVAL;

	/* put digital interface in ep_1 to simplify comparison below */
	if (!is_dai_port(ep_1)) {
		/* Swap endpoints */
		enum cg2900_audio_endpoint_id t = ep_1;
		ep_1 = ep_2;
		ep_2 = t;
	}

	if (ep_1 == ENDPOINT_PORT_1_I2S_PCM && ep_2 == ENDPOINT_BT_SCO_INOUT) {
		err = conn_start_pcm_to_sco(audio_user, stream_handle);
	} else if (ep_1 == ENDPOINT_PORT_0_I2S && ep_2 == ENDPOINT_FM_RX) {
		err = conn_start_i2s_to_fm_rx(audio_user, stream_handle);
	} else if (ep_1 == ENDPOINT_PORT_0_I2S && ep_2 == ENDPOINT_FM_TX) {
		err = conn_start_i2s_to_fm_tx(audio_user, stream_handle);
	} else {
		CG2900_ERR("Endpoint config not handled: ep1: %d, "
			   "ep2: %d", ep_1, ep_2);
		err = -EINVAL;
	}

	return err;
}
EXPORT_SYMBOL(cg2900_audio_start_stream);

/**
 * cg2900_audio_stop_stream() - Stops a stream and disconnects the endpoints.
 * @session:		Session identifier this call is related to.
 * @stream_handle:	Handle to the stream to stop.
 *
 * Returns:
 *   0 if there is no error.
 *   -EINVAL upon bad input parameter.
 *   -EIO if driver has not been opened.
 */
int cg2900_audio_stop_stream(unsigned int session, unsigned int stream_handle)
{
	struct audio_user *audio_user;

	CG2900_INFO("cg2900_audio_stop_stream");

	if (audio_info->state != OPENED) {
		CG2900_ERR("Audio driver not open");
		return -EIO;
	}

	audio_user = get_session_user(session);
	if (!audio_user)
		return -EINVAL;

	return conn_stop_stream(audio_user, stream_handle);
}
EXPORT_SYMBOL(cg2900_audio_stop_stream);

/**
 * audio_dev_open() - Open char device.
 * @inode:	Device driver information.
 * @filp:	Pointer to the file struct.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation failed.
 *   Errors from @cg2900_audio_open.
 */
static int audio_dev_open(struct inode *inode, struct file *filp)
{
	int err;
	struct char_dev_info *char_dev_info;

	CG2900_INFO("CG2900 Audio: audio_dev_open");

	/*
	 * Allocate the char dev info structure. It will be stored inside
	 * the file pointer and supplied when file_ops are called.
	 * It's free'd in audio_dev_release.
	 */
	char_dev_info = kzalloc(sizeof(*char_dev_info), GFP_KERNEL);
	if (!char_dev_info) {
		CG2900_ERR("Couldn't allocate char_dev_info");
		return -ENOMEM;
	}
	filp->private_data = char_dev_info;

	mutex_init(&char_dev_info->management_mutex);
	mutex_init(&char_dev_info->rw_mutex);

	mutex_lock(&char_dev_info->management_mutex);
	err = cg2900_audio_open(&char_dev_info->session);
	mutex_unlock(&char_dev_info->management_mutex);
	if (err) {
		CG2900_ERR("Failed to open CG2900 Audio driver (%d)", err);
		goto error_handling_free_mem;
	}

	return 0;

error_handling_free_mem:
	kfree(char_dev_info);
	filp->private_data = NULL;
	return err;
}

/**
 * audio_dev_release() - Release char device.
 * @inode:	Device driver information.
 * @filp:	Pointer to the file struct.
 *
 * Returns:
 *   0 if there is no error.
 *   -EBADF if NULL pointer was supplied in private data.
 *   Errors from @cg2900_audio_close.
 */
static int audio_dev_release(struct inode *inode, struct file *filp)
{
	int err = 0;
	struct char_dev_info *dev = (struct char_dev_info *)filp->private_data;

	CG2900_INFO("CG2900 Audio: audio_dev_release");

	if (!dev) {
		CG2900_ERR("No dev supplied in private data");
		return -EBADF;
	}

	mutex_lock(&dev->management_mutex);
	err = cg2900_audio_close(&dev->session);
	if (err)
		/*
		 * Just print the error. Still free the char_dev_info since we
		 * don't know the filp structure is valid after this call
		 */
		CG2900_ERR("Error when closing CG2900 audio driver (%d)", err);

	mutex_unlock(&dev->management_mutex);

	kfree(dev);
	filp->private_data = NULL;

	return err;
}

/**
 * audio_dev_read() - Return information to the user from last @write call.
 * @filp:	Pointer to the file struct.
 * @buf:	Received buffer.
 * @count:	Size of buffer.
 * @f_pos:	Position in buffer.
 *
 * The audio_dev_read() function returns information from
 * the last @write call to same char device.
 * The data is in the following format:
 *   * OpCode of command for this data
 *   * Data content (Length of data is determined by the command OpCode, i.e.
 *     fixed for each command)
 *
 * Returns:
 *   Bytes successfully read (could be 0).
 *   -EBADF if NULL pointer was supplied in private data.
 *   -EFAULT if copy_to_user fails.
 *   -ENOMEM upon allocation failure.
 */
static ssize_t audio_dev_read(struct file *filp, char __user *buf, size_t count,
			      loff_t *f_pos)
{
	struct char_dev_info *dev = (struct char_dev_info *)filp->private_data;
	unsigned int bytes_to_copy = 0;
	int err = 0;

	CG2900_INFO("CG2900 Audio: audio_dev_read");

	if (!dev) {
		CG2900_ERR("No dev supplied in private data");
		return -EBADF;
	}
	mutex_lock(&dev->rw_mutex);

	if (dev->stored_data_len == 0) {
		/* No data to read */
		bytes_to_copy = 0;
		goto finished;
	}

	bytes_to_copy = min(count, (unsigned int)(dev->stored_data_len));
	if (bytes_to_copy < dev->stored_data_len)
		CG2900_ERR("Not enough buffer to store all data. Throwing away "
			   "rest of data. Saved len: %d, stored_len: %d",
			   count, dev->stored_data_len);

	err = copy_to_user(buf, dev->stored_data, bytes_to_copy);
	/*
	 * Throw away all data, even though not all was copied.
	 * This char device is primarily for testing purposes so we can keep
	 * such a limitation.
	 */
	kfree(dev->stored_data);
	dev->stored_data = NULL;
	dev->stored_data_len = 0;

	if (err) {
		CG2900_ERR("copy_to_user error %d", err);
		err = -EFAULT;
		goto error_handling;
	}

	goto finished;

error_handling:
	mutex_unlock(&dev->rw_mutex);
	return (ssize_t)err;
finished:
	mutex_unlock(&dev->rw_mutex);
	return bytes_to_copy;
}

/**
 * audio_dev_write() - Call CG2900 Audio API function.
 * @filp:	Pointer to the file struct.
 * @buf:	Write buffer.
 * @count:	Size of the buffer write.
 * @f_pos:	Position of buffer.
 *
 * audio_dev_write() function executes supplied data and
 * interprets it as if it was a function call to the CG2900 Audio API.
 * The data is according to:
 *   * OpCode (4 bytes)
 *   * Data according to OpCode (see API). No padding between parameters
 *
 * OpCodes are:
 *   * OP_CODE_SET_DAI_CONF 0x00000001
 *   * OP_CODE_GET_DAI_CONF 0x00000002
 *   * OP_CODE_CONFIGURE_ENDPOINT 0x00000003
 *   * OP_CODE_START_STREAM 0x00000004
 *   * OP_CODE_STOP_STREAM 0x00000005
 *
 * Returns:
 *   Bytes successfully written (could be 0). Equals input @count if successful.
 *   -EBADF if NULL pointer was supplied in private data.
 *   -EFAULT if copy_from_user fails.
 *   Error codes from all CG2900 Audio API functions.
 */
static ssize_t audio_dev_write(struct file *filp, const char __user *buf,
			       size_t count, loff_t *f_pos)
{
	u8 *rec_data;
	struct char_dev_info *dev = (struct char_dev_info *)filp->private_data;
	int err = 0;
	int op_code = 0;
	u8 *curr_data;
	unsigned int stream_handle;
	struct cg2900_dai_config dai_config;
	struct cg2900_endpoint_config ep_config;
	enum cg2900_audio_endpoint_id ep_1;
	enum cg2900_audio_endpoint_id ep_2;
	int bytes_left = count;

	CG2900_INFO("CG2900 Audio: audio_dev_write count %d", count);

	if (!dev) {
		CG2900_ERR("No dev supplied in private data");
		return -EBADF;
	}

	rec_data = kmalloc(count, GFP_KERNEL);
	if (!rec_data) {
		CG2900_ERR("kmalloc failed");
		return -ENOMEM;
	}

	mutex_lock(&dev->rw_mutex);

	err = copy_from_user(rec_data, buf, count);
	if (err) {
		CG2900_ERR("copy_from_user failed (%d)", err);
		err = -EFAULT;
		goto finished_mutex_unlock;
	}

	/* Initialize temporary data pointer used to traverse the packet */
	curr_data = rec_data;

	op_code = curr_data[0];
	CG2900_DBG("op_code %d", op_code);
	/* OpCode is int size to keep data int aligned */
	curr_data += sizeof(unsigned int);
	bytes_left -= sizeof(unsigned int);

	switch (op_code) {
	case OP_CODE_SET_DAI_CONF:
		CG2900_DBG("OP_CODE_SET_DAI_CONF %d", sizeof(dai_config));
		if (bytes_left < sizeof(dai_config)) {
			CG2900_ERR("Not enough data supplied for "
				   "OP_CODE_SET_DAI_CONF");
			err = -EINVAL;
			goto finished_mutex_unlock;
		}
		memcpy(&dai_config, curr_data, sizeof(dai_config));
		CG2900_DBG("dai_config.port %d", dai_config.port);
		err = cg2900_audio_set_dai_config(dev->session, &dai_config);
		break;

	case OP_CODE_GET_DAI_CONF:
		CG2900_DBG("OP_CODE_GET_DAI_CONF %d", sizeof(dai_config));
		if (bytes_left < sizeof(dai_config)) {
			CG2900_ERR("Not enough data supplied for "
				   "OP_CODE_GET_DAI_CONF");
			err = -EINVAL;
			goto finished_mutex_unlock;
		}
		/*
		 * Only need to copy the port really, but let's copy
		 * like this for simplicity. It's only test functionality
		 * after all.
		 */
		memcpy(&dai_config, curr_data, sizeof(dai_config));
		CG2900_DBG("dai_config.port %d", dai_config.port);
		err = cg2900_audio_get_dai_config(dev->session, &dai_config);
		if (!err) {
			/*
			 * Command succeeded. Store data so it can be returned
			 * when calling read.
			 */
			if (dev->stored_data) {
				CG2900_ERR("Data already allocated (%d bytes). "
					   "Throwing it away.",
					   dev->stored_data_len);
				kfree(dev->stored_data);
			}
			dev->stored_data_len = sizeof(op_code) +
					       sizeof(dai_config);
			dev->stored_data = kmalloc(dev->stored_data_len,
						   GFP_KERNEL);
			if (dev->stored_data) {
				memcpy(dev->stored_data, &op_code,
				       sizeof(op_code));
				memcpy(&(dev->stored_data[sizeof(op_code)]),
				       &dai_config, sizeof(dai_config));
			}
		}
		break;

	case OP_CODE_CONFIGURE_ENDPOINT:
		CG2900_DBG("OP_CODE_CONFIGURE_ENDPOINT %d", sizeof(ep_config));
		if (bytes_left < sizeof(ep_config)) {
			CG2900_ERR("Not enough data supplied for "
				   "OP_CODE_CONFIGURE_ENDPOINT");
			err = -EINVAL;
			goto finished_mutex_unlock;
		}
		memcpy(&ep_config, curr_data, sizeof(ep_config));
		CG2900_DBG("ep_config.endpoint_id %d", ep_config.endpoint_id);
		err = cg2900_audio_config_endpoint(dev->session, &ep_config);
		break;

	case OP_CODE_START_STREAM:
		CG2900_DBG("OP_CODE_START_STREAM %d",
			   (sizeof(ep_1) + sizeof(ep_2)));
		if (bytes_left < (sizeof(ep_1) + sizeof(ep_2))) {
			CG2900_ERR("Not enough data supplied for "
				   "OP_CODE_START_STREAM");
			err = -EINVAL;
			goto finished_mutex_unlock;
		}
		memcpy(&ep_1, curr_data, sizeof(ep_1));
		curr_data += sizeof(ep_1);
		memcpy(&ep_2, curr_data, sizeof(ep_2));
		CG2900_DBG("ep_1 %d ep_2 %d", ep_1,
			   ep_2);

		err = cg2900_audio_start_stream(dev->session,
			ep_1, ep_2, &stream_handle);
		if (!err) {
			/*
			 * Command succeeded. Store data so it can be returned
			 * when calling read.
			 */
			if (dev->stored_data) {
				CG2900_ERR("Data already allocated (%d bytes). "
					   "Throwing it away.",
					   dev->stored_data_len);
				kfree(dev->stored_data);
			}
			dev->stored_data_len = sizeof(op_code) +
					       sizeof(stream_handle);
			dev->stored_data = kmalloc(dev->stored_data_len,
						   GFP_KERNEL);
			if (dev->stored_data) {
				memcpy(dev->stored_data, &op_code,
				       sizeof(op_code));
				memcpy(&(dev->stored_data[sizeof(op_code)]),
				       &stream_handle, sizeof(stream_handle));
			}
			CG2900_DBG("stream_handle %d", stream_handle);
		}
		break;

	case OP_CODE_STOP_STREAM:
		if (bytes_left < sizeof(stream_handle)) {
			CG2900_ERR("Not enough data supplied for "
				   "OP_CODE_STOP_STREAM");
			err = -EINVAL;
			goto finished_mutex_unlock;
		}
		CG2900_DBG("OP_CODE_STOP_STREAM %d", sizeof(stream_handle));
		memcpy(&stream_handle, curr_data, sizeof(stream_handle));
		CG2900_DBG("stream_handle %d", stream_handle);
		err = cg2900_audio_stop_stream(dev->session, stream_handle);
		break;

	default:
		CG2900_ERR("Received bad op_code %d", op_code);
		break;
	};

finished_mutex_unlock:
	kfree(rec_data);
	mutex_unlock(&dev->rw_mutex);

	if (err)
		return err;
	else
		return count;
}

/**
 * audio_dev_poll() - Handle POLL call to the interface.
 * @filp:	Pointer to the file struct.
 * @wait:	Poll table supplied to caller.
 *
 * This function is used by the User Space application to see if the device is
 * still open and if there is any data available for reading.
 *
 * Returns:
 *   Mask of current set POLL values.
 */
static unsigned int audio_dev_poll(struct file *filp, poll_table *wait)
{
	struct char_dev_info *dev = (struct char_dev_info *)filp->private_data;
	unsigned int mask = 0;

	if (!dev) {
		CG2900_ERR("No dev supplied in private data");
		return POLLERR | POLLRDHUP;
	}

	if (RESET == audio_info->state)
		mask |= POLLERR | POLLRDHUP | POLLPRI;
	else
		/* Unless RESET we can transmit */
		mask |= POLLOUT;

	if (dev->stored_data)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations char_dev_fops = {
	.open = audio_dev_open,
	.release = audio_dev_release,
	.read = audio_dev_read,
	.write = audio_dev_write,
	.poll = audio_dev_poll
};

/*
 *	Module related methods
 */

/**
 * cg2900_audio_probe() - Initialize CG2900 audio resources.
 * @pdev:	Platform device.
 *
 * Initialize the module and register misc device.
 *
 * Returns:
 *   0 if there is no error.
 *   -ENOMEM if allocation fails.
 *   -EEXIST if device has already been started.
 *   Error codes from misc_register.
 */
static int __devinit cg2900_audio_probe(struct platform_device *pdev)
{
	int err;

	CG2900_INFO("cg2900_audio_probe");

	if (audio_info) {
		CG2900_ERR("ST-Ericsson CG2900 Audio driver already initiated");
		return -EEXIST;
	}

	/* Initialize private data. */
	audio_info = kzalloc(sizeof(*audio_info), GFP_KERNEL);
	if (!audio_info) {
		CG2900_ERR("Could not alloc audio_info struct.");
		return -ENOMEM;
	}

	/* Initiate the mutexes */
	mutex_init(&(audio_info->management_mutex));
	mutex_init(&(audio_info->bt_mutex));
	mutex_init(&(audio_info->fm_mutex));
	mutex_init(&(audio_info->endpoints.management_mutex));

	/* Initiate the SKB queues */
	skb_queue_head_init(&(audio_info->bt_queue));
	skb_queue_head_init(&(audio_info->fm_queue));

	/* Initiate the endpoint list */
	INIT_LIST_HEAD(&(audio_info->endpoints.ep_list));

	/* Prepare and register MISC device */
	audio_info->dev.minor = MISC_DYNAMIC_MINOR;
	audio_info->dev.name = DEVICE_NAME;
	audio_info->dev.fops = &char_dev_fops;
	audio_info->dev.parent = &(pdev->dev);

	err = misc_register(&(audio_info->dev));
	if (err) {
		CG2900_ERR("Error %d registering misc dev!", err);
		goto error_handling;
	}

	return 0;

error_handling:
	mutex_destroy(&audio_info->management_mutex);
	mutex_destroy(&audio_info->bt_mutex);
	mutex_destroy(&audio_info->fm_mutex);
	mutex_destroy(&audio_info->endpoints.management_mutex);
	kfree(audio_info);
	audio_info = NULL;
	return err;
}

/**
 * cg2900_audio_remove() - Release CG2900 audio resources.
 * @pdev:	Platform device.
 *
 * Remove misc device and free resources.
 *
 * Returns:
 *   0 if success.
 *   Error codes from misc_deregister.
 */
static int __devexit cg2900_audio_remove(struct platform_device *pdev)
{
	int err;

	CG2900_INFO("cg2900_audio_remove");

	if (!audio_info)
		return 0;

	err = misc_deregister(&audio_info->dev);
	if (err)
		CG2900_ERR("Error deregistering misc dev (%d)!", err);

	mutex_destroy(&audio_info->management_mutex);
	mutex_destroy(&audio_info->bt_mutex);
	mutex_destroy(&audio_info->fm_mutex);

	flush_endpoint_list(&(audio_info->endpoints));

	skb_queue_purge(&(audio_info->bt_queue));
	skb_queue_purge(&(audio_info->fm_queue));

	mutex_destroy(&audio_info->endpoints.management_mutex);

	kfree(audio_info);
	audio_info = NULL;
	return err;
}

static struct platform_driver cg2900_audio_driver = {
	.driver = {
		.name	= "cg2900-audio",
		.owner	= THIS_MODULE,
	},
	.probe	= cg2900_audio_probe,
	.remove	= __devexit_p(cg2900_audio_remove),
};

/**
 * cg2900_audio_init() - Initialize module.
 *
 * Registers platform driver.
 */
static int __init cg2900_audio_init(void)
{
	CG2900_INFO("cg2900_audio_init");
	return platform_driver_register(&cg2900_audio_driver);
}

/**
 * cg2900_audio_exit() - Remove module.
 *
 * Unregisters platform driver.
 */
static void __exit cg2900_audio_exit(void)
{
	CG2900_INFO("cg2900_audio_exit");
	platform_driver_unregister(&cg2900_audio_driver);
}

module_init(cg2900_audio_init);
module_exit(cg2900_audio_exit);

MODULE_AUTHOR("Par-Gunnar Hjalmdahl ST-Ericsson");
MODULE_AUTHOR("Kjell Andersson ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Linux Bluetooth Audio ST-Ericsson controller");
