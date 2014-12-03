/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/input/ft5x06_ts.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#include <linux/input-polldev.h>
#include <linux/wakelock.h>
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define CFG_MAX_TOUCH_POINTS	10
#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

#define POINT_READ_BUF	(3 + FT_ONE_TCH_LEN * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5X16_ID		0x0A
#define FT5X06_ID		0x55
#define FT6X06_ID		0x06
#define FT6X36_ID		0x36

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		(54 * 1024)

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0
#define FT_FW_FILE_VENDOR_ID(x)	((x)->data[(x)->size - 1])

#define FT_FW_FILE_MAJ_VER_FT6X36(x)	((x)->data[0x10a])
#define FT_FW_FILE_VENDOR_ID_FT6X36(x)	((x)->data[0x108])

/**
* Application data verification will be run before upgrade flow.
* Firmware image stores some flags with negative and positive value
* in corresponding addresses, we need pick them out do some check to
* make sure the application data is valid.
*/
#define FT_FW_CHECK(x, ts_data) \
	(ts_data->family_id == FT6X36_ID ? \
	(((x)->data[0x104] ^ (x)->data[0x105]) == 0xFF \
	&& ((x)->data[0x106] ^ (x)->data[0x107]) == 0xFF) : \
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& ((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& ((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

struct Upgrade_Info {
       u8 CHIP_ID;
       u8 FTS_NAME[20];
       u8 TPD_MAX_POINTS;
       u8 AUTO_CLB;
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};

struct Upgrade_Info fts_updateinfo[] =
{
       {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
       {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
};

#define FTS_DBG
#ifdef FTS_DBG
#define DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define DBG(fmt, args...) 				do{}while(0)
#endif

struct Upgrade_Info fts_updateinfo_curr;

struct i2c_client *fts_i2c_client = NULL;

#define FTS_GESTRUE

#ifdef FTS_GESTRUE
#define  KEY_GESTURE_U		KEY_U
#define  KEY_GESTURE_UP		KEY_UP
#define  KEY_GESTURE_DOWN		KEY_DOWN
#define  KEY_GESTURE_LEFT		KEY_LEFT 
#define  KEY_GESTURE_RIGHT		KEY_RIGHT
#define  KEY_GESTURE_O		KEY_O
#define  KEY_GESTURE_E		KEY_E
#define  KEY_GESTURE_M		KEY_M 
#define  KEY_GESTURE_L		KEY_L
#define  KEY_GESTURE_W		KEY_W
#define  KEY_GESTURE_S		KEY_S 
#define  KEY_GESTURE_V		KEY_V
#define  KEY_GESTURE_Z		KEY_Z

#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_L		    0x44
#define GESTURE_S		    0x46
#define GESTURE_V		    0x54
#define GESTURE_Z		    0x41

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

struct ts_event {
	u16 au16_x[TPD_MAX_POINTS_10];	/*x coordinate */
	u16 au16_y[TPD_MAX_POINTS_10];	/*y coordinate */
	u8 au8_touch_event[TPD_MAX_POINTS_10];	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[TPD_MAX_POINTS_10];	/*touch ID */
	u16 pressure;
	u8 touch_point;
	u8 point_num;
};

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct ft5x06_ts_platform_data *pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	struct input_dev  *input_prox_dev;
	struct input_polled_dev *input_poll_dev;
	#endif
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

static struct ft5x06_ts_data *fts_wq_data;

static DEFINE_MUTEX(i2c_rw_access);

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#define PROXIMITY_INPUT_DEV_NAME 	"proximity"

#define SENSOR_PROX_TP_USE_WAKELOCK
static struct i2c_client * i2c_prox_client = NULL;
#ifdef SENSOR_PROX_TP_USE_WAKELOCK
static struct wake_lock sensor_prox_tp_wake_lock;
#endif
static DEFINE_MUTEX(tp_prox_sensor_mutex);

static int tp_prox_sensor_opened;
static char tp_prox_sensor_data = 0; // 0 near 1 far
static char tp_prox_sensor_data_changed = 0;
static char tp_prox_pre_sensor_data = 1;

static int is_suspend = 0;
static int is_need_report_pointer = 1;

static void tp_prox_sensor_enable(int enable);
#endif

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	mutex_lock(&i2c_rw_access);
	
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	mutex_lock(&i2c_rw_access);
	
	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
static void tp_prox_sensor_enable(int enable)
{
	u8 state;
	int ret = -1;

    if(i2c_prox_client==NULL)
    	return;


	ft5x0x_read_reg(i2c_prox_client, 0xB0, &state);
	DBG("[proxi_fts]read: 0xb0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
	}else{
		state &= 0x00;	
	}
	ret = ft5x0x_write_reg(i2c_prox_client, 0xB0, &state);
	if(ret < 0)
	{
		DBG("[proxi_fts]write psensor switch command failed\n");
	}
	return;
}

static ssize_t tp_prox_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", tp_prox_sensor_opened);
}

static ssize_t tp_prox_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_data *prox = i2c_get_clientdata(client);
	struct input_dev *input_dev = prox->input_prox_dev;
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	DBG("%s, data=%ld\n",__func__,data);
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);

	mutex_lock(&tp_prox_sensor_mutex);
	tp_prox_sensor_enable((int)data);
	if(data){
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_lock(&sensor_prox_tp_wake_lock);
		#endif
		tp_prox_sensor_opened = 1;
		tp_prox_sensor_data = 1;
		tp_prox_sensor_data_changed = 1;
	}else{
		tp_prox_sensor_opened = 0;
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_unlock(&sensor_prox_tp_wake_lock);
		#endif
	}
	mutex_unlock(&tp_prox_sensor_mutex);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			tp_prox_enable_show, tp_prox_enable_store);

/* Returns currently selected poll interval (in ms) */
static ssize_t tp_prox_get_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_data *ps = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ps->input_poll_dev->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t tp_prox_set_poll_delay(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_data *ps = i2c_get_clientdata(client);
	struct input_dev *input_dev = ps->input_prox_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	ps->input_poll_dev->poll_interval = max((int)interval,(int)ps->input_poll_dev->poll_interval_min);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			tp_prox_get_poll_delay, tp_prox_set_poll_delay);

static struct attribute *tp_prox_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group tp_prox_attribute_group = {
	.attrs = tp_prox_attributes
};

static void __devinit tp_prox_init_input_device(struct fts_data *prox,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	input_dev->name = PROXIMITY_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &prox->client->dev;
}

static void tp_prox_poll(struct input_polled_dev *dev)
{
	struct fts_data *prox = dev->private;

	if (tp_prox_sensor_data_changed){
		mutex_lock(&tp_prox_sensor_mutex);
		tp_prox_sensor_data_changed = 0;
		mutex_unlock(&tp_prox_sensor_mutex);
		DBG("%s poll tp_prox_sensor_data=%d\n",__func__,tp_prox_sensor_data);
		input_report_abs(prox->input_prox_dev, ABS_DISTANCE, tp_prox_sensor_data);
		input_sync(prox->input_prox_dev);
	}
}

static int __devinit tp_prox_setup_polled_device(struct fts_data *ps)
{
	int err;
	struct input_polled_dev *poll_dev;

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		DBG("Failed to allocate polled device\n");
		return -ENOMEM;
	}

	ps->input_poll_dev = poll_dev;
	ps->input_prox_dev = poll_dev->input;

	poll_dev->private = ps;
	poll_dev->poll = tp_prox_poll;
	//poll_dev->open = tp_prox_polled_input_open;
	//poll_dev->close = tp_prox_polled_input_close;
	poll_dev->poll_interval = 100;
	poll_dev->poll_interval_min= 0;

	tp_prox_init_input_device(ps, poll_dev->input);
	err = input_register_polled_device(poll_dev);
	DBG("%s, err=%d, poll-interval=%d\n",__func__,err,poll_dev->poll_interval);
	if (err) {
		DBG("Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return -ENOMEM;
	}

	return 0;
}

static void tp_prox_teardown_polled_device(struct fts_data *ps)
{
	input_unregister_polled_device(ps->input_poll_dev);
	input_free_polled_device(ps->input_poll_dev);
}
#endif

#ifdef FTS_GESTRUE
static void check_gesture(int gesture_id)
{
    printk("fts gesture_id==0x%x\n ",gesture_id);

                switch(gesture_id)
                {
                        case GESTURE_LEFT:
                                input_report_key(tpd->dev, KEY_GESTURE_LEFT, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_LEFT, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_RIGHT:
                                input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 0);
                                input_sync(tpd->dev);
                		    break;
                        case GESTURE_UP:
                                input_report_key(tpd->dev, KEY_GESTURE_UP, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_UP, 0);
                                input_sync(tpd->dev);                                  
                                break;
                        case GESTURE_DOWN:
                                input_report_key(tpd->dev, KEY_GESTURE_DOWN, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_DOWN, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_DOUBLECLICK:
                                input_report_key(tpd->dev, KEY_GESTURE_U, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_U, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_O:
                                input_report_key(tpd->dev, KEY_GESTURE_O, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_O, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_W:
                                input_report_key(tpd->dev, KEY_GESTURE_W, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_W, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_M:
                                input_report_key(tpd->dev, KEY_GESTURE_M, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_M, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_E:
                                input_report_key(tpd->dev, KEY_GESTURE_E, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_E, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_L:
                                input_report_key(tpd->dev, KEY_GESTURE_L, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_L, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_S:
                                input_report_key(tpd->dev, KEY_GESTURE_S, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_S, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_V:
                                input_report_key(tpd->dev, KEY_GESTURE_V, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_V, 0);
                                input_sync(tpd->dev);
                                break;
                        case GESTURE_Z:
                                input_report_key(tpd->dev, KEY_GESTURE_Z, 1);
                                input_sync(tpd->dev);
                                input_report_key(tpd->dev, KEY_GESTURE_Z, 0);
                                input_sync(tpd->dev);
                                break;
                        default:
                                break;
                }

}

static int fts_read_Gestruedata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;

    pointnum = 0;

    ret = ft5x06_i2c_read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
                printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");

    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

	 gestrue_id = buf[0];
	 pointnum = (short)(buf[1]) & 0xff;
	 buf[0] = 0xd3;

	 if((pointnum * 4 + 8)<255)
	 {
	    	 ret = ft5x06_i2c_read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	 }
	 else
	 {
	        ret = ft5x06_i2c_read(i2c_client, buf, 1, buf, 255);
	        ret = ft5x06_i2c_read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	 }
	 if (ret < 0)
	 {
	       printk( "%s read touchdata failed.\n", __func__);
	       return ret;
	 }
	 check_gesture(gestrue_id);
	 for(i = 0;i < pointnum;i++)
	 {
	    	coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
	        	8 | (((s16) buf[1 + (4 * i)])& 0xFF);
	    	coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
	        	8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	 }
	 return -1;
}
#endif
static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *fts_ts = dev_id;

	if (!fts_ts) {
		pr_err("%s: Invalid fts_ts\n", __func__);
		return IRQ_HANDLED;
	}

	queue_work(fts_ts->ts_workqueue, &fts_ts->touch_event_work);

	return IRQ_HANDLED;
}

static int fts_read_Touchdata(struct ft5x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	u8 state;

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	u8 proximity_status;
	#endif

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if (tp_prox_sensor_opened == 1)
	{
		ft5x0x_read_reg(data->client, 0xB0, &state);
		DBG("proxi_fts 0xB0 state value is 0x%02X\n", state);
		if(!(state&0x01))
		{
			tp_prox_sensor_enable(1);
		}
		ft5x0x_read_reg(data->client, 0x01, &proximity_status);
		DBG("proxi_fts value is 0x%02X\n", proximity_status);
		tp_prox_pre_sensor_data = tp_prox_sensor_data;
		if (proximity_status == 0xC0)  //near
		{
			tp_prox_sensor_data = 0;	
		}
		else if(proximity_status == 0xE0)  //far
		{
			tp_prox_sensor_data = 1;
		}
		 DBG( "%s tp_pre_sensor_data=%d,tp_prox_sensor_data=%d\n", __func__,tp_pre_sensor_data,tp_prox_sensor_data);
	 	if( tp_prox_pre_sensor_data != tp_prox_sensor_data)
	 	{  
		    DBG( "%s ensor data changed\n", __func__);
			mutex_lock(&tp_prox_sensor_mutex);
	        tp_prox_sensor_data_changed = 1;
	        mutex_unlock(&tp_prox_sensor_mutex);
			return 1;
	 	}
		if(is_need_report_pointer == 0)
		{
			DBG( ":%s:  we don not report pointer when sleep in call\n", __func__);
			return 1;
		}
		
	}  
	#endif

	#ifdef FTS_GESTRUE
	ft5x0x_read_reg(data->client, 0xB0, &state);
       printk("tpd fts_read_Gestruedata state=%d\n",state);
       if(state ==1)
       {
          fts_read_Gestruedata();
	   return 1;
      }
      #endif
	
	ret = ft5x06_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
	}

	event->pressure = FT_PRESS;

	return 0;
}

static void fts_report_value(struct ft5x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	/*protocol B*/	
	for (i = 0; i < event->touch_point; i++)
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		
		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					event->au16_y[i]);
		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				false);
		}
	}
	if(event->touch_point == uppoint)
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	input_sync(data->input_dev);


}

static void fts_touch_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	ret = fts_read_Touchdata(fts_wq_data);
	if (ret == 0)
		fts_report_value(fts_wq_data);
	
	//enable_irq(fts_wq_data->client->irq);
}

static int ft5x06_gpio_configure(struct ft5x06_ts_data *data, bool on)
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"ft5x06_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			err = gpio_request(data->pdata->reset_gpio,
						"ft5x06_reset_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"reset gpio request failed");
				goto err_irq_gpio_dir;
			}

			err = gpio_direction_output(data->pdata->reset_gpio, 0);
			if (err) {
				dev_err(&data->client->dev,
				"set_direction for reset gpio failed\n");
				goto err_reset_gpio_dir;
			}
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}

		return 0;
	} else {
		if (gpio_is_valid(data->pdata->irq_gpio))
			gpio_free(data->pdata->irq_gpio);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			err = gpio_direction_input(data->pdata->reset_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"unable to set direction for gpio "
					"[%d]\n", data->pdata->irq_gpio);
			}
			gpio_free(data->pdata->reset_gpio);
		}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	ft5x06_data->pinctrl_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_active)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_active);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_suspend);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_release
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_release)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_release);
		dev_dbg(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ft5x06_data->ts_pinctrl);
err_pinctrl_get:
	ft5x06_data->ts_pinctrl = NULL;
	return retval;
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(tp_prox_sensor_opened){
		DBG("tp can not sleep in call\n");
		is_need_report_pointer = 0;
		return;
	}
	#endif

	#ifdef FTS_GESTRUE
                ft5x0x_write_reg(fts_i2c_client, 0xd0, 0x01);
		  if (fts_updateinfo_curr.CHIP_ID==0x54)
		  {
		  	ft5x0x_write_reg(i2c_client, 0xd1, 0xff);
			ft5x0x_write_reg(i2c_client, 0xd2, 0xff);
			ft5x0x_write_reg(i2c_client, 0xd5, 0xff);
			ft5x0x_write_reg(i2c_client, 0xd6, 0xff);
			ft5x0x_write_reg(i2c_client, 0xd7, 0xff);
			ft5x0x_write_reg(i2c_client, 0xd8, 0xff);
		  }

	data->suspended = true;
	
        return;
	#endif
	
	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	disable_irq(data->client->irq);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}

	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}

	err = ft5x06_gpio_configure(data, false);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in suspend state\n");
		goto gpio_configure_fail;
	}

	data->suspended = true;

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_suspend = 1;
	#endif
	
	return 0;

gpio_configure_fail:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err)
			dev_err(dev, "power on failed");
	} else {
		err = ft5x06_power_on(data, true);
		if (err)
			dev_err(dev, "power on failed");
	}
pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_need_report_pointer = 1;
	if((tp_prox_sensor_opened) && (is_suspend == 0)){
		DBG("%s tp no need to wake up in call\n",__func__);
		return;
	}
	#endif
	
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}

	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in resue state\n");
		goto err_gpio_configuration;
	}

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->client->irq);

	data->suspended = false;

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_suspend = 0;
	#endif

	 #ifdef FTS_GESTRUE
                   ft5x0x_write_reg(fts_i2c_client,0xD0,0x00);
        #endif
	
	return 0;

err_gpio_configuration:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err)
			dev_err(dev, "power off failed");
	} else {
		err = ft5x06_power_on(data, false);
		if (err)
			dev_err(dev, "power off failed");
	}
	return err;
}

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};

#else
static int ft5x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

void focaltech_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;
	u8 temp = FT_REG_ID;

	ft5x06_i2c_read(i2c_client,&temp,1,&chip_id);

	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;


	if(pbt_buf[0] != 0x02)
	{
		DBG("[FTS] FW first byte is not 0x02. so it is invalid \n");
		return -1;
	}

	if(dw_lenth > 0x11f)
	{
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if(dw_lenth < fw_length)
		{
			DBG("[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
		DBG("[FTS] Fw length is invalid \n");
		return -1;
	}
	
	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
		
		ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/*write 0x55 to register 0xbc */
		ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_55);

		msleep(fts_updateinfo_curr.delay_55);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, auc_i2c_write_buf, 1);

		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_readid);

		/*********Step 3:check READ-ID***********************/		
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			DBG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FT_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 5);
	
	//auc_i2c_write_buf[0] = 0xcd;
	//ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_earse_flash);

	for(i = 0;i < 200;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(0xb0 == reg_val[0] && 0x02 == reg_val[1])
		{
			DBG("[FTS] erase app finished \n");
			break;
		}
		msleep(50);
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);
		
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				DBG("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0) {
		temp = packet_number * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft5x06_i2c_write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				DBG("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}

int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	
	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
		
		ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/*write 0x55 to register 0xbc */
		ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_55);

		msleep(fts_updateinfo_curr.delay_55);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do {
			i++;
			i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);


		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			DBG("[FTS] Step 3: CTPM ID OK ,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i > FT_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);
		msleep(FT_FW_PKT_LEN / 6 + 1);
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0) {
		temp = packet_number * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft5x06_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}


	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		ft5x06_i2c_write(client, packet_buf, 7);
		msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}

int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32  packet_number;
	u32  j;
	u32  temp;
	u32  lenght;
	u8 	packet_buf[FT_FW_PKT_LEN + 6];
	u8  	auc_i2c_write_buf[10];
	u8  	bt_ecc;
	int	i_ret;
	int	fw_filenth = sizeof(CTPM_FW);

	if(CTPM_FW[fw_filenth-12] == 30)
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
    	/*********Step 1:Reset  CTPM *****/
    	/*write 0xaa to register 0xfc*/
		/*write 0xaa to register 0xfc*/
	   	ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		
		 /*write 0x55 to register 0xfc*/
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);   
		msleep(fts_updateinfo_curr.delay_55);   


		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		
	    i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 2);
	  
	    /*********Step 3:check READ-ID***********************/   
		msleep(fts_updateinfo_curr.delay_readid);
	   	auc_i2c_write_buf[0] = 0x90; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
              ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);           
              if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 
                              && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
              {
                      dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]); 
                      break; 
              }
              else
              {
                      dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAILD,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
                      continue;
              }

          }

          if (i >= FT_UPGRADE_LOOP)
                          return -EIO;
		  
         auc_i2c_write_buf[0] = 0xcd;
         ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);

	/*********20130705 mshl ********************/
	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f)
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}

     /*********Step 4:erase app and panel paramenter area ********************/
	if(is_5336_fwsize_30)
	{
		auc_i2c_write_buf[0] = 0x61;
		ft5x06_i2c_write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		 msleep(fts_updateinfo_curr.delay_earse_flash); 

		 auc_i2c_write_buf[0] = 0x63;
		ft5x06_i2c_write(client, auc_i2c_write_buf, 1); /*erase config area*/	
   		 msleep(50);
	}
	else
	{
		auc_i2c_write_buf[0] = 0x61;
		ft5x06_i2c_write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		msleep(fts_updateinfo_curr.delay_earse_flash); 
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;

	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		dw_lenth = dw_lenth - 8;
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF) dw_lenth = dw_lenth - 14;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FT_FW_PKT_LEN;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		lenght = FT_FW_PKT_LEN;
		packet_buf[4] = (u8)(lenght>>8);
		packet_buf[5] = (u8)lenght;

		for (i=0;i<FT_FW_PKT_LEN;i++)
		{
		    packet_buf[6+i] = pbt_buf[j*FT_FW_PKT_LEN + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}

		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN+6);
		msleep(FT_FW_PKT_LEN/6 + 1);
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0)
	{
		temp = packet_number * FT_FW_PKT_LEN;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		temp = (dw_lenth) % FT_FW_PKT_LEN;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;

		for (i=0;i<temp;i++)
		{
		    packet_buf[6+i] = pbt_buf[ packet_number*FT_FW_PKT_LEN + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}
  
		ft5x06_i2c_write(client, packet_buf, temp+6);
		msleep(20);
	}
        /*send the last six byte*/
        if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
        {
                for (i = 0; i<6; i++)
                {
                                if (is_5336_new_bootloader  == BL_VERSION_Z7 /*&& DEVICE_IC_TYPE==IC_FT5x36*/) 
                                {
                                                temp = 0x7bfa + i;
                                }
                                else if(is_5336_new_bootloader == BL_VERSION_LZ4)
                                {
                                                temp = 0x6ffa + i;
                                }
                                packet_buf[2] = (u8)(temp>>8);
                                packet_buf[3] = (u8)temp;
                                temp =1;
                                packet_buf[4] = (u8)(temp>>8);
                                packet_buf[5] = (u8)temp;
                                packet_buf[6] = pbt_buf[ dw_lenth + i]; 
                                bt_ecc ^= packet_buf[6];
                                ft5x06_i2c_write(client, packet_buf, 7);
                                msleep(10);
                }
        }
        else if(is_5336_new_bootloader == BL_VERSION_GZF)
        {
		for (i = 0; i<12; i++)
		{
		        if (is_5336_fwsize_30 /*&& DEVICE_IC_TYPE==IC_FT5x36*/) 
		        {
		                        temp = 0x7ff4 + i;
		        }
		        else if (1/*DEVICE_IC_TYPE==IC_FT5x36*/) 
		        {
		                        temp = 0x7bf4 + i;
		        }
		        packet_buf[2] = (u8)(temp>>8);
		        packet_buf[3] = (u8)temp;
		        temp =1;
		        packet_buf[4] = (u8)(temp>>8);
		        packet_buf[5] = (u8)temp;
		        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		        bt_ecc ^= packet_buf[6];
			ft5x06_i2c_write(client, packet_buf, 7);
			msleep(10);
		}
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	auc_i2c_write_buf[0] = 0xcc;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1); 

	if(reg_val[0] != bt_ecc)
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
	    	return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);  /*make sure CTP startup normally*/

	return 0;
}

int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	
	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/*write 0x55 to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do {
			i++;
			i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);


		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			DBG("[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FT_UPGRADE_LOOP)
		return -EIO;
	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);
		msleep(FT_FW_PKT_LEN / 6 + 1);
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0) {
		temp = packet_number * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft5x06_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}


	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		ft5x06_i2c_write(client, packet_buf, 7);
		msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}
		
	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */
	return 0;
}

int hid_to_i2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	ft5x06_i2c_write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	ft5x06_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
	{
		bRet = 1;		
	}
	else bRet = 0;

	return bRet;
	
}

int  fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	i_ret = hid_to_i2c(client);

	if(i_ret == 0)
	{
		DBG("[FTS] hid change to i2c fail ! \n");
	}

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		//write 0x55 to register 0xfc 
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hid_to_i2c(client);

		if(i_ret == 0)
		{
			DBG("[FTS] hid change to i2c fail ! \n");
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			DBG("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		
		reg_val[0] = reg_val[1] = 0x00;
		
		ft5x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			
			continue;
		}
	}
	if (i >= FT_UPGRADE_LOOP )
		return -EIO;

	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);	//erase app area 
	msleep(1350);

	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
		
	}

	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);

	ft5x06_i2c_write(client, auc_i2c_write_buf, 4);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
			
		}
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0) {
		temp = packet_number * FT_FW_PKT_LEN;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FT_FW_PKT_LEN;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FT_FW_PKT_LEN + i];
			bt_ecc ^= packet_buf[6 + i];
		}	
		ft5x06_i2c_write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
			
		}
	}

	msleep(50);
	
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1); 
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);
			
	}
	auc_i2c_write_buf[0] = 0x66;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
				
		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);	
	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(130);	//make sure CTP startup normally 

	return 0;
}

/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7


#define PROC_NAME	"ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*interface of write proc*/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	unsigned char writebuf[2 * 1024];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			DBG("%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		DBG("%s: autoclb\n", __func__);
		ft5x06_auto_cal(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	int ret = 0;
	unsigned char buf[PAGE_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ft5x0x_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ft5x0x_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
int fts_create_apk_debug_channel(struct i2c_client * client)
{
	fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	if (NULL == fts_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		fts_proc_entry->data = client;
		fts_proc_entry->write_proc = fts_debug_write;
		fts_proc_entry->read_proc = fts_debug_read;
	}
	return 0;
}

void fts_release_apk_debug_channel(void)
{
	if (fts_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID
			|| ts_data->family_id == FT6X36_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *pbt_buf = NULL;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;
	int i_ret;
	int fw_len = 0;

	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	i_ret= request_firmware(&pbt_buf, data->fw_name, dev);
	if (i_ret < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, i_ret);
		return i_ret;
	}

	if (pbt_buf->size < FT_FW_MIN_SIZE || pbt_buf->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", pbt_buf->size);
		rc = -EIO;
		goto rel_fw;
	}

	fw_len = pbt_buf->size;

	if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)
		||(fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x06) ||(fts_updateinfo_curr.CHIP_ID==0x0a) ||(fts_updateinfo_curr.CHIP_ID==0x08))
	{
		if (fw_len < 8 || fw_len > 32 * 1024) 
		{
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}

		if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
		&& (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
		&& (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) 
		{
			/*FW upgrade */
			/*call the upgrade function */
			if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a))
			{
				i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf->data, fw_len);
			}
			else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14))
			{
				i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf->data, fw_len);
			}
			else if ((fts_updateinfo_curr.CHIP_ID==0x06))
			{
				i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf->data, fw_len);
			}
			if (i_ret != 0)
				dev_err(&client->dev, "%s:upgrade failed. err.\n",
						__func__);
			else if(fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
			{
				fts_ctpm_auto_clb(client);
			}
		} 
		else 
		{
			dev_err(&client->dev, "%s:FW format error\n", __func__);
			return -EBADFD;
		}
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x36))
	{
		if (fw_len < 8 || fw_len > 32 * 1024) 
		{
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf->data, fw_len);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x54))
	{
		if (fw_len < 8 || fw_len > 54 * 1024) 
		{
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf->data, fw_len);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
	}

	/*
	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
		fw_upgrade = true;
	else if ((data->fw_ver[0] < fw_file_maj) &&
		data->fw_vendor_id == fw_file_vendor_id)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	// start firmware upgrade 
	if (FT_FW_CHECK(fw, data)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}
	*/

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(pbt_buf);
	return i_ret;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data->tch_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}

	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
		goto err_gpio_req;
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_gpio;
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_gpio;
	}

	data->family_id = pdata->family_id;

	fts_i2c_client = client;

	focaltech_get_upgrade_array();

	INIT_WORK(&data->touch_event_work, fts_touch_irq_work);
	data->ts_workqueue = create_workqueue(FTS_NAME);
	if (!data->ts_workqueue)
	{
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
				pdata->irqflags | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_gpio;
	}

	disable_irq(client->irq);
	
	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(client);
	#endif

	#ifdef FTS_GESTRUE
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z);
		
	__set_bit(KEY_GESTURE_RIGHT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_UP, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_U, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_O, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_E, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_M, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_W, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_L, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_S, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_V, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_Z, tpd->dev->keybit);
	#endif

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	i2c_prox_client = client;
	#ifdef SENSOR_PROX_TP_USE_WAKELOCK
	wake_lock_init(&sensor_prox_tp_wake_lock, WAKE_LOCK_SUSPEND, "prox_tp");
	#endif

	err = sysfs_create_group(&client->dev.kobj, &tp_prox_attribute_group);
	if (err) {
		DBG( "sysfs create failed: %d\n", err);
	}

	tp_prox_setup_polled_device(ft5x0x_ts);
	#endif
	
#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	fts_wq_data = data;
	enable_irq(client->irq);

	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(data);
err_gpio_req:
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int retval;

	cancel_work_sync(&data->touch_event_work);
	destroy_workqueue(data->ts_workqueue);

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	sysfs_remove_group(&client->dev.kobj, &tp_prox_attribute_group);
	tp_prox_teardown_polled_device(ft5x0x_ts);
	#ifdef SENSOR_PROX_TP_USE_WAKELOCK
	wake_lock_destroy(&sensor_prox_tp_wake_lock);
	#endif
#endif

	#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
	#endif
	
#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (retval < 0)
				pr_err("failed to select release pinctrl state\n");
		}
	}

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
