/* LIS3DH 3-axis accelerometer driver
 *
 * STMicroelectronics LIS3DH connected on I2C (bus 2 / i2c-6, address 0x19)
 * for the MediaTek accelerometer framework (hwmsensor v2 control path).
 *
 * This driver registers its control/data path with
 * drivers/misc/mediatek/accelerometer/accel.c which in turn creates
 * /dev/m_acc_misc (+ accactive/accdelay/accdevnum sysfs attributes) and the
 * "m_acc_input" input device that the android sensors HAL (sensors.mt8173.so)
 * uses for the accelerometer.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/mutex.h>

#include <cust_acc.h>
#include <accel.h>

#include "lis3dh.h"

#define LIS3DH_DEV_NAME		"gsensor"	/* i2c client name (DT node gsensor@19) */
#define LIS3DH_DTS_COMPAT	"mediatek,lis3dh"	/* cust_accel@0 node */
#define LIS3DH_DTS_COMPAT2	"mediatek,gsensor"	/* gsensor@19 node */

#define LIS3DH_TAG		"<LIS3DH> "
#define LIS3DH_ERR(fmt, args...)	pr_err(LIS3DH_TAG fmt, ##args)
#define LIS3DH_LOG(fmt, args...)	pr_debug(LIS3DH_TAG fmt, ##args)
#define LIS3DH_FUN()			LIS3DH_LOG("%s\n", __func__)

#define LIS3DH_AXIS_X		0
#define LIS3DH_AXIS_Y		1
#define LIS3DH_AXIS_Z		2
#define LIS3DH_VENDER_DIV	1000	/* mg * GRAVITY_EARTH_1000 / 1000 = mm/s^2 */

struct lis3dh_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t trace;
	atomic_t suspend;
	atomic_t power;
	atomic_t delay;		/* ms, poll period */
	struct mutex lock;
};

static struct acc_hw lis3dh_hw_builtin = {
	.i2c_num = 2,			/* i2c_num 2 -> /sys/bus/i2c/devices/i2c-6 */
	.direction = 7,			/* matches DTS cust_accel@0 */
	.power_id = -1,			/* MT65XX_POWER_NONE -> always on */
	.power_vol = 0,
	.firlen = 0,
	.i2c_addr = { LIS3DH_I2C_SLAVE_ADDR, 0x00 },
	.is_batch_supported = false,
};

static struct acc_hw *hw;
static struct lis3dh_i2c_data *obj_i2c_data;
static int lis3dh_init_flag = -1;
static DEFINE_MUTEX(lis3dh_mutex);

/*----------------------------------------------------------------------------*/
/* i2c helpers (MTK hwmsen helpers, same as every other MTK gsensor driver)   */
/*----------------------------------------------------------------------------*/
static int lis3dh_reg_read_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	return hwmsen_read_byte(client, reg, val);
}

static int lis3dh_reg_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	return hwmsen_write_byte(client, reg, val);
}

/* multi byte read: LIS3DH needs MSB of the sub address set for auto increment */
static int lis3dh_reg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	return hwmsen_read_block(client, reg | LIS3DH_AUTO_INC, buf, len);
}

/*----------------------------------------------------------------------------*/
static void lis3dh_power(struct lis3dh_i2c_data *obj, bool on)
{
	if (obj->hw && obj->hw->power)
		obj->hw->power(obj->hw, on ? 1 : 0, "lis3dh");
	else
		LIS3DH_LOG("power %d (no power hook, always on)\n", on);
}

static int lis3dh_init_client(struct i2c_client *client, int reset)
{
	u8 id = 0;
	u8 val = 0;
	int err = 0;

	err = lis3dh_reg_read_byte(client, LIS3DH_REG_WHO_AM_I, &id);
	if (err) {
		LIS3DH_ERR("read WHO_AM_I fail (%d)\n", err);
		return err;
	}
	LIS3DH_ERR("%s: WHO_AM_I = 0x%02X (expect 0x%02X), addr=0x%02X\n",
		   __func__, id, LIS3DH_WHO_AM_I_VALUE, client->addr);
	if (id != LIS3DH_WHO_AM_I_VALUE) {
		LIS3DH_ERR("device id mismatch, not a LIS3DH\n");
		return -ENODEV;
	}

	/* CTRL_REG2/3: no high pass filter, no interrupt on INT1/INT2 */
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG2, 0x00);
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG3, 0x00);
	/* CTRL_REG4: BDU=1, HR=1, FS=+-2g, little endian */
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG4, LIS3DH_CTRL4_BDU_HR_2G);
	/* CTRL_REG5/6: no FIFO, no INT routing */
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG5, 0x00);
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG6, 0x00);
	lis3dh_reg_write_byte(client, LIS3DH_REG_FIFO_CTRL_REG, 0x00);
	/* CTRL_REG1: 100Hz, normal mode (12bit), XYZ enabled */
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG1, LIS3DH_ODR_100HZ);
	mdelay(10);

	lis3dh_reg_read_byte(client, LIS3DH_REG_CTRL_REG1, &val);
	LIS3DH_ERR("after init: CTRL_REG1=0x%02X CTRL_REG4=0x%02X\n",
		   val, LIS3DH_CTRL4_BDU_HR_2G);

	/* stay in power down until the HAL enables the sensor */
	lis3dh_reg_write_byte(client, LIS3DH_REG_CTRL_REG1, LIS3DH_ODR_POWERDOWN);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lis3dh_open_report_data(int open)
{
	LIS3DH_LOG("open_report_data(%d)\n", open);
	return 0;
}

static int lis3dh_enable_nodata(int en)
{
	struct lis3dh_i2c_data *obj = obj_i2c_data;
	int err = 0;
	u8 odr = LIS3DH_ODR_100HZ;
	int delay;

	if (!obj || !obj->client)
		return -1;

	delay = atomic_read(&obj->delay);
	if (delay >= 100)
		odr = LIS3DH_ODR_10HZ;
	else if (delay >= 20)
		odr = LIS3DH_ODR_50HZ;

	LIS3DH_ERR("enable_nodata(%d) odr=0x%02X\n", en, en ? odr : 0);
	mutex_lock(&obj->lock);
	if (en) {
		lis3dh_power(obj, true);
		err = lis3dh_reg_write_byte(obj->client, LIS3DH_REG_CTRL_REG4,
					    LIS3DH_CTRL4_BDU_HR_2G);
		if (err == 0)
			err = lis3dh_reg_write_byte(obj->client,
						    LIS3DH_REG_CTRL_REG1, odr);
		atomic_set(&obj->power, 1);
	} else {
		lis3dh_reg_write_byte(obj->client, LIS3DH_REG_CTRL_REG1,
				      LIS3DH_ODR_POWERDOWN);
		atomic_set(&obj->power, 0);
		lis3dh_power(obj, false);
	}
	mutex_unlock(&obj->lock);

	return (err == 0) ? 0 : -1;
}

static int lis3dh_set_delay(u64 ns)
{
	struct lis3dh_i2c_data *obj = obj_i2c_data;
	int value;
	u8 odr = LIS3DH_ODR_100HZ;

	if (!obj)
		return -1;

	value = (int)ns / 1000 / 1000;	/* ms */
	if (value <= 0)
		value = 5;
	if (value >= 100)
		odr = LIS3DH_ODR_10HZ;
	else if (value >= 20)
		odr = LIS3DH_ODR_50HZ;

	atomic_set(&obj->delay, value);

	if (atomic_read(&obj->power)) {
		mutex_lock(&obj->lock);
		lis3dh_reg_write_byte(obj->client, LIS3DH_REG_CTRL_REG1, odr);
		mutex_unlock(&obj->lock);
	}
	LIS3DH_ERR("set_delay(%d ms -> odr 0x%02X)\n", value, odr);
	return 0;
}

static int lis3dh_get_data(int *x, int *y, int *z, int *status)
{
	struct lis3dh_i2c_data *obj = obj_i2c_data;
	u8 buf[6];
	int raw[LIS3DH_AXIS_Z + 1];
	int acc[LIS3DH_AXIS_Z + 1];
	int err = 0;

	if (!obj || !obj->client)
		return -1;

	mutex_lock(&obj->lock);
	err = lis3dh_reg_read_block(obj->client, LIS3DH_REG_OUT_X_L, buf, 6);
	mutex_unlock(&obj->lock);
	if (err) {
		LIS3DH_ERR("read sensor data fail (%d)\n", err);
		return -1;
	}

	/* 12-bit left justified output, 1 mg/LSB @ +-2g */
	raw[LIS3DH_AXIS_X] = (s16)((buf[1] << 8) | buf[0]) >> 4;
	raw[LIS3DH_AXIS_Y] = (s16)((buf[3] << 8) | buf[2]) >> 4;
	raw[LIS3DH_AXIS_Z] = (s16)((buf[5] << 8) | buf[4]) >> 4;

	/* remap axis according to the DTS 'direction' property */
	acc[obj->cvt.map[LIS3DH_AXIS_X]] =
	    obj->cvt.sign[LIS3DH_AXIS_X] * raw[LIS3DH_AXIS_X];
	acc[obj->cvt.map[LIS3DH_AXIS_Y]] =
	    obj->cvt.sign[LIS3DH_AXIS_Y] * raw[LIS3DH_AXIS_Y];
	acc[obj->cvt.map[LIS3DH_AXIS_Z]] =
	    obj->cvt.sign[LIS3DH_AXIS_Z] * raw[LIS3DH_AXIS_Z];

	/* mg -> mm/s^2, vender_div (accactive) = 1000 */
	*x = acc[LIS3DH_AXIS_X] * GRAVITY_EARTH_1000 / LIS3DH_VENDER_DIV;
	*y = acc[LIS3DH_AXIS_Y] * GRAVITY_EARTH_1000 / LIS3DH_VENDER_DIV;
	*z = acc[LIS3DH_AXIS_Z] * GRAVITY_EARTH_1000 / LIS3DH_VENDER_DIV;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	LIS3DH_LOG("raw(%d %d %d) -> (%d %d %d)\n",
		   raw[0], raw[1], raw[2], *x, *y, *z);

	return 0;
}

/*----------------------------------------------------------------------------*/
/* raw data debug attribute: cat /sys/bus/i2c/devices/6-0019/rawdata          */
/*----------------------------------------------------------------------------*/
static ssize_t lis3dh_show_rawdata(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int x, y, z, status;
	int err;

	if (!obj)
		return snprintf(buf, PAGE_SIZE, "no driver data\n");

	err = lis3dh_get_data(&x, &y, &z, &status);
	if (err)
		return snprintf(buf, PAGE_SIZE, "read fail (%d)\n", err);

	return snprintf(buf, PAGE_SIZE, "x=%d y=%d z=%d status=%d (mm/s2, div=%d)\n",
			x, y, z, status, LIS3DH_VENDER_DIV);
}

static DEVICE_ATTR(rawdata, S_IRUGO, lis3dh_show_rawdata, NULL);

static struct attribute *lis3dh_attributes[] = {
	&dev_attr_rawdata.attr,
	NULL
};

static struct attribute_group lis3dh_attribute_group = {
	.attrs = lis3dh_attributes
};

/*----------------------------------------------------------------------------*/
static int lis3dh_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lis3dh_i2c_data *obj;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	int err = 0;

	LIS3DH_ERR("probe: client=%s addr=0x%02X irq=%d\n",
		   client->name, client->addr, client->irq);

	/*
	 * Some MTK drivers (e.g. mc3410 which is part of the always built
	 * accelerometer directory) mutate client->addr while auto probing and
	 * leak the change when their probe fails. Make sure we talk to the
	 * right slave address.
	 */
	if (client->addr != LIS3DH_I2C_SLAVE_ADDR) {
		LIS3DH_ERR("client addr was 0x%02X, restoring to 0x%02X\n",
			   client->addr, LIS3DH_I2C_SLAVE_ADDR);
		client->addr = LIS3DH_I2C_SLAVE_ADDR;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	obj->client = client;
	obj->hw = hw ? hw : &lis3dh_hw_builtin;
	i2c_set_clientdata(client, obj);
	mutex_init(&obj->lock);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->power, 0);
	atomic_set(&obj->delay, 200);

	if (hwmsen_get_convert(obj->hw->direction, &obj->cvt)) {
		LIS3DH_ERR("invalid direction: %d\n", obj->hw->direction);
		err = -EINVAL;
		goto exit_kfree;
	}
	LIS3DH_ERR("direction=%d -> sign[%d %d %d] map[%d %d %d]\n",
		   obj->hw->direction, obj->cvt.sign[0], obj->cvt.sign[1],
		   obj->cvt.sign[2], obj->cvt.map[0], obj->cvt.map[1],
		   obj->cvt.map[2]);

	obj_i2c_data = obj;

	lis3dh_power(obj, true);
	if (lis3dh_init_client(client, 1) != 0) {
		err = -ENODEV;
		goto exit_kfree;
	}

	ctl.open_report_data = lis3dh_open_report_data;
	ctl.enable_nodata = lis3dh_enable_nodata;
	ctl.set_delay = lis3dh_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;

	err = acc_register_control_path(&ctl);
	if (err) {
		LIS3DH_ERR("register control path fail (%d)\n", err);
		goto exit_kfree;
	}

	data.get_data = lis3dh_get_data;
	data.vender_div = LIS3DH_VENDER_DIV;
	err = acc_register_data_path(&data);
	if (err) {
		LIS3DH_ERR("register data path fail (%d)\n", err);
		goto exit_kfree;
	}

	err = sysfs_create_group(&client->dev.kobj, &lis3dh_attribute_group);
	if (err)
		LIS3DH_ERR("create rawdata attribute fail (%d)\n", err);

	lis3dh_init_flag = 0;
	LIS3DH_ERR("probe done, lis3dh registered\n");
	return 0;

exit_kfree:
	if (obj_i2c_data == obj)
		obj_i2c_data = NULL;
	kfree(obj);
exit:
	lis3dh_init_flag = -1;
	LIS3DH_ERR("probe fail (%d)\n", err);
	return err;
}

static int lis3dh_i2c_remove(struct i2c_client *client)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &lis3dh_attribute_group);
	if (obj) {
		lis3dh_power(obj, false);
		if (obj_i2c_data == obj)
			obj_i2c_data = NULL;
		kfree(obj);
	}
	lis3dh_init_flag = -1;
	return 0;
}

#ifdef CONFIG_PM
static int lis3dh_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	if (obj)
		atomic_set(&obj->suspend, 1);
	return 0;
}

static int lis3dh_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	if (obj)
		atomic_set(&obj->suspend, 0);
	return 0;
}

static const struct dev_pm_ops lis3dh_pm_ops = {
	.suspend = lis3dh_i2c_suspend,
	.resume = lis3dh_i2c_resume,
};
#endif

static const struct i2c_device_id lis3dh_i2c_id[] = {
	{ LIS3DH_DEV_NAME, 0 },	/* i2c client created from DT node gsensor@19 */
	{ "lis3dh", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis3dh_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lis3dh_of_match[] = {
	{ .compatible = LIS3DH_DTS_COMPAT2 },
	{ .compatible = LIS3DH_DTS_COMPAT },
	{ }
};
MODULE_DEVICE_TABLE(of, lis3dh_of_match);
#endif

static struct i2c_driver lis3dh_i2c_driver = {
	.driver = {
		   .name = LIS3DH_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = lis3dh_of_match,
#endif
#ifdef CONFIG_PM
		   .pm = &lis3dh_pm_ops,
#endif
		   },
	.probe = lis3dh_i2c_probe,
	.remove = lis3dh_i2c_remove,
	.id_table = lis3dh_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int lis3dh_local_init(void)
{
	LIS3DH_FUN();

	if (hw && hw->power)
		hw->power(hw, 1, "lis3dh");

	if (i2c_add_driver(&lis3dh_i2c_driver)) {
		LIS3DH_ERR("add i2c driver fail\n");
		return -1;
	}

	if (lis3dh_init_flag == -1) {
		LIS3DH_ERR("device not probed\n");
		return -1;
	}
	return 0;
}

static int lis3dh_remove(void)
{
	LIS3DH_FUN();
	i2c_del_driver(&lis3dh_i2c_driver);
	if (hw && hw->power)
		hw->power(hw, 0, "lis3dh");
	return 0;
}

static struct acc_init_info lis3dh_init_info = {
	.name = "lis3dh",
	.init = lis3dh_local_init,
	.uninit = lis3dh_remove,
};

static int __init lis3dh_init(void)
{
	LIS3DH_FUN();

	hw = get_accel_dts_func(LIS3DH_DTS_COMPAT, &lis3dh_hw_builtin);
	if (!hw) {
		LIS3DH_ERR("no DTS node \"%s\", using built-in defaults\n",
			   LIS3DH_DTS_COMPAT);
		hw = &lis3dh_hw_builtin;
	}
	LIS3DH_ERR("dts: i2c_num=%d i2c_addr=0x%02X direction=%d power_id=%d batch=%d\n",
		   hw->i2c_num, hw->i2c_addr[0], hw->direction, hw->power_id,
		   hw->is_batch_supported);

	acc_driver_add(&lis3dh_init_info);
	return 0;
}

static void __exit lis3dh_exit(void)
{
	i2c_del_driver(&lis3dh_i2c_driver);
}

module_init(lis3dh_init);
module_exit(lis3dh_exit);

MODULE_AUTHOR("h3000s kernel port");
MODULE_DESCRIPTION("LIS3DH accelerometer driver for MTK hwmsensor framework");
MODULE_LICENSE("GPL");
