#if !defined(XAGA_CAM) || defined(MATISSE_CAM) || defined(RUBENS_CAM)
#error "This isolated actuator candidate is xaga-only"
#endif
// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
// Copyright (C) 2022 XiaoMi, Inc.
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "dw9800v"
#define DW9800V_I2C_SLAVE_ADDR 0x18
#define AK7314_I2C_SLAVE_ADDR  0x18
#define EEPROM_I2C_SLAVE_ADDR  0xA2


#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9800V_NAME				"dw9800v"
#define DW9800V_MAX_FOCUS_POS			1023
#define DW9800V_INIT_FOCUS_POS			512
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9800V_FOCUS_STEPS			1
#define DW9800V_SET_POSITION_ADDR		0x03
#define AK7314_SET_POSITION_ADDR		0x00


#define DW9800V_CMD_DELAY			0xff
#define DW9800V_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9800V_MOVE_STEPS			16
#define DW9800V_MOVE_DELAY_US			8400
#define DW9800V_STABLE_TIME_US			20000

/* dw9800v device structure */
struct dw9800v_device {
	int vendor_id;
	bool vin_enabled;
	bool vdd_enabled;
	bool initialized;
	bool power_fault;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};


static int read_vendor_id(struct i2c_client *client, u16 address)
{
	u8 command[2] = { address >> 8, address & 0xff };
	u8 vendor_id;
	struct i2c_msg msg = {
		.addr = EEPROM_I2C_SLAVE_ADDR >> 1,
		.len = sizeof(command), .buf = command,
	};
	int ret;

	/* Keep the two native STOPs and serialize the shared EEPROM pointer. */
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		ret = ret < 0 ? ret : -EIO;
		goto unlock;
	}
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = &vendor_id;
	ret = __i2c_transfer(client->adapter, &msg, 1);
	ret = ret == 1 ? vendor_id : (ret < 0 ? ret : -EIO);
unlock:
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret;
}


static inline struct dw9800v_device *to_dw9800v_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9800v_device, ctrls);
}

static inline struct dw9800v_device *sd_to_dw9800v_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9800v_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int dw9800v_set_position(struct dw9800v_device *dw9800v, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800v->sd);
	int dac_addr   = (dw9800v->vendor_id == 0x03/*semco*/) ?\
		AK7314_SET_POSITION_ADDR : DW9800V_SET_POSITION_ADDR;
	int data_shift = (dw9800v->vendor_id == 0x03/*semco*/) ? 6 : 0;

	return i2c_smbus_write_word_data(client, dac_addr, swab16(val << data_shift));
}

static int dw9800v_release(struct dw9800v_device *dw9800v)
{
#if defined(XAGA_CAM)
	int timeout;
#endif
	int ret, val;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800v->sd);

#if defined(XAGA_CAM)
	timeout = 0;
	val = round_down(dw9800v->focus->val, DW9800V_MOVE_STEPS);
	while (val != DW9800V_INIT_FOCUS_POS) {
		(val > DW9800V_INIT_FOCUS_POS)?(val -= DW9800V_MOVE_STEPS):(val += DW9800V_MOVE_STEPS);
		ret = dw9800v_set_position(dw9800v, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9800V_MOVE_DELAY_US,
			     DW9800V_MOVE_DELAY_US + 1000);

		if (timeout > DW9800V_INIT_FOCUS_POS/DW9800V_MOVE_STEPS) {
			break;
		} else {
			timeout++;
		}
	}
#else
	for (val = round_down(dw9800v->focus->val, DW9800V_MOVE_STEPS);
	     val >= 0; val -= DW9800V_MOVE_STEPS) {
		ret = dw9800v_set_position(dw9800v, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9800V_MOVE_DELAY_US,
			     DW9800V_MOVE_DELAY_US + 1000);
	}
#endif

	ret = i2c_smbus_write_byte_data(client, 0x02, 0x20);
	msleep(20);

	/*
	 * Wait for the motor to stabilize after the last movement
	 * to prevent the motor from shaking.
	 */
	usleep_range(DW9800V_STABLE_TIME_US - DW9800V_MOVE_DELAY_US,
		     DW9800V_STABLE_TIME_US - DW9800V_MOVE_DELAY_US + 1000);

	return ret;
}

static int dw9800v_init(struct dw9800v_device *dw9800v)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800v->sd);
	int ret = 0;
	int i = 0;
	unsigned char cmd_number = 7;
#if defined(MATISSE_CAM)
	char puSendCmdArray[8][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x07}, {0x10, 0x01}, {0xFE, 0xFE},
	};
	cmd_number = 8;
#elif defined(RUBENS_CAM)
	char puSendCmdArray[7][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x01}, {0xFE, 0xFE},
	};
#elif defined(XAGA_CAM)
	char puSendCmdArray[8][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x07}, {0x10, 0x00}, {0xFE, 0xFE},
	};
	cmd_number = 8;
#else
	char puSendCmdArray[7][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x80}, {0x07, 0x7C}, {0xFE, 0xFE},
	};
#endif

	LOG_INF("+\n");

	client->addr = DW9800V_I2C_SLAVE_ADDR >> 1;
	ret = i2c_smbus_read_byte_data(client, 0x00);

	LOG_INF("Check HW version: %x\n", ret);
	if (ret < 0)
		return ret;

	for (i = 0; i < cmd_number; i++) {
		if (puSendCmdArray[i][0] != 0xFE) {
			LOG_INF("0x%02x <= 0x%02x\n", puSendCmdArray[i][0],
				puSendCmdArray[i][1]);
			ret = i2c_smbus_write_byte_data(client,
					puSendCmdArray[i][0],
					puSendCmdArray[i][1]);

			if (ret < 0)
				return ret;
		} else {
			udelay(100);
		}
	}

	LOG_INF("-\n");

	return ret;
}

#ifdef MATISSE_CAM
static int ak7314_init(struct dw9800v_device *dw9800v)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800v->sd);
	int ret = 0;

	LOG_INF("+\n");

	client->addr = AK7314_I2C_SLAVE_ADDR >> 1;
	ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	/* 00:active mode , 10:Standby mode , x1:Sleep mode */
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);

	LOG_INF("-\n");

	return ret;
}
#endif

/* Power handling */
static int dw9800v_power_off(struct dw9800v_device *dw9800v)
{
	int ret, first = 0;

	if (dw9800v->initialized) {
		ret = dw9800v_release(dw9800v);
		if (ret)
			first = ret;
		dw9800v->initialized = false;
	}
	/* Preserve the native vin -> vdd -> pinctrl power-down order. */
	if (dw9800v->vin_enabled) {
		ret = regulator_disable(dw9800v->vin);
		if (!ret)
			dw9800v->vin_enabled = false;
		else if (!first)
			first = ret;
	}
	if (dw9800v->vdd_enabled) {
		ret = regulator_disable(dw9800v->vdd);
		if (!ret)
			dw9800v->vdd_enabled = false;
		else if (!first)
			first = ret;
	}
	if (dw9800v->vcamaf_pinctrl && dw9800v->vcamaf_off) {
		ret = pinctrl_select_state(dw9800v->vcamaf_pinctrl, dw9800v->vcamaf_off);
		if (ret && !first)
			first = ret;
	}
	/* A failed transition is quarantined instead of claiming a known PM state. */
	if (first)
		dw9800v->power_fault = true;
	return first;
}

static int dw9800v_power_on(struct dw9800v_device *dw9800v)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800v->sd);
	int ret;

	if (dw9800v->power_fault || dw9800v->vin_enabled || dw9800v->vdd_enabled)
		return -EIO;
	ret = regulator_enable(dw9800v->vin);
	if (ret)
		return ret;
	dw9800v->vin_enabled = true;
	ret = regulator_enable(dw9800v->vdd);
	if (ret)
		goto fail;
	dw9800v->vdd_enabled = true;
	if (dw9800v->vcamaf_pinctrl && dw9800v->vcamaf_on) {
		ret = pinctrl_select_state(dw9800v->vcamaf_pinctrl, dw9800v->vcamaf_on);
		if (ret)
			goto fail;
	}
	usleep_range(DW9800V_CTRL_DELAY_US, DW9800V_CTRL_DELAY_US + 100);
	ret = read_vendor_id(client, 0x01);
	if (ret < 0)
		goto fail;
	dw9800v->vendor_id = ret;
	LOG_INF("vendor id: %x\n", ret);
	ret = dw9800v_init(dw9800v);
	if (ret < 0)
		goto fail;
	dw9800v->initialized = true;
	return 0;
fail:
	if (dw9800v_power_off(dw9800v))
		LOG_INF("power rollback incomplete; reopen blocked until reboot\n");
	return ret;
}

static int dw9800v_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9800v_device *dw9800v = to_dw9800v_vcm(ctrl);
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return 0;
	if (dw9800v->power_fault)
		return -EIO;
	ret = pm_runtime_get_if_in_use(dw9800v->sd.dev);
	if (ret <= 0)
		return ret < 0 ? ret : 0;
	ret = dw9800v_set_position(dw9800v, ctrl->val);
	LOG_INF("vendor id(0x%x), pos(%d), result(%d)\n", dw9800v->vendor_id, ctrl->val, ret);
	pm_runtime_put(dw9800v->sd.dev);
	return ret;
}

static const struct v4l2_ctrl_ops dw9800v_vcm_ctrl_ops = {
	.s_ctrl = dw9800v_set_ctrl,
};

static int dw9800v_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9800v_device *dw9800v = sd_to_dw9800v_vcm(sd);

	if (dw9800v->power_fault)
		return -EIO;
	return pm_runtime_resume_and_get(sd->dev);
}

static int dw9800v_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	LOG_INF("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9800v_int_ops = {
	.open = dw9800v_open,
	.close = dw9800v_close,
};

static const struct v4l2_subdev_ops dw9800v_ops = { };

static void dw9800v_subdev_cleanup(struct dw9800v_device *dw9800v)
{
	v4l2_async_unregister_subdev(&dw9800v->sd);
	v4l2_ctrl_handler_free(&dw9800v->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9800v->sd.entity);
#endif
}

static int dw9800v_init_controls(struct dw9800v_device *dw9800v)
{
	struct v4l2_ctrl_handler *hdl = &dw9800v->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9800v_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9800v->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9800V_MAX_FOCUS_POS, DW9800V_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9800v->sd.ctrl_handler = hdl;

	return 0;
}

/* Xaga has no supply phandle here; the native named provider can bind late.
 * Optional-get rejects dummy substitution; this rail is mandatory on xaga.
 */
static struct regulator *dw9800v_get_motor_supply(struct device *dev)
{
	struct regulator *supply;

	supply = devm_regulator_get_optional(dev, "fan53870-l6");
	if (IS_ERR(supply) && PTR_ERR(supply) == -ENODEV)
		return ERR_PTR(-EPROBE_DEFER);
	return supply;
}

static int dw9800v_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9800v_device *dw9800v;
	int ret;

	LOG_INF("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	dw9800v = devm_kzalloc(dev, sizeof(*dw9800v), GFP_KERNEL);
	if (!dw9800v)
		return -ENOMEM;

	dw9800v->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9800v->vin)) {
		ret = PTR_ERR(dw9800v->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

#if defined(XAGA_CAM)
	dw9800v->vdd = dw9800v_get_motor_supply(dev);
#else
	dw9800v->vdd = devm_regulator_get(dev, "vdd");
#endif
	if (IS_ERR(dw9800v->vdd)) {
		ret = PTR_ERR(dw9800v->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9800v->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9800v->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9800v->vcamaf_pinctrl);
		if (ret == -EPROBE_DEFER)
			return ret;
		dw9800v->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9800v->vcamaf_on = pinctrl_lookup_state(
			dw9800v->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9800v->vcamaf_on)) {
			ret = PTR_ERR(dw9800v->vcamaf_on);
			dw9800v->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9800v->vcamaf_off = pinctrl_lookup_state(
			dw9800v->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9800v->vcamaf_off)) {
			ret = PTR_ERR(dw9800v->vcamaf_off);
			dw9800v->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9800v->sd, client, &dw9800v_ops);
	dw9800v->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9800v->sd.internal_ops = &dw9800v_int_ops;

	ret = dw9800v_init_controls(dw9800v);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9800v->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9800v->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	/* Initialize PM before async publication can expose the subdev to a user. */
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev(&dw9800v->sd);
	if (ret < 0) {
		pm_runtime_disable(dev);
		goto err_cleanup;
	}
	/* Static-board bring-up: no live unregister/free beneath open subdev files. */
	__module_get(THIS_MODULE);
	return 0;

err_cleanup:
	dw9800v_subdev_cleanup(dw9800v);
	return ret;
}

static void dw9800v_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800v_device *dw9800v = sd_to_dw9800v_vcm(sd);

	LOG_INF("%s\n", __func__);

	dw9800v_subdev_cleanup(dw9800v);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		dw9800v_power_off(dw9800v);
	pm_runtime_set_suspended(&client->dev);

	return;
}

static int __maybe_unused dw9800v_vcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800v_device *dw9800v = sd_to_dw9800v_vcm(sd);

	return dw9800v_power_off(dw9800v);
}

static int __maybe_unused dw9800v_vcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800v_device *dw9800v = sd_to_dw9800v_vcm(sd);

	return dw9800v_power_on(dw9800v);
}

static const struct i2c_device_id dw9800v_id_table[] = {
	{ DW9800V_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9800v_id_table);

static const struct of_device_id dw9800v_of_table[] = {
	{ .compatible = "mediatek,dw9800v" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9800v_of_table);

static const struct dev_pm_ops dw9800v_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(dw9800v_vcm_suspend, dw9800v_vcm_resume, NULL)
};

static struct i2c_driver dw9800v_i2c_driver = {
	.driver = {
		.name = DW9800V_NAME,
		.suppress_bind_attrs = true,
		.pm = &dw9800v_pm_ops,
		.of_match_table = dw9800v_of_table,
	},
	.probe  = dw9800v_probe,
	.remove = dw9800v_remove,
	.id_table = dw9800v_id_table,
};

module_i2c_driver(dw9800v_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9800V VCM driver");
MODULE_LICENSE("GPL v2");
