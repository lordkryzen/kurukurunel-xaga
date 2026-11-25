
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8; typedef uint16_t u16;
#define XAGA_CAM 1
#define LOG_INF(...) do {} while (0)
#define EEPROM_I2C_SLAVE_ADDR 0xa2
#define I2C_M_RD 1
#define I2C_LOCK_SEGMENT 2
#define DW9800V_MOVE_STEPS 16
#define DW9800V_INIT_FOCUS_POS 512
#define DW9800V_MOVE_DELAY_US 8400
#define DW9800V_STABLE_TIME_US 20000
#define DW9800V_CTRL_DELAY_US 10000
#define round_down(x,y) ((x)/(y)*(y))
#define V4L2_CID_FOCUS_ABSOLUTE 1
struct i2c_adapter {int unused;};
struct i2c_client {struct i2c_adapter *adapter; unsigned short addr;};
struct i2c_msg {unsigned short addr,flags,len;u8 *buf;};
struct regulator {int refs;char name;};
struct v4l2_subdev {struct i2c_client *data;void *dev;};
struct v4l2_ctrl {int id,val;};
struct dw9800v_device {
 int vendor_id;bool vin_enabled,vdd_enabled,initialized,power_fault;
 struct v4l2_subdev sd;struct v4l2_ctrl *focus;
 struct regulator *vin,*vdd;void *vcamaf_pinctrl,*vcamaf_on,*vcamaf_off;
};
static int calls,fail_at,short_at,positions,last_pos,active,pm_refs,transfer_index;
static int bus_locked;
static char order[128];static int order_len;
static struct dw9800v_device *current;
static int step(char c) {order[order_len++]=c;assert(order_len<128);return ++calls==fail_at?-EIO:0;}
static void usleep_range(int a,int b){assert(a>0&&b>=a);}
static void msleep(int a){assert(a==20);}
static void *v4l2_get_subdevdata(struct v4l2_subdev *s){return s->data;}
static int regulator_enable(struct regulator *r){int e=step(r->name);if(!e){assert(!r->refs);r->refs++;}return e;}
static int regulator_disable(struct regulator *r){int e=step(r->name+32);if(!e){assert(r->refs==1);r->refs--;}return e;}
static int pinctrl_select_state(void *p,void *s){assert(p);return step(s==current->vcamaf_on?'P':'p');}
static void i2c_lock_bus(struct i2c_adapter *a,int mode){assert(a&&mode==I2C_LOCK_SEGMENT&&!bus_locked);bus_locked=1;}
static void i2c_unlock_bus(struct i2c_adapter *a,int mode){assert(a&&mode==I2C_LOCK_SEGMENT&&bus_locked);bus_locked=0;}
static int __i2c_transfer(struct i2c_adapter *a,struct i2c_msg *m,int n){
 assert(bus_locked&&a&&n==1&&m->addr==0x51);int e=step('R');
 if(!m->flags){assert(m->len==2&&m->buf[0]==0&&m->buf[1]==1);transfer_index=1;}
 else {assert(transfer_index==1&&m->flags==I2C_M_RD&&m->len==1);m->buf[0]=3;transfer_index=0;}
 return e?e:(calls==short_at?0:1);
}
static int i2c_smbus_write_byte_data(struct i2c_client *c,int reg,int value){
 assert(c->addr==0x0c&&reg==2&&value==0x20);return step('S');
}
static int dw9800v_set_position(struct dw9800v_device *d,u16 p){
 assert(d&&p<=1023);positions++;last_pos=p;return step('F');
}
static int dw9800v_init(struct dw9800v_device *d){assert(d);return step('I');}
static struct dw9800v_device *to_dw9800v_vcm(struct v4l2_ctrl *c){(void)c;return current;}
static int pm_runtime_get_if_in_use(void *d){(void)d;if(!active)return 0;pm_refs++;return 1;}
static void pm_runtime_put(void *d){(void)d;assert(pm_refs==1);pm_refs--;}
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
static struct regulator vin,vdd;static struct i2c_adapter adapter;
static struct i2c_client client;static struct v4l2_ctrl focus;
static struct dw9800v_device dev;
static void reset(void){
 memset(&dev,0,sizeof(dev));vin=(struct regulator){0,'A'};vdd=(struct regulator){0,'B'};
 client=(struct i2c_client){&adapter,0x0c};focus=(struct v4l2_ctrl){1,512};
 dev.sd.data=&client;dev.vin=&vin;dev.vdd=&vdd;dev.focus=&focus;
 dev.vcamaf_pinctrl=(void*)1;dev.vcamaf_on=(void*)2;dev.vcamaf_off=(void*)3;current=&dev;
 calls=fail_at=short_at=positions=last_pos=active=pm_refs=transfer_index=order_len=0;memset(order,0,sizeof(order));
}
int main(void){
 int checks=0;
 for(int fail=0;fail<=2;fail++)for(int short_xfer=0;short_xfer<=2;short_xfer++){
  reset();fail_at=fail;short_at=short_xfer;int ret=read_vendor_id(&client,1);
  assert(ret==((fail||short_xfer)?-EIO:3));assert(client.addr==0x0c&&!bus_locked);checks++;
 }
 for(int position=0;position<=1023;position++){
  reset();focus.val=position;assert(dw9800v_release(&dev)==0);
  assert(positions<=32);if(positions)assert(last_pos==512);assert(client.addr==0x0c);checks++;
 }
 /* Every startup operation can fail; all successfully enabled rails unwind. */
 for(int fail=1;fail<=6;fail++){
  reset();fail_at=fail;assert(dw9800v_power_on(&dev)==-EIO);
  assert(vin.refs==0&&vdd.refs==0&&!dev.initialized&&!dev.vin_enabled&&!dev.vdd_enabled);
  assert(!dev.power_fault&&client.addr==0x0c);checks++;
 }
 reset();assert(!dw9800v_power_on(&dev));assert(dev.initialized&&vin.refs==1&&vdd.refs==1);
 assert(!memcmp(order,"ABPRRI",6));calls=order_len=0;memset(order,0,sizeof(order));
 assert(!dw9800v_power_off(&dev));assert(!memcmp(order,"Sabp",4));
 assert(!vin.refs&&!vdd.refs&&!dev.initialized);checks++;
 /* Parking and each shutdown failure still attempt all remaining cleanup. */
 for(int fail=1;fail<=4;fail++){
  reset();assert(!dw9800v_power_on(&dev));calls=order_len=0;fail_at=fail;
  assert(dw9800v_power_off(&dev)==-EIO);assert(calls==4&&dev.power_fault);
  int count=calls;assert(dw9800v_power_on(&dev)==-EIO&&calls==count);
  fail_at=0;assert(!dw9800v_power_off(&dev));assert(!vin.refs&&!vdd.refs);checks++;
 }
 reset();focus.val=200;assert(!dw9800v_set_ctrl(&focus));assert(!positions&&!pm_refs);
 active=1;assert(!dw9800v_set_ctrl(&focus));assert(positions==1&&last_pos==200&&!pm_refs);checks++;
 fail_at=calls+1;assert(dw9800v_set_ctrl(&focus)==-EIO&&!pm_refs);checks++;
 dev.power_fault=true;int count=calls;assert(dw9800v_set_ctrl(&focus)==-EIO&&calls==count);checks++;
 printf("PASS: %d actual-function fault/parking/PM cases\n",checks);
 return 0;
}
