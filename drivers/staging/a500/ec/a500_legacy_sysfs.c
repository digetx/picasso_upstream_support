/*
 * Legacy sysfs for Android compatibility
 *
 * based on Acer EC battery driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>

#include "ec.h"

#define BTMAC_PARTS_NB		3
#define CABC_MASK		BIT(8)
#define GYRO_GAIN_PARTS_NB	18
#define PART_SZ			4
#define SYSCONF_MASK		0x0000FFFF
#define WIFIMAC_PARTS_NB	3

#define EC_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute _name##_attr = __ATTR(_name, _mode, _show, _store)

/*				addr	timeout */
EC_REG_DATA(RESET_LED,		0x40,	100);
EC_REG_DATA(LEDS_OFF,		0x41,	100);
EC_REG_DATA(POWER_LED_ON,	0x42,	100);
EC_REG_DATA(CHARGE_LED_ON,	0x43,	100);
EC_REG_DATA(AUDIO_CTRL,		0x44,	0);
EC_REG_DATA(POWER_CTRL_3G,	0x45,	100);
EC_REG_DATA(GPS_POWER_OFF,	0x47,	0);
EC_REG_DATA(GPS_3G_STATUS_RD,	0x48,	0);
EC_REG_DATA(GPS_3G_STATUS_WR,	0x49,	0);
EC_REG_DATA(GPS_POWER_ON,	0x4A,	0);
EC_REG_DATA(MISC_CTRL_RD,	0x4C,	10);
EC_REG_DATA(MISC_CTRL_WR,	0x4D,	10);
EC_REG_DATA(ANDROID_LEDS_OFF,	0x5A,	100);
EC_REG_DATA(BTMAC_RD,		0x62,	10);
EC_REG_DATA(BTMAC_WR,		0x63,	10);
EC_REG_DATA(WIFIMAC_RD,		0x64,	10);
EC_REG_DATA(WIFIMAC_WR,		0x65,	10);
EC_REG_DATA(LS_GAIN_RD,		0x71,	10);
EC_REG_DATA(LS_GAIN_WR,		0x72,	10);
EC_REG_DATA(GYRO_GAIN_RD,	0x73,	10);
EC_REG_DATA(GYRO_GAIN_WR,	0x74,	10);

static int power_state_3g;
static int power_state_gps;

static void ec_read_multipart(char *buf, struct ec_reg_data *reg_data,
			      int parts_nb)
{
	char *write_offset;
	int length, i;
	s32 ret;

	length = parts_nb * PART_SZ;
	write_offset = buf + length;

	ec_lock();
	for (i = 0; i < parts_nb; i++) {
		ret = ec_read_word_data_locked(reg_data);

		write_offset -= PART_SZ;

		snprintf(write_offset, length + 1, "%04x%s",
			 ret, (i == 0) ? "" : write_offset + PART_SZ);
	}
	ec_unlock();
}

static void ec_write_multipart(const char *buf, struct ec_reg_data *reg_data,
			       int parts_nb)
{
	char part_buf[PART_SZ + 1];
	int read_offset, val, i;
	s32 ret;

	/* don't count trailing "\n" */
	ret = strlen(buf) - 1;

	if (ret != parts_nb * PART_SZ) {
		pr_err("%s: length %d is not equal to required %d\n",
			  __func__, ret, parts_nb * PART_SZ);
		return;
	}

	ec_lock();
	for (i = 0; i < parts_nb; i++) {
		read_offset = (parts_nb - i + 1) * PART_SZ;

		snprintf(part_buf, ARRAY_SIZE(part_buf),
			 "%s", buf + read_offset);

		ret = kstrtoint(part_buf, 16, &val);
		if (ret < 0)
			pr_err("%s: failed to convert hex str: %s\n",
				__func__, part_buf);

		ec_write_word_data_locked(reg_data, val);
	}
	ec_unlock();
}

static ssize_t gyro_show(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	ec_read_multipart(buf, GYRO_GAIN_RD, GYRO_GAIN_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t gyro_store(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	ec_write_multipart(buf, GYRO_GAIN_WR, GYRO_GAIN_PARTS_NB);

	return n;
}

static ssize_t pwr_led_on_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	ec_write_word_data(POWER_LED_ON, 0);

	return n;
}

static ssize_t chrg_led_on_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t n)
{
	ec_write_word_data(CHARGE_LED_ON, 0);

	return n;
}

static ssize_t reset_led_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	ec_write_word_data(RESET_LED, 0);

	return n;
}

static ssize_t leds_off_store(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      const char *buf, size_t n)
{
	ec_write_word_data(LEDS_OFF, 0);

	return n;
}

static ssize_t android_off_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t n)
{
	ec_write_word_data(ANDROID_LEDS_OFF, 0);

	return n;
}

static ssize_t ls_gain_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	s32 ret = ec_read_word_data(LS_GAIN_RD);

	return sprintf(buf, "%04x\n", ret);
}

static ssize_t ls_gain_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t n)
{
	int val;
	s32 ret;

	ret = kstrtoint(buf, 16, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert hex str: %s\n", __func__, buf);
		return ret;
	}

	ec_write_word_data(LS_GAIN_WR, val);

	return n;
}

static ssize_t btmac_show(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  char *buf)
{
	ec_read_multipart(buf, BTMAC_RD, BTMAC_PARTS_NB);

	return sprintf(buf, "%c%c:%c%c:%c%c:%c%c:%c%c:%c%c\n",
			buf[0], buf[1], buf[2], buf[3], buf[4],
			buf[5], buf[6], buf[7], buf[8], buf[9],
			buf[10], buf[11]);
}

static ssize_t btmac_store(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	ec_write_multipart(buf, BTMAC_WR, BTMAC_PARTS_NB);

	return n;
}

static ssize_t wifimac_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	ec_read_multipart(buf, WIFIMAC_RD, WIFIMAC_PARTS_NB);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t wifimac_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t n)
{
	ec_write_multipart(buf, WIFIMAC_WR, WIFIMAC_PARTS_NB);

	return n;
}

static ssize_t device_status_show(struct kobject *kobj,
				  struct kobj_attribute *attr,  char *buf)
{
	s32 ret;
	int i;

	ret = ec_read_word_data(GPS_3G_STATUS_RD);

	for (i = 15; i >= 0; i--)
		buf[i] = ret >> (15 - i) & 0x1 ? '1' : '0';

	return sprintf(buf, "%s\n", buf);
}

static ssize_t device_status_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}

	ec_write_word_data(GPS_3G_STATUS_WR, val);

	return n;
}

static ssize_t status_3g_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_state_3g);
}

static ssize_t status_3g_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}

	power_state_3g = val;
	ec_write_word_data(POWER_CTRL_3G, val);

	return n;
}

static ssize_t status_gps_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_state_gps);
}

static ssize_t status_gps_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}

	power_state_gps = !!val;

	if (power_state_gps)
		ec_write_word_data(GPS_POWER_ON, 0);
	else
		ec_write_word_data(GPS_POWER_OFF, 0);

	return n;
}

static ssize_t cabc_show(struct kobject *kobj,
			 struct kobj_attribute *attr, char *buf)
{
	bool enabled = ec_read_word_data(MISC_CTRL_RD) & CABC_MASK;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t cabc_store(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}

	ret = ec_read_word_data(MISC_CTRL_RD);
	if (ret < 0)
		return ret;

	if (val)
		ret |= CABC_MASK;
	else
		ret &= (~CABC_MASK);

	ec_write_word_data(MISC_CTRL_WR, ret);

	return n;
}

static ssize_t sysconf_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	s32 ret = ec_read_word_data(MISC_CTRL_RD) & SYSCONF_MASK;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t sysconf_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}
	val &= SYSCONF_MASK;

	ec_write_word_data(MISC_CTRL_WR, val);

	return n;
}

static ssize_t audioconf_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	s32 ret = ec_read_word_data(AUDIO_CTRL) & SYSCONF_MASK;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t audioconf_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	s32 ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: failed to convert str: %s\n", __func__, buf);
		return ret;
	}
	val &= SYSCONF_MASK;

	ec_write_word_data(AUDIO_CTRL, val);

	return n;
}

static EC_ATTR(GyroGain,
	       S_IWUSR|S_IRUGO, gyro_show, gyro_store);
static EC_ATTR(PowerLED,
	       S_IRUGO, NULL, pwr_led_on_store);
static EC_ATTR(ChargeLED,
	       S_IRUGO, NULL, chrg_led_on_store);
static EC_ATTR(OriSts,
	       S_IRUGO, NULL, reset_led_store);
static EC_ATTR(OffLED,
	       S_IRUGO, NULL, leds_off_store);
static EC_ATTR(LEDAndroidOff,
	       S_IRUGO, NULL, android_off_store);
static EC_ATTR(AutoLSGain,
	       S_IWUSR|S_IRUGO, ls_gain_show, ls_gain_store);
static EC_ATTR(BTMAC,
	       S_IWUSR|S_IRUGO, btmac_show, btmac_store);
static EC_ATTR(WIFIMAC,
	       S_IWUSR|S_IRUGO, wifimac_show, wifimac_store);
static EC_ATTR(DeviceStatus,
	       S_IWUSR|S_IRUGO, device_status_show, device_status_store);
static EC_ATTR(ThreeGPower,
	       S_IWUSR|S_IRUGO, status_3g_show, status_3g_store);
static EC_ATTR(GPSPower,
	       S_IWUSR|S_IRUGO, status_gps_show, status_gps_store);
static EC_ATTR(Cabc,
	       S_IWUSR|S_IRUGO, cabc_show, cabc_store);
static EC_ATTR(SystemConfig,
	       S_IWUSR|S_IRUGO, sysconf_show, sysconf_store);
static EC_ATTR(MicSwitch,
	       S_IWUSR|S_IRUGO, audioconf_show, audioconf_store);

static struct attribute *ec_attrs[] = {
	&GyroGain_attr.attr,
	&PowerLED_attr.attr,
	&ChargeLED_attr.attr,
	&OriSts_attr.attr,
	&OffLED_attr.attr,
	&LEDAndroidOff_attr.attr,
	&AutoLSGain_attr.attr,
	&BTMAC_attr.attr,
	&WIFIMAC_attr.attr,
	&DeviceStatus_attr.attr,
	&ThreeGPower_attr.attr,
	&GPSPower_attr.attr,
	&Cabc_attr.attr,
	&SystemConfig_attr.attr,
	&MicSwitch_attr.attr,
	NULL,
};

static struct attribute_group ec_attr_group = {
	.attrs = ec_attrs,
};

static struct kobject *ec_legacy_kobj;

int ec_create_legacy_sysfs(void)
{
	int ret;

	ec_legacy_kobj = kobject_create_and_add("EcControl", NULL);
	if (!ec_legacy_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(ec_legacy_kobj, &ec_attr_group);
	if (ret)
		kobject_put(ec_legacy_kobj);

	return ret;
}

void ec_release_legacy_sysfs(void)
{
	sysfs_remove_group(ec_legacy_kobj, &ec_attr_group);
	kobject_put(ec_legacy_kobj);
}

module_init(ec_create_legacy_sysfs);
module_exit(ec_release_legacy_sysfs);

MODULE_DESCRIPTION("Acer A50x EC legacy sysfs module");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL");
