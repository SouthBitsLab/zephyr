/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icg20660l

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icg20660l.h"
#include "icg20660l_reg.h"

LOG_MODULE_REGISTER(ICG20660L, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Convert raw accelerometer counts to m/s².
 *
 * Each FS setting has a fixed sensitivity in LSB/g:
 *   ±2g  -> 16384 LSB/g = 2^14
 *   ±4g  -> 8192  LSB/g = 2^13
 *   ±8g  -> 4096  LSB/g = 2^12
 *   ±16g -> 2048  LSB/g = 2^11
 * The driver stores the exponent as accel_sensitivity_shift so the conversion
 * becomes a simple integer multiply-and-shift.
 */
static void icg20660l_convert_accel(struct sensor_value *val, int16_t raw_val,
				    uint16_t sensitivity_shift)
{
	int64_t conv_val;

	/* raw * SENSOR_G (µm/s² per g) >> shift gives µm/s². */
	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/*
 * Convert raw gyroscope counts to rad/s.
 *
 * sensitivity_x10 is the datasheet sensitivity multiplied by 10 so the division
 * stays in integer arithmetic.  The formula comes from:
 *   rad/s = raw / (LSB/(dps)) * (π / 180)
 */
static void icg20660l_convert_gyro(struct sensor_value *val, int16_t raw_val,
				   uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/*
 * Convert raw temperature counts to degrees Celsius.
 *
 * The datasheet formula is:
 *   TEMP_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 25
 *
 * The exact constants are not listed in the extracted datasheet pages. The
 * ICG20660L belongs to the same family as the MPU6500, whose typical
 * sensitivity is 333.87 LSB/°C. With RoomTemp_Offset = 0 the formula anchors
 * 0 counts to 25 °C.
 */
static inline void icg20660l_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	int64_t tmp_val;

	/* raw * 1_000_000 / 333.870 gives micro-degrees offset from 25 °C. */
	tmp_val = ((int64_t)raw_val * 1000000) / 333870;
	tmp_val += 25000000;

	val->val1 = tmp_val / 1000000;
	val->val2 = tmp_val % 1000000;
}

static int icg20660l_channel_get(const struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *val)
{
	const struct icg20660l_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icg20660l_convert_accel(val, drv_data->accel_x,
					drv_data->accel_sensitivity_shift);
		icg20660l_convert_accel(val + 1, drv_data->accel_y,
					drv_data->accel_sensitivity_shift);
		icg20660l_convert_accel(val + 2, drv_data->accel_z,
					drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icg20660l_convert_accel(val, drv_data->accel_x,
					drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icg20660l_convert_accel(val, drv_data->accel_y,
					drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icg20660l_convert_accel(val, drv_data->accel_z,
					drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icg20660l_convert_gyro(val, drv_data->gyro_x,
				       drv_data->gyro_sensitivity_x10);
		icg20660l_convert_gyro(val + 1, drv_data->gyro_y,
				       drv_data->gyro_sensitivity_x10);
		icg20660l_convert_gyro(val + 2, drv_data->gyro_z,
				       drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icg20660l_convert_gyro(val, drv_data->gyro_x,
				       drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icg20660l_convert_gyro(val, drv_data->gyro_y,
				       drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icg20660l_convert_gyro(val, drv_data->gyro_z,
				       drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icg20660l_convert_temp(val, drv_data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icg20660l_sample_fetch(const struct device *dev,
				  enum sensor_channel chan)
{
	struct icg20660l_data *drv_data = dev->data;
	const struct icg20660l_config *cfg = dev->config;
	uint8_t buf[14];
	int ret;

	/*
	 * Burst read the sensor data registers starting at ACCEL_XOUT_H.
	 * The 14 bytes contain:
	 *   accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z
	 * Each quantity is a big-endian signed 16-bit value.
	 */
	ret = i2c_burst_read_dt(&cfg->i2c, ICG20660L_REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read sensor data: %d", ret);
		return ret;
	}

	drv_data->accel_x = (int16_t)sys_get_be16(&buf[0]);
	drv_data->accel_y = (int16_t)sys_get_be16(&buf[2]);
	drv_data->accel_z = (int16_t)sys_get_be16(&buf[4]);
	drv_data->temp   = (int16_t)sys_get_be16(&buf[6]);
	drv_data->gyro_x = (int16_t)sys_get_be16(&buf[8]);
	drv_data->gyro_y = (int16_t)sys_get_be16(&buf[10]);
	drv_data->gyro_z = (int16_t)sys_get_be16(&buf[12]);

	return 0;
}

/*
 * Map an accel full-scale value in g to the ACCEL_FS_SEL index and write
 * ACCEL_CONFIG. The index is also used to update the sensitivity shift.
 */
static int icg20660l_accel_set_fs(const struct device *dev, uint32_t fs_g)
{
	struct icg20660l_data *drv_data = dev->data;
	const struct icg20660l_config *cfg = dev->config;
	uint8_t fs_sel;
	uint8_t reg;
	int ret;

	switch (fs_g) {
	case 2:
		fs_sel = ICG20660L_ACCEL_FS_SEL_2G;
		break;
	case 4:
		fs_sel = ICG20660L_ACCEL_FS_SEL_4G;
		break;
	case 8:
		fs_sel = ICG20660L_ACCEL_FS_SEL_8G;
		break;
	case 16:
		fs_sel = ICG20660L_ACCEL_FS_SEL_16G;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * ACCEL_FS_SEL occupies bits [4:3] of ACCEL_CONFIG. The sensitivity in
	 * LSB/g is 2^(14 - fs_sel), so store the exponent for later conversion.
	 */
	reg = fs_sel << ICG20660L_ACCEL_FS_SHIFT;
	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_ACCEL_CONFIG, reg);
	if (ret < 0) {
		LOG_ERR("Failed to write accel full scale: %d", ret);
		return ret;
	}

	drv_data->accel_sensitivity_shift = 14 - fs_sel;

	return 0;
}

/*
 * Map a gyro full-scale value in dps to the GYRO_FS_SEL index and write
 * GYRO_CONFIG. The index selects the sensitivity lookup table entry.
 */
static int icg20660l_gyro_set_fs(const struct device *dev, uint32_t fs_dps)
{
	struct icg20660l_data *drv_data = dev->data;
	const struct icg20660l_config *cfg = dev->config;
	uint8_t fs_sel;
	uint8_t reg;
	int ret;

	switch (fs_dps) {
	case 125:
		fs_sel = ICG20660L_GYRO_FS_SEL_125DPS;
		break;
	case 250:
		fs_sel = ICG20660L_GYRO_FS_SEL_250DPS;
		break;
	case 500:
		fs_sel = ICG20660L_GYRO_FS_SEL_500DPS;
		break;
	default:
		return -EINVAL;
	}

	/* GYRO_FS_SEL occupies bits [4:3] of GYRO_CONFIG. */
	reg = fs_sel << ICG20660L_GYRO_FS_SHIFT;
	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_GYRO_CONFIG, reg);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro full scale: %d", ret);
		return ret;
	}

	drv_data->gyro_sensitivity_x10 = icg20660l_gyro_sensitivity_x10[fs_sel];

	return 0;
}

/*
 * Set the output data rate for both accel and gyro via SMPLRT_DIV.
 * The divider only takes effect when DLPF_CFG is between 1 and 6; the init
 * sequence already sets DLPF_CFG=1 for a 1 kHz internal rate.
 *   ODR = 1000 / (1 + SMPLRT_DIV) Hz
 */
static int icg20660l_set_odr(const struct device *dev, uint16_t hz)
{
	const struct icg20660l_config *cfg = dev->config;
	struct icg20660l_data *drv_data = dev->data;
	uint8_t smplrt_div;
	int ret;

	/* Allowed ODR range with DLPF_CFG=1 is 4 Hz to 1000 Hz. */
	if (hz < 4U || hz > 1000U) {
		return -EINVAL;
	}

	smplrt_div = (1000U / hz) - 1U;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_SMPLRT_DIV, smplrt_div);
	if (ret < 0) {
		LOG_ERR("Failed to write sample rate divider: %d", ret);
		return ret;
	}

	drv_data->accel_hz = hz;
	drv_data->gyro_hz = hz;

	return 0;
}

static int icg20660l_attr_set(const struct device *dev,
			      enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			return icg20660l_set_odr(dev, (uint16_t)val->val1);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			return icg20660l_accel_set_fs(dev, (uint32_t)val->val1);
		} else {
			return -ENOTSUP;
		}

	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			return icg20660l_set_odr(dev, (uint16_t)val->val1);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			return icg20660l_gyro_set_fs(dev, (uint32_t)val->val1);
		} else {
			return -ENOTSUP;
		}

	default:
		return -ENOTSUP;
	}
}

static int icg20660l_attr_get(const struct device *dev,
			      enum sensor_channel chan,
			      enum sensor_attribute attr,
			      struct sensor_value *val)
{
	const struct icg20660l_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->accel_hz;
			val->val2 = 0;
		} else {
			return -ENOTSUP;
		}
		break;

	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->gyro_hz;
			val->val2 = 0;
		} else {
			return -ENOTSUP;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Read the device identification register and verify that the chip is an
 * ICG20660L (WHO_AM_I == 0x91).
 */
static int icg20660l_check_chip_id(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;
	uint8_t id;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, ICG20660L_REG_WHO_AM_I, &id);
	if (ret < 0) {
		LOG_ERR("Failed to read WHO_AM_I: %d", ret);
		return ret;
	}

	if (id != ICG20660L_WHO_AM_I) {
		LOG_ERR("Invalid chip ID 0x%02x, expected 0x%02x", id,
			ICG20660L_WHO_AM_I);
		return -EINVAL;
	}

	LOG_DBG("ICG20660L detected");
	return 0;
}

/*
 * Reset the sensor and select the best clock source.
 *
 * The datasheet requires a soft reset (write 0x81 to PWR_MGMT_1) after
 * power-up. Bit 7 starts the reset and self-clears. CLKSEL=1 selects the
 * best available clock source (PLL if ready, otherwise internal oscillator).
 */
static int icg20660l_reset(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_PWR_MGMT_1,
				ICG20660L_PWR_MGMT_1_DEVICE_RESET |
					ICG20660L_PWR_MGMT_1_CLKSEL_1);
	if (ret < 0) {
		LOG_ERR("Failed to soft reset device: %d", ret);
		return ret;
	}

	k_msleep(10);

	/* Keep CLKSEL=1 and make sure SLEEP is cleared. */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_PWR_MGMT_1,
				ICG20660L_PWR_MGMT_1_CLKSEL_1);
	if (ret < 0) {
		LOG_ERR("Failed to wake device: %d", ret);
		return ret;
	}

	return 0;
}

/*
 * Enable the digital low-pass filter and set a default sample rate.
 *
 * DLPF_CFG=1 gives a 1 kHz internal sample rate for both accel and gyro,
 * allowing SMPLRT_DIV to control the final ODR. SMPLRT_DIV=9 yields 100 Hz.
 */
static int icg20660l_set_default_odr(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;
	struct icg20660l_data *drv_data = dev->data;
	int ret;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_CONFIG, ICG20660L_DLPF_CFG_1);
	if (ret < 0) {
		LOG_ERR("Failed to write CONFIG: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_ACCEL_CONFIG2,
				ICG20660L_ACCEL_DLPF_CFG_1);
	if (ret < 0) {
		LOG_ERR("Failed to write ACCEL_CONFIG2: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, ICG20660L_REG_SMPLRT_DIV, 9);
	if (ret < 0) {
		LOG_ERR("Failed to write SMPLRT_DIV: %d", ret);
		return ret;
	}

	drv_data->accel_hz = 100;
	drv_data->gyro_hz = 100;

	return 0;
}

static int icg20660l_init(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	/* Verify the chip identity before configuring it. */
	ret = icg20660l_check_chip_id(dev);
	if (ret < 0) {
		return ret;
	}

	/* Reset and select the best clock source. */
	ret = icg20660l_reset(dev);
	if (ret < 0) {
		return ret;
	}

	/* Configure accelerometer full scale from the device tree. */
	ret = icg20660l_accel_set_fs(dev, cfg->accel_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set accel full scale");
		return ret;
	}

	/* Configure gyroscope full scale from the device tree. */
	ret = icg20660l_gyro_set_fs(dev, cfg->gyro_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set gyro full scale");
		return ret;
	}

	/* Set a sensible default ODR so sample_fetch returns fresh data. */
	ret = icg20660l_set_default_odr(dev);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_ICG20660L_TRIGGER
	if (cfg->int_gpio.port != NULL) {
		ret = icg20660l_init_interrupt(dev);
		if (ret < 0) {
			LOG_ERR("Failed to initialize interrupts: %d", ret);
			return ret;
		}
	}
#endif

	return 0;
}

static DEVICE_API(sensor, icg20660l_driver_api) = {
#ifdef CONFIG_ICG20660L_TRIGGER
	.trigger_set = icg20660l_trigger_set,
#endif
	.sample_fetch = icg20660l_sample_fetch,
	.channel_get = icg20660l_channel_get,
	.attr_set = icg20660l_attr_set,
	.attr_get = icg20660l_attr_get,
};

/*
 * Per-instance configuration macro. The accel-fs and gyro-fs device tree
 * properties use raw register shift values, so use DT_INST_ENUM_IDX to obtain
 * the FS_SEL index directly.
 */
#define ICG20660L_DEFINE_CONFIG(inst)						\
	static const struct icg20660l_config icg20660l_cfg_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),				\
		.accel_fs = 2U << DT_INST_ENUM_IDX(inst, accel_fs),		\
		.gyro_fs = 125U << DT_INST_ENUM_IDX(inst, gyro_fs),		\
		IF_ENABLED(CONFIG_ICG20660L_TRIGGER,				\
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),)) \
	}

#define ICG20660L_INIT(inst)						\
	ICG20660L_DEFINE_CONFIG(inst);						\
	static struct icg20660l_data icg20660l_data_##inst;			\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icg20660l_init, NULL,		\
				    &icg20660l_data_##inst,			\
				    &icg20660l_cfg_##inst, POST_KERNEL,		\
				    CONFIG_SENSOR_INIT_PRIORITY,			\
				    &icg20660l_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICG20660L_INIT)
