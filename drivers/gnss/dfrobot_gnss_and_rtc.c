/*
 * Copyright 2025 SouthBitsLab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dfrobot_gnss_and_rtc, CONFIG_GNSS_LOG_LEVEL);

/* L76K GNSS register block (DFRobot GNSSAndRTC, GNSS-only). */
#define REG_YEAR_H      0x00
#define REG_YEAR_L      0x01
#define REG_MONTH       0x02
#define REG_DATE        0x03
#define REG_HOUR        0x04
#define REG_MINUTE      0x05
#define REG_SECOND      0x06
/*
 * NOTE: The numeric parts follow the DFRobot header layout:
 * https://github.com/DFRobot/DFRobot_GNSSAndRTC
 *   0x07-0x0B : latitude degrees/minutes/fraction
 *   0x0D-0x11 : longitude degrees/minutes/fraction
 * But on real hardware the direction bytes are swapped:
 *   0x0C : longitude direction ('E'/'W')
 *   0x12 : latitude direction ('N'/'S')
 */
#define REG_LAT_1       0x07
#define REG_LAT_2       0x08
#define REG_LAT_X_24    0x09
#define REG_LAT_X_16    0x0A
#define REG_LAT_X_8     0x0B
#define REG_LAT_DIS     0x12
#define REG_LON_1       0x0D
#define REG_LON_2       0x0E
#define REG_LON_X_24    0x0F
#define REG_LON_X_16    0x10
#define REG_LON_X_8     0x11
#define REG_LON_DIS     0x0C
#define REG_USE_STAR    0x13
#define REG_ALT_H       0x14
#define REG_ALT_L       0x15
#define REG_ALT_X       0x16
#define REG_SOG_H       0x17
#define REG_SOG_L       0x18
#define REG_SOG_X       0x19
#define REG_COG_H       0x1A
#define REG_COG_L       0x1B
#define REG_COG_X       0x1C
#define REG_START_GET   0x1D
#define REG_I2C_ADDR    0x1E
#define REG_DATA_LEN_H  0x1F
#define REG_DATA_LEN_L  0x20
#define REG_ALL_DATA    0x21
#define REG_GNSS_MODE   0x22
#define REG_SLEEP_MODE  0x23

#define REG_BLOCK_SIZE  0x24

/* Sign extension helpers. Latitude/longitude are in nanodegrees (1 deg = 1e9). */
#define NANODEG_PER_DEG  1000000000LL
#define LAT_LON_FRAC_DIV (100000U * 60U)
#define ALT_FRAC_DIV     100U
#define SOG_COG_FRAC_DIV 100U

struct dfrobot_gnss_and_rtc_config {
	struct i2c_dt_spec bus;
	uint16_t poll_interval_ms;
	uint8_t gnss_mode;
};

struct dfrobot_gnss_and_rtc_data {
	const struct device *dev;
	struct k_work_delayable poll_work;
	struct gnss_data last_data;
};

static int64_t dfrobot_minutes_to_nanodeg(uint8_t deg, uint8_t min, uint32_t frac_min,
					  char direction)
{
	int64_t value;
	int64_t sign;

	sign = (direction == 'S' || direction == 'W') ? -1 : 1;

	/* value = deg + min/60 + frac_min/(100000*60), in nanodegrees. */
	value = ((int64_t)deg) * NANODEG_PER_DEG;
	value += ((int64_t)min * NANODEG_PER_DEG) / 60;
	value += ((int64_t)frac_min * NANODEG_PER_DEG) / LAT_LON_FRAC_DIV;

	return sign * value;
}

static uint32_t dfrobot_frac_value(uint8_t high, uint8_t mid, uint8_t low)
{
	return ((uint32_t)high << 16) | ((uint32_t)mid << 8) | (uint32_t)low;
}

static void dfrobot_gnss_and_rtc_publish(struct dfrobot_gnss_and_rtc_data *data,
					 const uint8_t *regs)
{
	struct gnss_data gnss_data = {0};
	uint16_t year;
	uint32_t lat_frac;
	uint32_t lon_frac;
	uint16_t alt_int;
	uint16_t sog_int;
	uint16_t cog_int;
	bool fix_valid;

	year = ((uint16_t)regs[REG_YEAR_H] << 8) | regs[REG_YEAR_L];

	gnss_data.utc.century_year = year % 100;
	gnss_data.utc.month = regs[REG_MONTH];
	gnss_data.utc.month_day = regs[REG_DATE];
	gnss_data.utc.hour = regs[REG_HOUR];
	gnss_data.utc.minute = regs[REG_MINUTE];
	gnss_data.utc.millisecond = regs[REG_SECOND] * 1000U;

	// LOG_INF("raw lat=%c lon=%c", regs[REG_LAT_DIS], regs[REG_LON_DIS]);


	lat_frac = dfrobot_frac_value(regs[REG_LAT_X_24], regs[REG_LAT_X_16], regs[REG_LAT_X_8]);
	gnss_data.nav_data.latitude = dfrobot_minutes_to_nanodeg(regs[REG_LAT_1], regs[REG_LAT_2],
							       lat_frac,
							       (char)regs[REG_LAT_DIS]);

	lon_frac = dfrobot_frac_value(regs[REG_LON_X_24], regs[REG_LON_X_16], regs[REG_LON_X_8]);
	gnss_data.nav_data.longitude = dfrobot_minutes_to_nanodeg(regs[REG_LON_1], regs[REG_LON_2],
							        lon_frac,
							        (char)regs[REG_LON_DIS]);

	gnss_data.info.satellites_cnt = regs[REG_USE_STAR];

	/* Altitude: 15-bit integer + fraction/100 in meters -> millimeters. */
	alt_int = (((uint16_t)regs[REG_ALT_H] & 0x7FU) << 8) | regs[REG_ALT_L];
	gnss_data.nav_data.altitude = ((int32_t)alt_int * 1000)
				      + ((int32_t)regs[REG_ALT_X] * 1000 / (int32_t)ALT_FRAC_DIV);

	/* Speed over ground: integer + fraction/100 (units TBD, stored as mm/s placeholder). */
	sog_int = (uint16_t)(((uint16_t)regs[REG_SOG_H] & 0x7FU) << 8) | regs[REG_SOG_L];
	gnss_data.nav_data.speed = ((uint32_t)sog_int * 1000)
				   + ((uint32_t)regs[REG_SOG_X] * 1000U / SOG_COG_FRAC_DIV);

	/* Course over ground: integer + fraction/100 in degrees -> millidegrees. */
	cog_int = (uint16_t)(((uint16_t)regs[REG_COG_H] & 0x7FU) << 8) | regs[REG_COG_L];
	gnss_data.nav_data.bearing = ((uint32_t)cog_int * 1000)
				     + ((uint32_t)regs[REG_COG_X] * 1000U / SOG_COG_FRAC_DIV);

	fix_valid = (gnss_data.info.satellites_cnt > 0);

	if (fix_valid) {
		gnss_data.info.fix_status = GNSS_FIX_STATUS_GNSS_FIX;
		gnss_data.info.fix_quality = GNSS_FIX_QUALITY_GNSS_SPS;
	} else {
		gnss_data.info.fix_status = GNSS_FIX_STATUS_NO_FIX;
		gnss_data.info.fix_quality = GNSS_FIX_QUALITY_INVALID;
	}

	data->last_data = gnss_data;
	gnss_publish_data(data->dev, &gnss_data);
}

static void dfrobot_gnss_and_rtc_poll(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct dfrobot_gnss_and_rtc_data *data = CONTAINER_OF(dwork,
							      struct dfrobot_gnss_and_rtc_data,
							      poll_work);
	const struct dfrobot_gnss_and_rtc_config *cfg = data->dev->config;
	uint8_t regs[REG_BLOCK_SIZE];
	int ret;

	ret = i2c_burst_read_dt(&cfg->bus, REG_YEAR_H, regs, sizeof(regs));
	if (ret == 0) {
		dfrobot_gnss_and_rtc_publish(data, regs);
	} else {
		LOG_WRN("GNSS register read failed: %d", ret);
	}

	k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_interval_ms));
}

static int dfrobot_gnss_and_rtc_init(const struct device *dev)
{
	const struct dfrobot_gnss_and_rtc_config *cfg = dev->config;
	struct dfrobot_gnss_and_rtc_data *data = dev->data;
	uint8_t buf;
	int ret;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Probe the device with a single-byte read from the GNSS register block. */
	ret = i2c_burst_read_dt(&cfg->bus, REG_YEAR_H, &buf, 1);
	if (ret < 0) {
		LOG_ERR("Device not responding on I2C: %d", ret);
		return ret;
	}

	/* Enable GNSS power. */
	ret = i2c_reg_write_byte_dt(&cfg->bus, REG_SLEEP_MODE, 0x00);
	if (ret < 0) {
		LOG_ERR("Failed to enable GNSS power: %d", ret);
		return ret;
	}

	/* Set constellation mode. */
	ret = i2c_reg_write_byte_dt(&cfg->bus, REG_GNSS_MODE, cfg->gnss_mode);
	if (ret < 0) {
		LOG_ERR("Failed to set GNSS mode: %d", ret);
		return ret;
	}

	data->dev = dev;
	k_work_init_delayable(&data->poll_work, dfrobot_gnss_and_rtc_poll);
	k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_interval_ms));

	return 0;
}

static DEVICE_API(gnss, dfrobot_gnss_and_rtc_api) = {
};

#define DFROBOT_GNSS_AND_RTC_INIT(inst)                                                       \
	static const struct dfrobot_gnss_and_rtc_config dfrobot_gnss_and_rtc_cfg_##inst = {   \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                             \
		.poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms),                      \
		.gnss_mode = DT_INST_PROP(inst, gnss_mode),                                    \
	};                                                                                    \
                                                                                              \
	static struct dfrobot_gnss_and_rtc_data dfrobot_gnss_and_rtc_data_##inst;             \
                                                                                              \
	DEVICE_DT_INST_DEFINE(inst, dfrobot_gnss_and_rtc_init, NULL,                          \
			      &dfrobot_gnss_and_rtc_data_##inst,                                \
			      &dfrobot_gnss_and_rtc_cfg_##inst, POST_KERNEL,                    \
			      CONFIG_GNSS_INIT_PRIORITY, &dfrobot_gnss_and_rtc_api);

#define DT_DRV_COMPAT dfrobot_gnss_and_rtc
DT_INST_FOREACH_STATUS_OKAY(DFROBOT_GNSS_AND_RTC_INIT)
#undef DT_DRV_COMPAT
