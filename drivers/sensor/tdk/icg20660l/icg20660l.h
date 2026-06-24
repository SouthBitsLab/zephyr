/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_H_
#define ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "icg20660l_reg.h"

#define DT_DRV_COMPAT invensense_icg20660l

#define ICG20660L_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define ICG20660L_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

/* Gyroscope sensitivity in LSB/(dps) * 10, indexed by FS_SEL value. */
static const uint16_t icg20660l_gyro_sensitivity_x10[] = {
	2620,	/* FS_SEL = 0: ±125 dps, 262 LSB/(dps) */
	1310,	/* FS_SEL = 1: ±250 dps, 131 LSB/(dps) */
	655,	/* FS_SEL = 2: ±500 dps, 65.5 LSB/(dps) */
};

/*
 * Driver runtime data. Holds the most recently fetched raw sensor values and
 * the current sensitivity settings needed by channel_get.
 */
struct icg20660l_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

	uint16_t accel_hz;
	uint16_t gyro_hz;

#ifdef CONFIG_ICG20660L_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;

	const struct sensor_trigger *data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_ICG20660L_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICG20660L_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICG20660L_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ICG20660L_TRIGGER */
};

union icg20660l_bus {
#if ICG20660L_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if ICG20660L_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*icg20660l_bus_check_fn)(const union icg20660l_bus *bus);
typedef int (*icg20660l_reg_read_fn)(const union icg20660l_bus *bus,
				     uint8_t start, uint8_t *buf, uint16_t len);
typedef int (*icg20660l_reg_write_fn)(const union icg20660l_bus *bus,
				      uint8_t reg, uint8_t val);
typedef int (*icg20660l_bus_configure_fn)(const struct device *dev);

struct icg20660l_bus_io {
	icg20660l_bus_check_fn check;
	icg20660l_reg_read_fn read;
	icg20660l_reg_write_fn write;
	icg20660l_bus_configure_fn configure;
};

#if ICG20660L_BUS_SPI
#define ICG20660L_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
extern const struct icg20660l_bus_io icg20660l_bus_io_spi;
#endif

#if ICG20660L_BUS_I2C
extern const struct icg20660l_bus_io icg20660l_bus_io_i2c;
#endif

/*
 * Driver configuration, populated from the device tree instance.
 */
struct icg20660l_config {
	union icg20660l_bus bus;
	const struct icg20660l_bus_io *bus_io;
	uint16_t accel_fs;
	uint16_t gyro_fs;
	uint16_t hz;

#ifdef CONFIG_ICG20660L_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
};

static inline int icg20660l_bus_check(const struct device *dev)
{
	const struct icg20660l_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int icg20660l_bus_read(const struct device *dev,
				     uint8_t start, uint8_t *buf, uint16_t len)
{
	const struct icg20660l_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, start, buf, len);
}

static inline int icg20660l_bus_write(const struct device *dev,
				      uint8_t reg, uint8_t val)
{
	const struct icg20660l_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, val);
}

#ifdef CONFIG_ICG20660L_TRIGGER
int icg20660l_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler);

int icg20660l_init_interrupt(const struct device *dev);
#endif /* CONFIG_ICG20660L_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_H_ */
