/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include "icg20660l.h"

#if ICG20660L_BUS_SPI

LOG_MODULE_DECLARE(ICG20660L, CONFIG_SENSOR_LOG_LEVEL);

static int icg20660l_bus_check_spi(const union icg20660l_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

/*
 * The ICG20660L powers up with both I2C and SPI interfaces enabled. When the
 * device is wired for SPI, the I2C slave must be disabled so that register
 * writes over SPI are accepted. USER_CTRL.I2C_IF_DIS does that.
 */
static int icg20660l_configure_spi(const struct device *dev)
{
	return icg20660l_bus_write(dev, ICG20660L_REG_USER_CTRL,
				   ICG20660L_USER_CTRL_I2C_IF_DIS);
}

/*
 * SPI read: address byte with bit 7 set, then read len bytes in one CS assertion.
 * The ICG20660L auto-increments the register address while bit 7 is set, so
 * contiguous registers can be read in a single transaction.
 */
static int icg20660l_reg_read_spi(const union icg20660l_bus *bus,
				  uint8_t start, uint8_t *buf, uint16_t len)
{
	uint8_t addr = start | 0x80U;
	const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;
	rx_buf[1].buf = buf;
	rx_buf[1].len = len;

	return spi_transceive_dt(&bus->spi, &tx, &rx);
}

/*
 * SPI write: address byte with bit 7 clear, then write the data byte.
 */
static int icg20660l_reg_write_spi(const union icg20660l_bus *bus,
				   uint8_t reg, uint8_t val)
{
	uint8_t cmd[] = { reg & 0x7FU, val };
	const struct spi_buf tx_buf = { .buf = cmd, .len = sizeof(cmd) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&bus->spi, &tx);
}

const struct icg20660l_bus_io icg20660l_bus_io_spi = {
	.check = icg20660l_bus_check_spi,
	.read  = icg20660l_reg_read_spi,
	.write = icg20660l_reg_write_spi,
	.configure = icg20660l_configure_spi,
};

#endif /* ICG20660L_BUS_SPI */
