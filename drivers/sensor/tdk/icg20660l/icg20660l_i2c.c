/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icg20660l.h"

#if ICG20660L_BUS_I2C

static int icg20660l_bus_check_i2c(const union icg20660l_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int icg20660l_reg_read_i2c(const union icg20660l_bus *bus,
				  uint8_t start, uint8_t *buf, uint16_t len)
{
	return i2c_burst_read_dt(&bus->i2c, start, buf, len);
}

static int icg20660l_reg_write_i2c(const union icg20660l_bus *bus,
				   uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&bus->i2c, reg, val);
}

const struct icg20660l_bus_io icg20660l_bus_io_i2c = {
	.check = icg20660l_bus_check_i2c,
	.read  = icg20660l_reg_read_i2c,
	.write = icg20660l_reg_write_i2c,
	.configure = NULL,
};

#endif /* ICG20660L_BUS_I2C */
