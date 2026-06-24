/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_REG_H_

/*
 * ICG20660L register map.
 * The sensor is functionally similar to the MPU6500 family; registers are
 * accessed over I2C (and optionally SPI) in big-endian byte order.
 */

/* Self-test registers */
#define ICG20660L_REG_SELF_TEST_X_GYRO	0x00
#define ICG20660L_REG_SELF_TEST_Y_GYRO	0x01
#define ICG20660L_REG_SELF_TEST_Z_GYRO	0x02
#define ICG20660L_REG_SELF_TEST_X_ACCEL	0x0D
#define ICG20660L_REG_SELF_TEST_Y_ACCEL	0x0E
#define ICG20660L_REG_SELF_TEST_Z_ACCEL	0x0F

/* Gyro offset registers (optional user offset cancellation) */
#define ICG20660L_REG_XG_OFFS_USRH	0x13
#define ICG20660L_REG_XG_OFFS_USRL	0x14
#define ICG20660L_REG_YG_OFFS_USRH	0x15
#define ICG20660L_REG_YG_OFFS_USRL	0x16
#define ICG20660L_REG_ZG_OFFS_USRH	0x17
#define ICG20660L_REG_ZG_OFFS_USRL	0x18

/* Sample rate divider.
 * ODR = internal_sample_rate / (1 + SMPLRT_DIV).
 * Effective only when FCHOICE_B = 0 and 0 < DLPF_CFG < 7 (1 kHz internal rate).
 */
#define ICG20660L_REG_SMPLRT_DIV	0x19

/* Configuration register: DLPF_CFG bits [2:0] select the low-pass filter. */
#define ICG20660L_REG_CONFIG		0x1A
#define ICG20660L_DLPF_CFG_1		0x01

/* Gyroscope configuration: FS_SEL bits [4:3]. */
#define ICG20660L_REG_GYRO_CONFIG	0x1B
#define ICG20660L_GYRO_FS_SHIFT		3
#define ICG20660L_GYRO_FS_SEL_125DPS	0x00
#define ICG20660L_GYRO_FS_SEL_250DPS	0x01
#define ICG20660L_GYRO_FS_SEL_500DPS	0x02

/* Accelerometer configuration: ACCEL_FS_SEL bits [4:3]. */
#define ICG20660L_REG_ACCEL_CONFIG	0x1C
#define ICG20660L_ACCEL_FS_SHIFT	3
#define ICG20660L_ACCEL_FS_SEL_2G	0x00
#define ICG20660L_ACCEL_FS_SEL_4G	0x01
#define ICG20660L_ACCEL_FS_SEL_8G	0x02
#define ICG20660L_ACCEL_FS_SEL_16G	0x03

/* Accelerometer configuration 2: A_DLPF_CFG bits [2:0]. */
#define ICG20660L_REG_ACCEL_CONFIG2	0x1D
#define ICG20660L_ACCEL_DLPF_CFG_1	0x01

/* FIFO enable register */
#define ICG20660L_REG_FIFO_EN		0x23

/* FSYNC interrupt status */
#define ICG20660L_REG_FSYNC_INT		0x36

/* INT pin / bypass enable configuration */
#define ICG20660L_REG_INT_PIN_CFG	0x37

/* Interrupt enable register: DATA_RDY_INT_EN bit [0]. */
#define ICG20660L_REG_INT_ENABLE	0x38
#define ICG20660L_INT_ENABLE_DATA_RDY	BIT(0)

/* Interrupt status register: DATA_RDY_INT bit [0]. */
#define ICG20660L_REG_INT_STATUS	0x3A
#define ICG20660L_INT_STATUS_DATA_RDY	BIT(0)

/* Sensor data registers, 14 bytes contiguous starting here. */
#define ICG20660L_REG_ACCEL_XOUT_H	0x3B
#define ICG20660L_REG_ACCEL_XOUT_L	0x3C
#define ICG20660L_REG_ACCEL_YOUT_H	0x3D
#define ICG20660L_REG_ACCEL_YOUT_L	0x3E
#define ICG20660L_REG_ACCEL_ZOUT_H	0x3F
#define ICG20660L_REG_ACCEL_ZOUT_L	0x40
#define ICG20660L_REG_TEMP_OUT_H	0x41
#define ICG20660L_REG_TEMP_OUT_L	0x42
#define ICG20660L_REG_GYRO_XOUT_H	0x43
#define ICG20660L_REG_GYRO_XOUT_L	0x44
#define ICG20660L_REG_GYRO_YOUT_H	0x45
#define ICG20660L_REG_GYRO_YOUT_L	0x46
#define ICG20660L_REG_GYRO_ZOUT_H	0x47
#define ICG20660L_REG_GYRO_ZOUT_L	0x48

/* Signal path reset */
#define ICG20660L_REG_SIGNAL_PATH_RESET	0x68

/* Accelerometer intelligence control (wake-on-motion) */
#define ICG20660L_REG_ACCEL_INTEL_CTRL	0x69

/* User control: FIFO enable/reset, I2C disable, signal conditioning reset. */
#define ICG20660L_REG_USER_CTRL		0x6A
#define ICG20660L_USER_CTRL_I2C_IF_DIS		BIT(4)

/* Power management 1:
 * [7] DEVICE_RESET
 * [6] SLEEP
 * [5] CYCLE
 * [4] GYRO_STANDBY
 * [3] TEMP_DIS
 * [2:0] CLKSEL
 */
#define ICG20660L_REG_PWR_MGMT_1	0x6B
#define ICG20660L_PWR_MGMT_1_DEVICE_RESET	BIT(7)
#define ICG20660L_PWR_MGMT_1_SLEEP		BIT(6)
#define ICG20660L_PWR_MGMT_1_CLKSEL_1		0x01

/* Power management 2: per-axis standby control. */
#define ICG20660L_REG_PWR_MGMT_2	0x6C

/* FIFO count registers */
#define ICG20660L_REG_FIFO_COUNTH	0x72
#define ICG20660L_REG_FIFO_COUNTL	0x73

/* FIFO read/write register */
#define ICG20660L_REG_FIFO_R_W		0x74

/* WHO_AM_I: device identification register. Expected value 0x91. */
#define ICG20660L_REG_WHO_AM_I		0x75
#define ICG20660L_WHO_AM_I		0x91

#endif /* ZEPHYR_DRIVERS_SENSOR_ICG20660L_ICG20660L_REG_H_ */
