/*
 * LIS3DH accelerometer register definitions
 *
 * Copyright (C) 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LIS3DH_H__
#define __LIS3DH_H__

#define LIS3DH_I2C_SLAVE_ADDR		0x19
#define LIS3DH_WHO_AM_I_VALUE		0x33

/* register map */
#define LIS3DH_REG_WHO_AM_I		0x0F
#define LIS3DH_REG_CTRL_REG1		0x20
#define LIS3DH_REG_CTRL_REG2		0x21
#define LIS3DH_REG_CTRL_REG3		0x22
#define LIS3DH_REG_CTRL_REG4		0x23
#define LIS3DH_REG_CTRL_REG5		0x24
#define LIS3DH_REG_CTRL_REG6		0x25
#define LIS3DH_REG_STATUS_REG		0x27
#define LIS3DH_REG_OUT_X_L		0x28
#define LIS3DH_REG_OUT_X_H		0x29
#define LIS3DH_REG_OUT_Y_L		0x2A
#define LIS3DH_REG_OUT_Y_H		0x2B
#define LIS3DH_REG_OUT_Z_L		0x2C
#define LIS3DH_REG_OUT_Z_H		0x2D
#define LIS3DH_REG_FIFO_CTRL_REG	0x2E
#define LIS3DH_REG_FIFO_SRC_REG		0x2F

/* address auto increment for multi byte read/write */
#define LIS3DH_AUTO_INC			0x80

/* CTRL_REG1: ODR[7:4] | LPen[3] | Zen Yen Xen[2:0] */
#define LIS3DH_ODR_POWERDOWN		0x00
#define LIS3DH_ODR_1HZ			0x17
#define LIS3DH_ODR_10HZ			0x27
#define LIS3DH_ODR_50HZ			0x47
#define LIS3DH_ODR_100HZ		0x57

/* CTRL_REG4: BDU[7] BLE[6] FS[5:4] HR[3] ST[2] SIM[1] */
#define LIS3DH_CTRL4_BDU_HR_2G		0x88

#endif /* __LIS3DH_H__ */
