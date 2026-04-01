// Copyright 2026 Aditya Kamath
// Adapted from bdholt1/ros2_bno055_sensor (Apache-2.0)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BNO055_HARDWARE_INTERFACE__BNO055_I2C_H_
#define BNO055_HARDWARE_INTERFACE__BNO055_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bno055.h"

/** Open the I2C bus and configure it for the BNO055 device.
 *  @param i2c_bus  path to the I2C device, e.g. "/dev/i2c-1"
 *  @param i2c_addr sensor address, e.g. 0x28
 *  @return 0 on success, -1 on error
 */
int bno055_i2c_open(const char * i2c_bus, u8 i2c_addr);

/** Close the I2C file descriptor. */
void bno055_i2c_close(void);

/** Bosch Sensortec I2C read callback. */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 cnt);

/** Bosch Sensortec I2C write callback. */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 cnt);

/** Bosch Sensortec delay callback (milliseconds). */
void BNO055_delay_msek(u32 msek);

#ifdef __cplusplus
}
#endif

#endif  // BNO055_HARDWARE_INTERFACE__BNO055_I2C_H_
