// Adapted from bdholt1/ros2_bno055_sensor (Apache-2.0)

#include "bno055_hardware_interface/bno055_i2c.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

static int i2c_fd = -1;

int bno055_i2c_open(const char * i2c_bus, u8 i2c_addr)
{
  i2c_fd = open(i2c_bus, O_RDWR);
  if (i2c_fd < 0) {
    fprintf(stderr, "bno055_i2c: failed to open %s\n", i2c_bus);
    return -1;
  }
  if (ioctl(i2c_fd, I2C_SLAVE, (int)i2c_addr) < 0) {
    fprintf(stderr, "bno055_i2c: failed to set I2C slave 0x%02X\n", i2c_addr);
    close(i2c_fd);
    i2c_fd = -1;
    return -1;
  }
  return 0;
}

void bno055_i2c_close(void)
{
  if (i2c_fd >= 0) {
    close(i2c_fd);
    i2c_fd = -1;
  }
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 cnt)
{
  (void)dev_addr;
  int result;
  if (cnt == 1) {
    result = i2c_smbus_read_byte_data(i2c_fd, reg_addr);
    if (result < 0) {
      return 1;
    }
    reg_data[0] = (u8)result;
  } else {
    result = i2c_smbus_read_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
    if (result != (int)cnt) {
      return 1;
    }
  }
  return 0;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 cnt)
{
  (void)dev_addr;
  int result;
  if (cnt == 1) {
    result = i2c_smbus_write_byte_data(i2c_fd, reg_addr, reg_data[0]);
    if (result < 0) {
      return 1;
    }
  } else {
    result = i2c_smbus_write_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
    if (result < 0) {
      return 1;
    }
  }
  return 0;
}

void BNO055_delay_msek(u32 msek)
{
  usleep(msek * 1000);
}
