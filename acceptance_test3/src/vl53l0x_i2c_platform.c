#include <vl53l0x_i2c_platform.h>
#include "tm_stm32f4_i2c.h"
#include <stdio.h>


#define MAX_STR_SIZE 255
#define MAX_MSG_SIZE 100
#define MAX_DEVICES 2
#define STATUS_OK 0x00
#define STATUS_FAIL 0x01

static unsigned char _dataBytes[MAX_MSG_SIZE];

int VL53L0X_i2c_init(void) {
  //Wire.begin();
  return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count) {
  TM_I2C_WriteMulti(I2C1, address, index, pdata, count);
  return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count) {
  TM_I2C_ReadMulti(I2C1, address, index, pdata, count);
  return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) {
  return VL53L0X_write_multi(deviceAddress, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) {
  uint8_t buff[2];
  buff[1] = data & 0xFF;
  buff[0] = data >> 8;
  return VL53L0X_write_multi(deviceAddress, index, buff, 2);
}

int32_t VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) {
  uint8_t buff[4];

  buff[3] = data & 0xFF;
  buff[2] = data >> 8;
  buff[1] = data >> 16;
  buff[0] = data >> 24;

  return VL53L0X_write_multi(deviceAddress, index, buff, 4);
}

int32_t VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) {
  return VL53L0X_read_multi(deviceAddress, index, data, 1);
}

int32_t VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) {
  uint8_t buff[2];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 2);

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int32_t VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) {
  uint8_t buff[4];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 4);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}
