#include "pca9685.h"
#include "i2c.h"

#define PCA_MODE1 0x00
#define PCA_MODE2 0x01
#define PCA_PRESCALE 0xFE
#define PCA_LED0_ON_L 0x06

#define MODE1_RESTART 0x80
#define MODE1_AI 0x20
#define MODE1_SLEEP 0x10
#define MODE2_OUTDRV 0x04

// 舵机脆宽范围: 500~2500us
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U

static uint16_t addr8(uint8_t addr7) { return (uint16_t)(addr7 << 1); }

static HAL_StatusTypeDef write8(uint8_t addr7, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, 100);
}

static HAL_StatusTypeDef read8(uint8_t addr7, uint8_t reg, uint8_t *val) {
  return HAL_I2C_Mem_Read(&hi2c1, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT, val,
                          1, 100);
}

HAL_StatusTypeDef PCA9685_SetPWM(uint8_t addr7, uint8_t ch, uint16_t on,
                                 uint16_t off) {
  if (ch > 15)
    return HAL_ERROR;
  on &= 0x0FFF;
  off &= 0x0FFF;

  uint8_t reg = (uint8_t)(PCA_LED0_ON_L + 4 * ch);
  uint8_t buf[4];

  buf[0] = (uint8_t)(on & 0xFF);
  buf[1] = (uint8_t)((on >> 8) & 0x0F);
  buf[2] = (uint8_t)(off & 0xFF);
  buf[3] = (uint8_t)((off >> 8) & 0x0F);

  return HAL_I2C_Mem_Write(&hi2c1, addr8(addr7), reg, I2C_MEMADD_SIZE_8BIT, buf,
                           4, 100);
}

HAL_StatusTypeDef PCA9685_Init50Hz(uint8_t addr7) {
  // 50Hz prescale = 121
  const uint8_t prescale_50hz = 121;

  if (HAL_I2C_IsDeviceReady(&hi2c1, addr8(addr7), 3, 100) != HAL_OK)
    return HAL_ERROR;

  uint8_t oldmode = 0;
  if (read8(addr7, PCA_MODE1, &oldmode) != HAL_OK)
    return HAL_ERROR;

  // sleep
  if (write8(addr7, PCA_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP) !=
      HAL_OK)
    return HAL_ERROR;
  if (write8(addr7, PCA_PRESCALE, prescale_50hz) != HAL_OK)
    return HAL_ERROR;

  // wake
  if (write8(addr7, PCA_MODE1, MODE1_AI) != HAL_OK)
    return HAL_ERROR;
  HAL_Delay(5);

  // restart + totem pole
  if (write8(addr7, PCA_MODE1, MODE1_AI | MODE1_RESTART) != HAL_OK)
    return HAL_ERROR;
  if (write8(addr7, PCA_MODE2, MODE2_OUTDRV) != HAL_OK)
    return HAL_ERROR;

  // 初始化所有通道
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (PCA9685_SetPWM(addr7, ch, 0, 0) != HAL_OK)
      return HAL_ERROR;
  }

  return HAL_OK;
}

void PCA9685_ServoWriteDeg(uint8_t addr7, uint8_t ch, uint16_t deg) {
  if (deg > 180)
    deg = 180;

  // us = min + deg*(max-min)/180
  uint32_t us =
      SERVO_MIN_US + (uint32_t)deg * (SERVO_MAX_US - SERVO_MIN_US) / 180U;

  // 50Hz ?? 20000us:off = us*4096/20000
  uint32_t off = us * 4096U / 20000U;
  if (off > 4095U)
    off = 4095U;

  (void)PCA9685_SetPWM(addr7, ch, 0, (uint16_t)off);
}
