#ifndef __PCA9685_H
#define __PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define PCA9685_I2C_ADDR_7BIT  0x40

HAL_StatusTypeDef PCA9685_Init50Hz(uint8_t addr7);
HAL_StatusTypeDef PCA9685_SetPWM(uint8_t addr7, uint8_t ch, uint16_t on, uint16_t off);
void PCA9685_ServoWriteDeg(uint8_t addr7, uint8_t ch, uint16_t deg);

#ifdef __cplusplus
}
#endif

#endif
