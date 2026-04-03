/*
 * mpu9250.h
 *
 *  Created on: Jan 2, 2026
 *      Author: sidhu
 */
#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f4xx_hal.h"

#define MPU9250_ADDR  (0x68 << 1)

HAL_StatusTypeDef mpu9250_init(
    I2C_HandleTypeDef *hi2c,
    uint8_t *whoami
);

HAL_StatusTypeDef mpu9250_read_accel(
    I2C_HandleTypeDef *hi2c,
    int16_t *ax,
    int16_t *ay,
    int16_t *az
);

HAL_StatusTypeDef mpu9250_read_gyro(
    I2C_HandleTypeDef *hi2c,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz
);

#endif
