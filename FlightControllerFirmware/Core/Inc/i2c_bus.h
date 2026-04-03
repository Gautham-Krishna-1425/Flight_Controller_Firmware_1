/*
 * i2c_bus.h
 *
 *  Created on: Dec 18, 2025
 *      Author: sidhu
 */


#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

/* Scan entire I2C bus */
void I2C_Scan(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef i2c_read_reg(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t *data
);

HAL_StatusTypeDef i2c_read_bytes(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t *buf,
    uint16_t len
);

HAL_StatusTypeDef i2c_write_reg(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t value
);


#endif
