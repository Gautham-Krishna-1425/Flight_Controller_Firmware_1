/*
 * bmp280.h
 *
 *  Created on: Jan 3, 2026
 *      Author: sidhu
 */

#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"

#define BMP280_ADDR_0   (0x76 << 1)
#define BMP280_ADDR_1   (0x77 << 1)

typedef struct
{
    float temperature;     // °C
    float pressure;        // Pa
    float altitude;        // m (raw)
    float altitude_filt;   // m (low-pass filtered)
} BMP280_Data_t;

/* Initialization */
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);

/* Read sensor + compute altitude */
HAL_StatusTypeDef BMP280_Read(
    I2C_HandleTypeDef *hi2c,
    BMP280_Data_t *data,
    float dt
);
HAL_StatusTypeDef bmp280_read_temp_press(
    I2C_HandleTypeDef *hi2c,
    float dt,
    float *temperature,
    float *pressure,
    float *altitude,
    float *altitude_filtered,
    float *climb_rate
);


#endif
