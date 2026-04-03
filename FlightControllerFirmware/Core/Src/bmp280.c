/*
 * bmp280.c
 *
 *  Created on: Jan 3, 2026
 *      Author: sidhu
 *
 *  Purpose:
 *  - Read BMP280 temperature & pressure
 *  - Convert pressure to altitude
 *  - Apply low-pass filtering
 *  - NO control logic here
 */

#include "bmp280.h"
#include "i2c_bus.h"
#include <math.h>

/* BMP280 Registers */
#define BMP280_CHIP_ID      0xD0
#define BMP280_RESET        0xE0
#define BMP280_CTRL_MEAS    0xF4
#define BMP280_CONFIG       0xF5
#define BMP280_PRESS_MSB    0xF7

/* Calibration coefficients */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t t_fine;

static float altitude_prev = 0.0f;
static float climb_rate = 0.0f;


/* Internal state */
static uint8_t  bmp_addr = BMP280_ADDR_0;
static float    P0 = 0.0f;          // reference pressure
static float    altitude_filt = 0.0f;
static uint8_t  initialized = 0;

/* Internal helper */
static float bmp280_pressure_to_altitude(float pressure)
{
    return 44330.0f * (1.0f - powf(pressure / P0, 0.1903f));
}

/* Initialize BMP280 */
HAL_StatusTypeDef bmp280_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;

    if (i2c_read_reg(hi2c, bmp_addr, BMP280_CHIP_ID, &id) != HAL_OK)
        return HAL_ERROR;

    if (id != 0x58)
        return HAL_ERROR;

    /* Read calibration data */
    uint8_t calib[24];
    if (i2c_read_bytes(hi2c, bmp_addr, 0x88, calib, 24) != HAL_OK)
        return HAL_ERROR;

    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    /* Normal mode, oversampling x2, filter enabled */
    i2c_write_reg(hi2c, bmp_addr, BMP280_CTRL_MEAS, 0x27);
    i2c_write_reg(hi2c, bmp_addr, BMP280_CONFIG,    0xA0);

    initialized = 0;
    return HAL_OK;
}

/* Read temperature, pressure, and compute altitude */
HAL_StatusTypeDef bmp280_read_temp_press(
    I2C_HandleTypeDef *hi2c,
    float dt,
    float *temperature,
    float *pressure,
    float *altitude,
    float *altitude_filtered,
    float *climb_rate_out
)


{
    uint8_t data[6];

    if (i2c_read_bytes(hi2c, bmp_addr, BMP280_PRESS_MSB, data, 6) != HAL_OK)
        return HAL_ERROR;

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    /* ---- Temperature compensation ---- */
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
                       ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) *
                     dig_T3) >> 14;

    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) / 25600.0f;

    /* ---- Pressure compensation ---- */
    int64_t p;
    int64_t var1p = ((int64_t)t_fine) - 128000;
    int64_t var2p = var1p * var1p * (int64_t)dig_P6;
    var2p += ((var1p * (int64_t)dig_P5) << 17);
    var2p += (((int64_t)dig_P4) << 35);

    var1p = ((var1p * var1p * (int64_t)dig_P3) >> 8) +
            ((var1p * (int64_t)dig_P2) << 12);
    var1p = (((((int64_t)1) << 47) + var1p) * (int64_t)dig_P1) >> 33;

    if (var1p == 0)
        return HAL_ERROR;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2p = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + var1p + var2p) >> 8) + (((int64_t)dig_P7) << 4);

    *pressure = p / 256.0f;  // Pa

    /* ---- Altitude estimation ---- */
    if (!initialized)
    {
        P0 = *pressure;
        altitude_filt = 0.0f;
        altitude_prev = 0.0f;
        initialized = 1;
    }


    float alt_raw = bmp280_pressure_to_altitude(*pressure);

    /* Low-pass filter (critical) */
    float tau = 1.0f;  // 1 second time constant
    float alpha = dt / (tau + dt);

    altitude_filt += alpha * (alt_raw - altitude_filt);

    climb_rate = (altitude_filt - altitude_prev) / dt;
    altitude_prev = altitude_filt;



    *altitude = alt_raw;
    *altitude_filtered = altitude_filt;

    return HAL_OK;
}

