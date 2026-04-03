/*
 * mpu9250.c
 *
 *  Created on: Jan 2, 2026
 *      Author: sidhu
 */
#include "mpu9250.h"
#include "i2c_bus.h"

#define WHO_AM_I_REG  0x75
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

HAL_StatusTypeDef mpu9250_init(
    I2C_HandleTypeDef *hi2c,
    uint8_t *whoami
)
{
    uint8_t id = 0;

    if (i2c_read_reg(hi2c, MPU9250_ADDR, WHO_AM_I_REG, &id) != HAL_OK)
        return HAL_ERROR;

    *whoami = id;

    return i2c_write_reg(hi2c, MPU9250_ADDR, PWR_MGMT_1, 0x00);
}

HAL_StatusTypeDef mpu9250_read_accel(
    I2C_HandleTypeDef *hi2c,
    int16_t *ax, int16_t *ay, int16_t *az
)
{
    uint8_t buf[6];

    if (i2c_read_bytes(hi2c, MPU9250_ADDR, ACCEL_XOUT_H, buf, 6) != HAL_OK)
        return HAL_ERROR;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}

HAL_StatusTypeDef mpu9250_read_gyro(
    I2C_HandleTypeDef *hi2c,
    int16_t *gx, int16_t *gy, int16_t *gz
)
{
    uint8_t buf[6];

    if (i2c_read_bytes(hi2c, MPU9250_ADDR, GYRO_XOUT_H, buf, 6) != HAL_OK)
        return HAL_ERROR;

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}
