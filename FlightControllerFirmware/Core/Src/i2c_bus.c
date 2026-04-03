/*
 * i2c_bus.c
 *
 *  Created on: Dec 18, 2025
 *      Author: sidhu
 */

#include "i2c_bus.h"

/* ================= I2C SCAN ================= */

void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    printf("Scanning I2C bus...\r\n");

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 50) == HAL_OK)
        {
            printf("Found device at 0x%02X\r\n", addr);
        }
    }

    printf("Scan done\r\n");
}

/* ================= I2C LOW-LEVEL HELPERS ================= */

HAL_StatusTypeDef i2c_read_reg(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t *data
)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        devAddr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef i2c_read_bytes(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t *buf,
    uint16_t len
)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        devAddr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef i2c_write_reg(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAddr,
    uint8_t reg,
    uint8_t value
)
{
    return HAL_I2C_Mem_Write(
        hi2c,
        devAddr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );
}
