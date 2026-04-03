/*
 * mpu6050.h
 *
 *  Created on: Dec 18, 2025
 *      Author: sidhu
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

/* Gyro calibration offsets — defined in mpu6050.c */
extern float gyro_x_offset;
extern float gyro_y_offset;
extern float gyro_z_offset;

extern uint8_t mpu_angle_initialized;
void MPU6050_ResetAngleEstimator(void);

/* I2C address and registers */
#define MPU6050_ADDR     (0x68 << 1)
#define MPU6050_WHO_AM_I  0x75
#define MPU6050_PWR_MGMT_1 0x6B

typedef enum
{
    MPU_OK = 0,
    MPU_ERROR
} MPU_Status_t;

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} MPU6050_RawData_t;

MPU_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t      MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c);
MPU_Status_t MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data);
void         MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c);
void         MPU6050_ComputeAngles(int16_t ax, int16_t ay, int16_t az,
                                   int16_t gx, int16_t gy, int16_t gz,
                                   float dt,
                                   float *roll, float *pitch, float *yaw);

#endif  /* MPU6050_H */   // ← moved to here
