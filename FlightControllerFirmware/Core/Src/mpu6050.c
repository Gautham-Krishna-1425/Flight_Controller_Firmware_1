/*
 * mpu6050.c
 *
 *  Created on: Dec 18, 2025
 *      Author: sidhu
 */


#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

#define IMU_TIMEOUT_MS  5

/* ------------------------------------------------------------------ */
uint8_t MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1,
						 IMU_TIMEOUT_MS) != HAL_OK)
        return 0xFF;
    return who;
}

/* ------------------------------------------------------------------ */
MPU_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = MPU6050_ReadWhoAmI(hi2c);
    if (who != 0x68)
    {
        printf("WHO_AM_I wrong: 0x%02X\r\n", who);
        return MPU_ERROR;
    }

    uint8_t data;

    /* Wake up — use gyro X as clock source (more stable than internal RC) */
    data = 0x01;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
						  IMU_TIMEOUT_MS) != HAL_OK)
        return MPU_ERROR;

    HAL_Delay(10);

    /*
     * DLPF = 3 → Gyro BW 42Hz, Accel BW 44Hz
     * Attenuates motor vibration before aliasing into 100Hz control loop.
     */
    data = 0x03;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
						  IMU_TIMEOUT_MS) != HAL_OK)
        return MPU_ERROR;

    /* Gyro ±250 dps — scale factor 131 LSB/(deg/s) */
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
						  IMU_TIMEOUT_MS) != HAL_OK)
        return MPU_ERROR;

    /* Accel ±2g — scale factor 16384 LSB/g */
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
						  IMU_TIMEOUT_MS) != HAL_OK)
        return MPU_ERROR;

    return MPU_OK;
}

/* ------------------------------------------------------------------ */
MPU_Status_t MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data)
{
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B,
                         I2C_MEMADD_SIZE_8BIT, buf, 14,
						 IMU_TIMEOUT_MS) != HAL_OK)
        return MPU_ERROR;

    data->ax = (int16_t)(buf[0]  << 8 | buf[1]);
    data->ay = (int16_t)(buf[2]  << 8 | buf[3]);
    data->az = (int16_t)(buf[4]  << 8 | buf[5]);
    /* buf[6,7] = temperature — intentionally skipped */
    data->gx = (int16_t)(buf[8]  << 8 | buf[9]);
    data->gy = (int16_t)(buf[10] << 8 | buf[11]);
    data->gz = (int16_t)(buf[12] << 8 | buf[13]);

    return MPU_OK;
}

/* ------------------------------------------------------------------ */
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c)
{
    MPU6050_RawData_t imu;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    /* Discard first 200 samples — thermal settling (~400ms) */
    for (int i = 0; i < 200; i++) {
        MPU6050_ReadRaw(hi2c, &imu);
        HAL_Delay(2);
    }

    /* Accumulate 1000 samples (~2000ms) for low-noise offset estimate */
    const int samples = 1000;
    for (int i = 0; i < samples; i++) {
        MPU6050_ReadRaw(hi2c, &imu);
        gx_sum += imu.gx;
        gy_sum += imu.gy;
        gz_sum += imu.gz;
        HAL_Delay(2);
    }

    gyro_x_offset = (float)gx_sum / (float)samples;
    gyro_y_offset = (float)gy_sum / (float)samples;
    gyro_z_offset = (float)gz_sum / (float)samples;

    printf("Gyro offsets: GX=%.2f GY=%.2f GZ=%.2f\r\n",
           gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

/* ------------------------------------------------------------------ */
/*
 * ADD: Reset function so the angle estimator can be re-seeded without
 * power cycle. Call after MPU re-init or after a crash recovery.
 */
void MPU6050_ResetAngleEstimator(void)
{
    /*
     * The static 'initialized' flag inside ComputeAngles is reset to 0
     * by setting this file-scope flag. On the next ComputeAngles call
     * the integrator will be re-seeded from the accelerometer.
     * This is achieved by clearing the flag via the same static trick.
     */
    /* We use a shared extern flag — declare it static here and
       expose via the header for external reset capability.          */
    extern uint8_t mpu_angle_initialized;
    mpu_angle_initialized = 0;
}

/* ------------------------------------------------------------------ */
/*
 * Angle sign conventions (IMPORTANT — must match attitude_controller):
 *
 *   roll  > 0 : right side DOWN  (quad rolls right)
 *   pitch > 0 : nose UP          (quad pitches back)  ← verify on bench!
 *   yaw   > 0 : nose rotates CCW (gyro Z integrated)
 *
 * BENCH TEST before first flight:
 *   Flat on desk  → roll ≈ 0, pitch ≈ 0
 *   Tilt right    → roll goes positive
 *   Tilt nose up  → pitch should go positive (if negative, flip sign of
 *                   pitch_acc formula and gy_dps integration)
 */
uint8_t mpu_angle_initialized = 0;  /* exposed for ResetAngleEstimator */

void MPU6050_ComputeAngles(int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz,
                           float dt,
                           float *roll, float *pitch, float *yaw)
{
    static float roll_gyro  = 0.0f;
    static float pitch_gyro = 0.0f;
    static float yaw_gyro   = 0.0f;

    /* Convert accel to g */
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    /*
     * Accel-derived angles.
     * pitch_acc = atan2f(-ax_g, ...) → positive when nose pitches UP.
     * If bench test shows nose-up gives NEGATIVE pitch, change to:
     *   pitch_acc = atan2f(ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * ...
     */
    float roll_acc  = atan2f(ay_g, az_g) * 180.0f / M_PI;
    float pitch_acc = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;

    /* One-time init — seed gyro integrator from accel */
    if (!mpu_angle_initialized)
    {
        roll_gyro  = roll_acc;
        pitch_gyro = pitch_acc;
        yaw_gyro   = 0.0f;
        *roll  = roll_acc;
        *pitch = pitch_acc;
        *yaw   = 0.0f;
        mpu_angle_initialized = 1;
        return;
    }

    /* Gyro rates (deg/s) with calibration offsets removed */
    float gx_dps = (gx - gyro_x_offset) / 131.0f;
    float gy_dps = (gy - gyro_y_offset) / 131.0f;
    float gz_dps = (gz - gyro_z_offset) / 131.0f;

    /* Integrate gyro */
    roll_gyro  += gx_dps * dt;
    pitch_gyro += gy_dps * dt;
    yaw_gyro   += gz_dps * dt;

    /* Complementary filter — 96% gyro, 4% accel */
    *roll  = 0.96f * roll_gyro  + 0.04f * roll_acc;
    *pitch = 0.96f * pitch_gyro + 0.04f * pitch_acc;
    *yaw   = yaw_gyro;

    /* Feed fused output back into integrator to prevent divergence */
    roll_gyro  = *roll;
    pitch_gyro = *pitch;
    yaw_gyro   = *yaw;

    /* Clamp output AND integrator together — prevents state mismatch on tip-over */
    if (*roll >  85.0f) { *roll =  85.0f; roll_gyro =  85.0f; }
    if (*roll < -85.0f) { *roll = -85.0f; roll_gyro = -85.0f; }

    if (*pitch >  85.0f) { *pitch =  85.0f; pitch_gyro =  85.0f; }
    if (*pitch < -85.0f) { *pitch = -85.0f; pitch_gyro = -85.0f; }

    if (*yaw >  180.0f) { *yaw -= 360.0f; yaw_gyro = *yaw; }
    if (*yaw < -180.0f) { *yaw += 360.0f; yaw_gyro = *yaw; }
}
