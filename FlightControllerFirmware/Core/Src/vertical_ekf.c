/*
 * vertical_ekf.c
 *
 *  Created on: Jan 18, 2026
 *      Author: sidhu
 */
#include "vertical_ekf.h"
#include <math.h>

/* Steady-state Kalman gains (tune later) */
#define K_H   0.08f
#define K_VZ  0.02f

void VerticalEKF_Init(VerticalState_t *s)
{
    s->h  = 0.0f;
    s->vz = 0.0f;
}
void VerticalEKF_Update(VerticalState_t *s,
                        float ax, float ay, float az,
                        float roll, float pitch,
                        float baro_alt,
                        float dt)
{
    float g = 9.81f;
#define DEG2RAD 0.01745329251f

    /* FIX 2: convert degrees to radians before trig */
    float roll_r  = roll  * DEG2RAD;
    float pitch_r = pitch * DEG2RAD;

    /* Convert raw accel (MPU6050 ±2g) to m/s² */
    float ax_m = (ax / 16384.0f) * g;
    float ay_m = (ay / 16384.0f) * g;
    float az_m = (az / 16384.0f) * g;

    /* Rotate body accel to world Z */
    float az_world =
        -sinf(pitch_r)               * ax_m +
         sinf(roll_r)*cosf(pitch_r)  * ay_m +
         cosf(roll_r)*cosf(pitch_r)  * az_m;

    /* FIX 1: subtract 9.81, not 1.0 — result is net vertical accel */
    az_world -= 9.81f;

    /* Prediction */
    s->h  += s->vz * dt + 0.5f * az_world * dt * dt;
    s->vz += az_world * dt;

    /* Baro correction */
    float err = baro_alt - s->h;
    s->h  += K_H  * err;
    s->vz += K_VZ * err;
}
