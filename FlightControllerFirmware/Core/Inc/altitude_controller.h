/*
 * altitude_controller.h
 *
 *  Created on: Jan 14, 2026
 *      Author: sidhu
 */
#ifndef ALTITUDE_CONTROLLER_H
#define ALTITUDE_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_err;

    float out_min;
    float out_max;
} AltitudeController_t;

/* Init */
void AltitudeController_Init(AltitudeController_t *c);

/* Outer loop: Altitude → Desired vertical velocity */
float AltitudeOuter_Update(float desired_alt, float h_est);

/* Inner loop: Vertical velocity PID */
float AltitudeVelocityPID_Update(AltitudeController_t *c,
                                 float desired_vz,
                                 float vz,
                                 float dt);

#endif
