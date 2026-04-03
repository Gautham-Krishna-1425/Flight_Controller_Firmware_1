/*
 * altitude_controller.c
 *
 *  Created on: Jan 14, 2026
 *      Author: sidhu
 */
#include "altitude_controller.h"

void AltitudeController_Init(AltitudeController_t *c)
{
    c->integrator = 0.0f;
    c->prev_err   = 0.0f;
}

float AltitudeOuter_Update(float desired_alt, float h_est)
{
    float K_alt = 1.0f;     // altitude → velocity gain (tune later)
    return K_alt * (desired_alt - h_est);
}

float AltitudeVelocityPID_Update(AltitudeController_t *c,
                                 float desired_vz,
                                 float vz,
                                 float dt)
{
    float err = desired_vz - vz;

    c->integrator += err * dt;

    float deriv = (err - c->prev_err) / dt;
    c->prev_err = err;

    float out = c->kp * err +
                c->ki * c->integrator +
                c->kd * deriv;

    if(out > c->out_max) out = c->out_max;
    if(out < c->out_min) out = c->out_min;

    return out;
}
