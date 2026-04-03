/*
 * altitude_pid.c
 *
 *  Created on: Jan 19, 2026
 *      Author: sidhu
 */
#include "altitude_pid.h"

void AltPID_Init(AltPID_t *p)
{
    p->integral = 0.0f;
}
void AltPID_Reset(AltPID_t *p)
{
    p->integral = 0.0f;
}


float AltPID_Update(AltPID_t *p,
                    float target_h,
                    float h,
                    float vz,
                    float dt)
{
    float e = target_h - h;

    // Integral (altitude bias)
    float u_unsat =
        p->kp * e +
        p->ki * p->integral -
        p->kd * vz;

    /* Integrate only if not saturating */
    if (!((u_unsat > p->out_max && e > 0) ||
          (u_unsat < p->out_min && e < 0)))
    {
        p->integral += e * dt;
    }

    if (p->integral >  1.0f) p->integral =  1.0f;
    if (p->integral < -1.0f) p->integral = -1.0f;

    float u =
        p->kp * e +
        p->ki * p->integral -
        p->kd * vz;

    // Output clamp
    if (u > p->out_max) u = p->out_max;
    if (u < p->out_min) u = p->out_min;

    return u;
}


