/*
 * attitude controller.h
 *
 *  Created on: Jan 4, 2026
 *      Author: sidhu
 */

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

typedef struct {
    float K_po;            // outer P (angle → rate)
    float K_pi;            // inner P (rate)
    float K_ii;            // inner I
    float K_di;            // inner D

    float integrator;
    float prev_rate_meas;  // renamed: stores last rate measurement
    float d_filt;

    float integrator_min;
    float integrator_max;
    float alpha;           // D LPF time constant (seconds)
} AttitudeAxis_t;

void  Attitude_Init(AttitudeAxis_t *a);
float Attitude_Update(AttitudeAxis_t *a,
                      float desired_angle,
                      float angle_est,
                      float rate_est,
                      float dt);
#endif
