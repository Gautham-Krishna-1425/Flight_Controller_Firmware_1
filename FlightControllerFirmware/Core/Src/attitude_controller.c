/*
 * attitude controller.c
 *
 *  Created on: Jan 4, 2026
 *      Author: sidhu
 */
#include "attitude_controller.h"

void Attitude_Init(AttitudeAxis_t *a)
{
    a->integrator     = 0.0f;
    a->prev_rate_meas = 0.0f;
    a->d_filt         = 0.0f;
    a->integrator_min = -0.15f;
    a->integrator_max =  0.15f;
    a->alpha          =  0.005f;   // D-term time constant seconds ~1.6Hz cutoff
}

float Attitude_Update(
    AttitudeAxis_t *a,
    float desired_angle,
    float angle_est,
    float rate_est,
    float dt)
{
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.05f)   dt = 0.05f;

    // Outer loop: angle -> rate demand
    float angle_err = desired_angle - angle_est;
    if (angle_err >  30.0f) angle_err =  30.0f;
    if (angle_err < -30.0f) angle_err = -30.0f;

    float rate_des = a->K_po * angle_err;
    if (rate_des >  150.0f) rate_des =  150.0f;
    if (rate_des < -150.0f) rate_des = -150.0f;

    // Inner loop: rate error
    float rate_err = rate_des - rate_est;

    // P term
    float u_p = a->K_pi * rate_err;

    // I term with anti-windup
    a->integrator += a->K_ii * rate_err * dt;
    if (a->integrator > a->integrator_max) a->integrator = a->integrator_max;
    if (a->integrator < a->integrator_min) a->integrator = a->integrator_min;
    float u_i = a->integrator;

    // D term on rate ERROR delta — correct for cascaded controller
    // prev_rate_meas now stores previous rate_err (renamed for clarity)
    float rate_meas_delta = rate_est - a->prev_rate_meas;
    a->prev_rate_meas = rate_est;

    if (rate_meas_delta >  200.0f) rate_meas_delta =  200.0f;
    if (rate_meas_delta < -200.0f) rate_meas_delta = -200.0f;

    float u_d_raw = -a->K_di * (rate_meas_delta / dt);

    // Correct 1st-order LPF: tau = alpha
    float lpf_coeff = dt / (a->alpha + dt);
    a->d_filt += lpf_coeff * (u_d_raw - a->d_filt);
    float u_d = a->d_filt;

    // Sum and clamp (conservative for first flights)
    float u_total = u_p + u_i + u_d;
    if (u_total >  0.3f) u_total =  0.3f;
    if (u_total < -0.3f) u_total = -0.3f;

    return u_total;
}
