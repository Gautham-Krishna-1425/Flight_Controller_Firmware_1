/*
 * motor_mixer.h
 *
 *  Created on: Feb 2, 2026
 *      Author: sidhu
 */
#include <stddef.h>

#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
} MotorOutput_t;

/**
 * @brief Mix throttle and attitude control into motor commands
 * @param throttle 0.0 – 1.0
 * @param u_roll   roll control output
 * @param u_pitch  pitch control output
 * @param u_yaw    yaw control output
 * @param out      motor outputs (0.0 – 1.0)
 */
void MotorMixer_X(float throttle,
                  float u_roll,
                  float u_pitch,
                  float u_yaw,
                  MotorOutput_t *out);

#endif


