/*
 * altitude_pid.h
 *
 *  Created on: Jan 19, 2026
 *      Author: sidhu
 */

#ifndef ALTITUDE_PID_H
#define ALTITUDE_PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float out_min, out_max;
} AltPID_t;

void AltPID_Reset(AltPID_t *pid);

void AltPID_Init(AltPID_t *p);
float AltPID_Update(AltPID_t *p,
                    float target_h,
                    float h,
                    float vz,
                    float dt);


#endif

