/*
 * vertical_ekf.h
 *
 *  Created on: Jan 18, 2026
 *      Author: sidhu
 */


#ifndef VERTICAL_EKF_H
#define VERTICAL_EKF_H

typedef struct
{
    float h;     // altitude estimate (m)
    float vz;    // vertical velocity estimate (m/s)
} VerticalState_t;

void VerticalEKF_Init(VerticalState_t *s);
void VerticalEKF_Update(VerticalState_t *s,
                        float ax, float ay, float az,
                        float roll, float pitch,
                        float baro_alt,
                        float dt);

#endif


