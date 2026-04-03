/*
 * vertical_kf.c
 *
 *  Created on: Jan 19, 2026
 *      Author: sidhu
 */
#include "vertical_kf.h"

#define K_H   0.20f
#define K_VZ  0.01f   // VERY small


void VerticalKF_Init(VerticalKF_t *kf)
{
    kf->h  = 0.0f;
    kf->vz = 0.0f;
}

void VerticalKF_Update(VerticalKF_t *kf,
                       float az,
                       float baro_h,
                       float dt)
{
    // Prediction (constant velocity model)
    kf->h += kf->vz * dt;

    // Correction from baro
    float err = baro_h - kf->h;

    kf->h  += K_H  * err;
    kf->vz += K_VZ * err;
}
