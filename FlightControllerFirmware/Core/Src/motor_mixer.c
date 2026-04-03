/*
 * motor_mixer.c
 *
 *  Created on: Feb 2, 2026
 *      Author: sidhu
 */

#include "motor_mixer.h"

static inline float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void MotorMixer_X(float throttle,
                  float u_roll,
                  float u_pitch,
                  float u_yaw,
                  MotorOutput_t *out)
{
    if (out == NULL) return;

    // Cap throttle to leave headroom for attitude control
    throttle = clamp01(throttle) * 0.85f;

    /*
     *        FRONT
     *   M4(FL,CCW)  M3(FR,CW)
     *   M1(BL,CW)   M2(BR,CCW)
     *
     * Roll  right (+u_roll) : L motors UP   → M1↑ M4↑ | M2↓ M3↓
     * Pitch fwd   (+u_pitch): front UP      → M3↑ M4↑ | M1↓ M2↓
     * Yaw   CW    (+u_yaw)  : CW motors DN  → M1↓ M3↓ | M2↑ M4↑
     */
    float m1 = throttle + u_roll - u_pitch - u_yaw;  // BL CW
    float m2 = throttle - u_roll - u_pitch + u_yaw;  // BR CCW
    float m3 = throttle - u_roll + u_pitch - u_yaw;  // FR CW
    float m4 = throttle + u_roll + u_pitch + u_yaw;  // FL CCW

    out->m1 = clamp01(m1);
    out->m2 = clamp01(m2);
    out->m3 = clamp01(m3);
    out->m4 = clamp01(m4);
}
