/*
 * motor_pwm.h
 *
 *  Created on: Feb 2, 2026
 *      Author: sidhu
 */


#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "stm32f4xx_hal.h"

#define PWM_MIN_US 1000
#define PWM_MAX_US 2000

void MotorPWM_Init(void);
void MotorPWM_Write(float m1, float m2, float m3, float m4);
void MotorPWM_Disarm(void);

#endif

