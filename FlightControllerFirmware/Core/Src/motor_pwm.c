/*
 * motor_pwm.c
 *
 *  Created on: Feb 2, 2026
 *      Author: sidhu
 */
#include "motor_pwm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

static uint32_t pwm_from_norm(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return PWM_MIN_US + (uint32_t)(x * (PWM_MAX_US - PWM_MIN_US));
}

void MotorPWM_Init(void)
{
    __HAL_TIM_MOE_ENABLE(&htim1);          // Required for TIM1 (advanced timer)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    MotorPWM_Write(0.0f, 0.0f, 0.0f, 0.0f); // Send 1000us to arm SimonK ESCs
    HAL_Delay(3000);
}

void MotorPWM_Write(float m1, float m2, float m3, float m4)
{
    // MOE_ENABLE removed — belongs only in Init
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_from_norm(m1)); // M1 BL
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_from_norm(m2)); // M2 BR
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_from_norm(m3)); // M3 FR
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_from_norm(m4)); // M4 FL
}

void MotorPWM_Disarm(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_MIN_US);
}
