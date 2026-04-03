/*
 * ppm.h
 *
 *  Created on: Feb 6, 2026
 *      Author: sidhu
 */


#ifndef PPM_H
#define PPM_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

void PPM_InputCaptureCallback(TIM_HandleTypeDef *htim);


#define PPM_MAX_CH 8

void     PPM_Init(void);
uint8_t  PPM_IsFrameReady(void);
uint16_t PPM_GetChannel(uint8_t ch);

#endif
