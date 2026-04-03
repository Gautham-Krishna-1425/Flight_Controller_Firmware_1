/*
 * ppm.c
 *
 *  Created on: Feb 6, 2026
 *      Author: sidhu
 */

#include "ppm.h"
#include "stm32f4xx_hal.h"

static volatile uint16_t ppm_ch[PPM_MAX_CH];
static volatile uint8_t  ppm_count = 0;
static volatile uint32_t last_ppm_time = 0;
static volatile uint8_t  frame_ready = 0;

void PPM_Init(void)
{
    ppm_count = 0;
    frame_ready = 0;
}

void PPM_InputCaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 ||
        htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_1)
        return;

    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t diff = (now >= last_ppm_time)
                  ? (now - last_ppm_time)
                  : (0xFFFF - last_ppm_time + now);

    last_ppm_time = now;

    printf("PPM diff = %lu\r\n", diff);


    if (diff > 3000)
    {
        ppm_count = 0;
        frame_ready = 1;
    }
    else
    {
        if (ppm_count < PPM_MAX_CH)
            ppm_ch[ppm_count++] = diff;
    }
}


uint8_t PPM_IsFrameReady(void)
{
    if (frame_ready)
    {
        frame_ready = 0;
        return 1;
    }
    return 0;
}

uint16_t PPM_GetChannel(uint8_t ch)
{
    if (ch >= PPM_MAX_CH)
        return 1500;
    return ppm_ch[ch];
}

