/*
 * ibus.c
 *
 *  Created on: Feb 5, 2026
 *      Author: sidhu
 */
#include "ibus.h"
#include "stm32f4xx_hal.h"

#define IBUS_FRAME_LEN 32
#define IBUS_MAX_CH    14

static uint8_t  ibus_buf[IBUS_FRAME_LEN];
static volatile uint8_t  ibus_idx = 0;
static volatile uint16_t ibus_ch[IBUS_MAX_CH];
static volatile uint8_t  frame_ready = 0;
extern uint32_t last_rc_time;
static inline uint16_t rc_map(uint16_t v)
{
    const uint16_t RAW_MIN = 1200;
    const uint16_t RAW_MAX = 31500;

    if (v < RAW_MIN) v = RAW_MIN;
    if (v > RAW_MAX) v = RAW_MAX;

    return 1000 + (v - RAW_MIN) * 1000 / (RAW_MAX - RAW_MIN);
}


void IBUS_Init(void)
{
    ibus_idx = 0;
    frame_ready = 0;
}
void IBUS_ProcessByte(uint8_t b)
{
    static uint8_t buf[32];
    static uint8_t idx = 0;

    if (idx == 0)
    {
        if (b != 0x20)   // frame length
            return;
    }

    buf[idx++] = b;

    if (idx < 32)
        return;

    // checksum
    uint16_t sum = 0xFFFF;
    for (int i = 0; i < 30; i++)
        sum -= buf[i];

    uint16_t rx_sum = buf[30] | (buf[31] << 8);

    if (sum == rx_sum)
    {
        for (int i = 0; i < 14; i++)
        {
            ibus_ch[i] =
                buf[2 + i*2] |
               (buf[3 + i*2] << 8);
        }

        frame_ready = 1;

        // ===== CRITICAL: update RC alive timer =====
        last_rc_time = HAL_GetTick();
    }

    idx = 0;
}


uint8_t IBUS_IsFrameReady(void)
{
    if (frame_ready)
    {
        frame_ready = 0;
        return 1;
    }
    return 0;
}

uint16_t IBUS_GetChannel(uint8_t ch)
{
    if (ch >= IBUS_MAX_CH)
        return 1500;
    return ibus_ch[ch];
}
