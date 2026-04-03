/*
 * ibus.h
 *
 *  Created on: Feb 5, 2026
 *      Author: sidhu
 */
#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>

void IBUS_Init(void);
void IBUS_ProcessByte(uint8_t b);

uint8_t  IBUS_IsFrameReady(void);
uint16_t IBUS_GetChannel(uint8_t ch);

#endif
