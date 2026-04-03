/*
 * gps.h
 *
 *  Created on: Jan 17, 2026
 *      Author: sidhu
 */

#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef struct
{
    double latitude;
    double longitude;
    float altitude;
    float speed_mps;
    float course_deg;
    uint8_t satellites;
    uint8_t fix;
} GPS_Data_t;

void GPS_Init(void);
void GPS_ProcessByte(uint8_t c);
bool GPS_GetData(GPS_Data_t *out);
bool GPS_IsDetected(void);
bool GPS_HasFix(void);
uint8_t GPS_GetSatelliteCount(void);

#endif


