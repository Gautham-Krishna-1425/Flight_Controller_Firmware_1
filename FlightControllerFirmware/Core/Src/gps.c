/*
 * gps.c
 *
 *  Created on: Jan 17, 2026
 *      Author: sidhu
 */


#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char nmea_buf[128];
static uint8_t idx = 0;
static uint32_t gps_byte_count = 0;

static GPS_Data_t gps;
static bool new_data = false;

static double nmea_to_deg(char *s)
{
    double v = atof(s);
    int deg = (int)(v / 100);
    double min = v - deg * 100;
    return deg + min / 60.0;
}

void GPS_Init(void)
{
    memset(&gps, 0, sizeof(gps));
}

void GPS_ProcessByte(uint8_t c)
{
    gps_byte_count++;   // count incoming bytes

    if (c == '$')
    {
        idx = 0;
        nmea_buf[idx++] = c;
    }
    else if (c == '\n')
    {
        nmea_buf[idx] = 0;

        if (strstr(nmea_buf, "GPGGA"))
        {
            char *p = strtok(nmea_buf, ",");
            uint8_t field = 0;

            while (p)
            {
                if (field == 2) gps.latitude = nmea_to_deg(p);
                if (field == 3 && p[0] == 'S') gps.latitude = -gps.latitude;
                if (field == 4) gps.longitude = nmea_to_deg(p);
                if (field == 5 && p[0] == 'W') gps.longitude = -gps.longitude;
                if (field == 6) gps.fix = atoi(p);
                if (field == 7) gps.satellites = atoi(p);
                if (field == 9) gps.altitude = atof(p);
                p = strtok(NULL, ",");
                field++;
            }
        }

        if (strstr(nmea_buf, "GPRMC"))
        {
            char *p = strtok(nmea_buf, ",");
            uint8_t field = 0;

            while (p)
            {
                if (field == 7) gps.speed_mps = atof(p) * 0.5144f;
                if (field == 8) gps.course_deg = atof(p);
                p = strtok(NULL, ",");
                field++;
            }
        }

        new_data = true;
        idx = 0;
    }
    else if (idx < sizeof(nmea_buf) - 1)
    {
        nmea_buf[idx++] = c;
    }
}

bool GPS_IsDetected(void)
{
    return gps_byte_count > 100;
}

bool GPS_HasFix(void)
{
    return gps.fix > 0;
}

bool GPS_GetData(GPS_Data_t *out)
{
    if (!new_data) return false;
    *out = gps;
    new_data = false;
    return true;
}
uint8_t GPS_GetSatelliteCount(void)
{
    return gps.satellites;
}
