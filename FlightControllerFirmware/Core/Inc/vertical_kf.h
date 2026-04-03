/*
 * vertical_kf.h
 *
 *  Created on: Jan 19, 2026
 *      Author: sidhu
 */

#ifndef INC_VERTICAL_KF_H_
#define INC_VERTICAL_KF_H_

typedef struct {
    float h;      // altitude (m)
    float vz;     // vertical speed (m/s)
} VerticalKF_t;

void VerticalKF_Init(VerticalKF_t *kf);
void VerticalKF_Update(VerticalKF_t *kf,
                       float az,        // vertical accel (m/s²)
                       float sonar_h,   // baro altitude (m)
                       float dt);


#endif /* INC_VERTICAL_KF_H_ */
