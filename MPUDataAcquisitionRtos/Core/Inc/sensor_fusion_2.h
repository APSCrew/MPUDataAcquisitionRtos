/*
 * sensor_fusion_2.h
 *
 * Created: 10/09/2014 10:27:54
 *  Author: Luca
 */ 


#ifndef SENSOR_FUSION_2_H_
#define SENSOR_FUSION_2_H_

#include <math.h>
#include "MPU9150.h"

#define MAX_THETA 1.3963   //  80�
#define MIN_THETA -1.3963  // -80�
#define SAT_THETA(x) (fmin(MAX_THETA, fmax(MIN_THETA, x)))

void getEulerAngles(float *acc, float *gyr, float *mag, float *eulerAngle);


void getRollPitch(float *acc, float *gyr, float *rollPitch);
void getYaw(float *mag,  float *gyr, float phi_am, float theta_am, float *eulerAngle);



#endif /* SENSOR_FUSION_2_H_ */
