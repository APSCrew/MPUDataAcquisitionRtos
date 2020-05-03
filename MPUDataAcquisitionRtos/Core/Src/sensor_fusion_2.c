/*
 * sensor_fusion_2.c
 *
 * Created: 10/09/2014 10:27:32
 *  Author: Luca
 */ 


#include "sensor_fusion_2.h"




//#define   TS 0.0614 // 5/81.38
//#define ALFA 0.1069


//float a = (float) ( 0.0114 );
//float b = (float) ( 0.0059 );
//float c = (float) ( 1.7862 );
//float d = (float) ( 0.7976 );
//
//float eulerAngle_k_2[3] = {0.0, 0.0, 0.0};
//float eulerAngle_k_1[3] = {0.0, 0.0, 0.0};


float alfa = 0.9628;
float ts = 0.0614;

float phi_k_1 = 0;
float theta_k_1 = 0;
float psi_k_1 = 0;


void getEulerAngles(float *acc, float *gyr, float *mag, float *eulerAngle)
{
	// get low frequency measure data from accelerometer and magnetometer
	float phi_am   = (float) ( atan2f(acc[1], acc[2]) );	            // roll  angle in [rad]
	float theta_am = (float) ( -atan2f(acc[0], acc[2]) );		        // pitch angle in [rad]
	
	theta_am = SAT_THETA(theta_am);
	
	float hx = (float) (                cosf(theta_am) * mag[0] +                                        sinf(theta_am) * mag[2] );
	float hy = (float) ( (sinf(phi_am)*sinf(theta_am)) * mag[0] + cosf(phi_am) * mag[1] - (cosf(theta_am)*sinf(phi_am)) * mag[2] );

	float psi_am = (float) ( -atan2f(hy, hx) );                                      // yaw   angle (psi) in [rad]

	// get high frequency measure data from gyroscope
	// NB only translate the vector from body frame to inertial frame
	float phi_g   = (float) ( gyr[0] + (gyr[1]*sinf(phi_am)*tanf(theta_am)) + (gyr[2]*cosf(phi_am)*tanf(theta_am)) );
	float theta_g = (float) (           gyr[1]*cosf(phi_am)                 -  gyr[2]*sinf(phi_am) );
	float psi_g   = (float) (          (gyr[1]*sinf(phi_am)                 +  gyr[2]*cosf(phi_am))/cosf(theta_am) );

	// apply sensor fusion second order filter
	//float phi   = (float) ( phi_am*a   + phi_g*b   + eulerAngle_k_1[0]*c - eulerAngle_k_2[0]*d );
	//float theta = (float) ( theta_am*a + theta_g*b + eulerAngle_k_1[1]*c - eulerAngle_k_2[1]*d );
	//float psi   = (float) ( psi_am*a   + psi_g*b   + eulerAngle_k_1[2]*c - eulerAngle_k_2[2]*d );
//
	//eulerAngle_k_2[0] = eulerAngle_k_1[0];
	//eulerAngle_k_2[1] = eulerAngle_k_1[1];
	//eulerAngle_k_2[2] = eulerAngle_k_1[2];
//
	//eulerAngle_k_1[0] = phi;
	//eulerAngle_k_1[1] = theta;
	//eulerAngle_k_1[2] = psi;
	
	float phi_k   = (float) ( alfa*phi_k_1   + (1-alfa)*phi_am   + alfa*phi_g*ts );
	phi_k_1 = phi_k;
	
	float theta_k = (float) ( alfa*theta_k_1 + (1-alfa)*theta_am + alfa*theta_g*ts );
	theta_k_1 = theta_k;
	
	float psi_k   = (float) ( alfa*psi_k_1   + (1-alfa)*psi_am   + alfa*psi_g*ts );
	psi_k_1 = psi_k;	

	eulerAngle[0] = phi_k;
	eulerAngle[1] = theta_k;
	eulerAngle[2] = psi_k;
}

void getRollPitch(float *acc, float *gyr, float *eulerAngle)
{
	//// get low frequency measure data from accelerometer and magnetometer
	//float phi_am   = (float) (  atan2f(acc[1], acc[2]) );	            // roll  angle in [rad]
	//float theta_am = (float) ( -atan2f(acc[0], acc[2]) );		        // pitch angle in [rad]
		//
	//theta_am = SAT_THETA(theta_am);
	//
	//// get high frequency measure data from gyroscope
	//// NB only translate the vector from body frame to inertial frame
	//float phi_g   = (float) ( gyr[0] + (gyr[1]*sinf(phi_am)*tanf(theta_am)) + (gyr[2]*cosf(phi_am)*tanf(theta_am)) );
	//float theta_g = (float) (           gyr[1]*cosf(phi_am)                 -  gyr[2]*sinf(phi_am) );
	//
	//// apply sensor fusion second order filter
	//float phi   = (float) ( phi_am*a   + phi_g*b   + eulerAngle_k_1[0]*c - eulerAngle_k_2[0]*d );
	//float theta = (float) ( theta_am*a + theta_g*b + eulerAngle_k_1[1]*c - eulerAngle_k_2[1]*d );
	//
	//eulerAngle_k_2[0] = eulerAngle_k_1[0];
	//eulerAngle_k_2[1] = eulerAngle_k_1[1];
//
	//eulerAngle_k_1[0] = phi;
	//eulerAngle_k_1[1] = theta;
//
	//eulerAngle[0] = phi;
	//eulerAngle[1] = theta;	
}

void getYaw(float *mag, float *gyr, float phi_am, float theta_am, float *eulerAngle)
{
	//float hx = (float) (                cosf(theta_am) * mag[0] +                                        sinf(theta_am) * mag[2] );
	//float hy = (float) ( (sinf(phi_am)*sinf(theta_am)) * mag[0] + cosf(phi_am) * mag[1] - (cosf(theta_am)*sinf(phi_am)) * mag[2] );
//
	//float psi_am = (float) ( -atan2f(hy, hx) );    // yaw   angle (psi) in [rad]
	//// get high frequency measure data from gyroscope
	//// NB only translate the vector from body frame to inertial frame
	//float psi_g   = (float) (          (gyr[1]*sinf(phi_am)                 +  gyr[2]*cosf(phi_am))/cosf(theta_am) );
	//
	//// apply sensor fusion second order filter
	//float psi   = (float) ( psi_am*a   + psi_g*b   + eulerAngle_k_1[2]*c - eulerAngle_k_2[2]*d );
//
	//eulerAngle_k_2[2] = eulerAngle_k_1[2];
	//eulerAngle_k_1[2] = psi;
//
	//eulerAngle[2] = psi;	
}
