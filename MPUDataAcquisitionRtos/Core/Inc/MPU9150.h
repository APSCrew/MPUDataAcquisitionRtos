/*
 * MPU9150.h
 *
 * Created: 30/12/2013 10:48:50
 *  Author: Luca
 */

/*

	The register map used in this library it's a part of the register map
	present in the "MPU-6000/MPU-6050 Register Map and Descriptions, rev. 3.2"
	and in the "MPU-9150 Register Map and Descriptions, rev. 4.0".

	The other referencing document used are:
	"MPU-9150 Product Specification, rev. 4.0" and
	"MPU-6000/MPU-6050 Product Specification, rev. 1.0".

	NOTE: Not all registers of the MPU6050 are mapped here!

*/

#include <math.h>
#include "i2c.h"


#ifndef MPU9150_H_
#define MPU9150_H_

#define MPU9150_6050_ADR_0 0b1101000
#define MPU9150_6050_ADR_1 0b1101001

#define MPU9150_AK8975_ADR 0b0001100

/* MPU6050 Conversion Factor */ 
#define GYRO_SENS_UNIT_250  0.00013315 //[rad/s] => (1/cf) * (pi/180) = [rad/s]
#define GYRO_SENS_UNIT_500  0.00026631 //[rad/s] 
#define GYRO_SENS_UNIT_1000 0.00053263 //[rad/s]
#define GYRO_SENS_UNIT_2000 0.00106526 //[rad/s]
//
#define ACC_SENS_UNIT_2  0.00059870 //[m/s^2] => (1/cf) * (9.81) = [m/s^2]
#define ACC_SENS_UNIT_4  0.00119750 //[m/s^2]
#define ACC_SENS_UNIT_8  0.00239501 //[m/s^2]
#define ACC_SENS_UNIT_16 0.00479002 //[m/s^2]

#define ACC_SENS_UNIT_2_G  0.00006103 //[G] => (1/cf) = [G]
#define ACC_SENS_UNIT_4_G  0.00012207 //[G]
#define ACC_SENS_UNIT_8_G  0.00024414 //[G]
#define ACC_SENS_UNIT_16_G 0.00048828 //[G]

#define TEMP_SENS  0.00294117     //[�C]
/*****************************/

/* MPU6050 sensitivity scale factor */
#define GYRO_SENS_250 131.072 //[LSB/(�/s)]
#define GYRO_SENS_500  65.536 //[LSB/(�/s)]
#define GYRO_SENS_1000 32.768 //[LSB/(�/s)]
#define GYRO_SENS_2000 16.384 //[LSB/(�/s)]

#define ACC_SENS_2 16384 //[LSB/g]
#define ACC_SENS_4 8192 //[LSB/g]
#define ACC_SENS_8 4096 //[LSB/g]
#define ACC_SENS_16 2048 //[LSB/g]



/* AK8975 Conversion Factor */
#define MAG_SENS_T 0.00000029   //[T/LSB] --> 1 uT = 10^-6 T
#define MAG_SENS_G 0.00292968   //[G/LSB] ---> le misure vanno prese in Gauss -> vedi documento MEMSENSE...
/****************************/


/* MPU6050 register map */
#define SELF_TEST_X 0x0D
#define XA_TEST_4 7
#define XA_TEST_3 6
#define XA_TEST_2 5
#define XG_TEST_4 4
#define XG_TEST_3 3
#define XG_TEST_2 2
#define XG_TEST_1 1
#define XG_TEST_0 0

#define SELF_TEST_Y 0x0E
#define YA_TEST_4 7
#define YA_TEST_3 6
#define YA_TEST_2 5
#define YG_TEST_4 4
#define YG_TEST_3 3
#define YG_TEST_2 2
#define YG_TEST_1 1
#define YG_TEST_0 0

#define SELF_TEST_Z 0x0F
#define ZA_TEST_4 7
#define ZA_TEST_3 6
#define ZA_TEST_2 5
#define ZG_TEST_4 4
#define ZG_TEST_3 3
#define ZG_TEST_2 2
#define ZG_TEST_1 1
#define ZG_TEST_0 0

#define SELF_TEST_A 0x10
#define XA_TEST_1 5
#define XA_TEST_0 4
#define YA_TEST_1 3
#define YA_TEST_0 2
#define ZA_TEST_1 1
#define ZA_TEST_0 0

#define SAMPLE_RATE_DIVIDER 0x19

#define CONFIG 0x1A
#define EXT_SYNC_SET_0 3
#define EXT_SYNC_SET_1 4
#define EXT_SYNC_SET_2 5
#define DLPF_CFG_0 0
#define DLPF_CFG_1 1
#define DLPF_CFG_2 2

#define GYRO_CONFIG 0x1B
#define XG_ST 7
#define YG_ST 6
#define ZG_ST 5
#define FS_SEL_0 4
#define FS_SEL_1 3

#define ACCEL_CONFIG 0x1C
#define XA_ST 7
#define YA_ST 6
#define ZA_ST 5
#define AFS_SEL_0 4
#define AFS_SEL_1 3
#define ACCEL_HPF_2 2
#define ACCEL_HPF_1 1
#define ACCEL_HPF_0 0

#define FF_THR 0x1D
#define FF_DUR 0x1E

#define MOT_THR 0x1F
#define MOT_DUR 0x20

#define ZRMOT_THR 0x21
#define ZRMOT_DUR 0x22

#define FIFO_EN_REG 0x23  // is FIFO_EN register
#define TEMP_FIFO_EN 7
#define XG_FIFO_EN 6
#define YG_FIFO_EN 5
#define ZG_FIFO_EN 4
#define ACCEL_FIFO_EN 3
#define SLV2_FIFO_EN 2
#define SLV1_FIFO_EN 1
#define SLV0_FIFO_EN 0


#define I2C_MST_CNTR 0x24


#define INT_PIN_CFG 0x37
#define INT_LEVEL 7
#define INT_OPEN 6
#define LATCH_INT_EN 5
#define INT_RD_CLEAR 4
#define FSYNC_INT_LEVEL 3
#define FSYNC_INT_EN 2
#define I2C_BYPASS_EN 1
#define CLKOUT_EN 0

#define INT_ENABLE 0x38
#define FF_EN 7
#define MOT_EN 6
#define ZMOT_EN 5
#define FIFO_OFLOW_EN 4
#define I2C_MST_INT_EN 3
#define DATA_RDY_EN 0

#define INT_STATUS 0x3A
#define FF_INT 7
#define MOT_INT 6
#define ZMOT_INT 5
#define FIFO_OFLOW_INT _EN 4
#define I2C_MST_INT 3
#define DATA_RDY_INT 0

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E

#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44

#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46

#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MOT_DETECT_STATUS 0x61
#define MOT_XNEG 7
#define MOT_XPOS 6
#define MOT_YNEG 5
#define MOT_YPOS 4
#define MOT_ZNEG 3
#define MOT_ZPOS 2
#define MOT_ZRMOT 0

#define SIGNAL_PATH_RESET 0x68
#define GYRO_RESET 2
#define ACCEL_RESET 1
#define TEMP_RESET 0

#define MOT_DETECT_CTRL 0x69
#define ACCEL_ON_DELAY_1 5
#define ACCEL_ON_DELAY_0 4
#define FF_COUNT_1 3
#define FF_COUNT_0 2
#define MOT_COUNT_1 1
#define MOT_COUNT_0 0

#define USER_CTRL 0x6A
#define FIFO_EN 6
#define I2C_MST_EN 5
#define I2C_IF_DIS 4
#define FIFO_RESET 2
#define I2C_MST_RESET 1
#define SIG_COND_RESET 0

#define PWR_MGMT_1 0x6B
#define DEVICE_RESET 7
#define SLEEP 6
#define CYCLE 5
#define TEMP_DIS 3
#define CLKSEL_2 2
#define CLKSEL_1 1
#define CLKSEL_0 0

#define PWR_MGMT_2 0x6C
#define LP_WAKE_CTRL_1 7
#define LP_WAKE_CTRL_2 6
#define STBY_XA 5
#define STBY_YA 4
#define STBY_ZA 3
#define STBY_XG 2
#define STBY_YG 1
#define STBY_ZG 0

#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73

#define FIFO_R_W 0x74

#define WHO_AM_I 0x75

/*****************************************/

/* AK8975 register map */
#define WIA    0x00

#define INFO   0x01
#define INFO_7 7
#define INFO_6 6
#define INFO_5 5
#define INFO_4 4
#define INFO_3 3
#define INFO_2 2
#define INFO_1 1
#define INFO_0 0

#define ST1    0x02
#define DRDY   0

#define HXL    0x03
#define HXH    0x04

#define HYL    0x05
#define HYH    0x06

#define HZL    0x07
#define HZH    0x08

#define ST2    0x09
#define HOFL   3
#define DERR   2

#define CNTL   0x0A
#define MODE_3 3
#define MODE_2 2
#define MODE_1 1
#define MODE_0 0

//#define RSV    0x0B  //do not access

#define ASTC   0x0C
#define SELF 6
//#define TS1    0x0D  //do not access
//#define TS2    0x0E  //do not access

#define I2CDIS 0x0F
#define I2CDIS_0 0 //I2CDIS => I2CDIS_0

#define ASAX   0x10
#define COEFX7 7
#define COEFX6 6
#define COEFX5 5
#define COEFX4 4
#define COEFX3 3
#define COEFX2 2
#define COEFX1 1
#define COEFX0 0

#define ASAY   0x11
#define COEFY7 7
#define COEFY6 6
#define COEFY5 5
#define COEFY4 4
#define COEFY3 3
#define COEFY2 2
#define COEFY1 1
#define COEFY0 0

#define ASAZ   0x12
#define COEFZ7 7
#define COEFZ6 6
#define COEFZ5 5
#define COEFZ4 4
#define COEFZ3 3
#define COEFZ2 2
#define COEFZ1 1
#define COEFZ0 0


#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)


#define AKM_DATA_READY      (0x01)

#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)
/* ************************************** */

/* MPU9150 Methods Prototypes */
void init_MPU9150(unsigned char adr, unsigned char f);
/* ************************************** */

/* MPU6050 Methods Prototypes */
void set_sample_rate_divider(unsigned char);
unsigned char get_sample_rate_divider(void);

void set_DLPF_frequency(unsigned char);
unsigned char get_DLPF_frequency(void);

void set_DHPF_frequency(unsigned char);

void set_gyro_config(unsigned char);
unsigned char get_gyro_config(void);

void set_accel_config(unsigned char);
unsigned char get_accel_config(void);

void set_free_fall_accel_threshold(unsigned char);
void set_free_fall_duration(unsigned char);

void set_motion_detection_accel_threshold(unsigned char);
void set_motion_detection_duration(unsigned char);

void set_zero_motion_detection_threshold(unsigned char);
void set_zero_motion_detection_duration(unsigned char);

void set_fifo(unsigned char);
unsigned char get_fifo(void);

void set_interrupts(unsigned char);
unsigned char get_interrupts(void);

unsigned char get_interrupts_status(void);

int get_ax_raw(void); // MPU6050 au
int get_ay_raw(void); // MPU6050 av
int get_az_raw(void); // MPU6050 aw

int get_gx_raw(void); // MPU6050 omega_P
int get_gy_raw(void); // MPU6050 omega_Q
int get_gz_raw(void); // MPU6050 omega_R

int get_temp_raw(void); // MPU6050 temp

void getAccRawValue(int buffer[]);  // MPU6050 au, av, aw
void getGyroRawValue(int buffer[]); // MPU6050 omega_P, omega_Q, omega_R

void getSixRawValue(int buf[]); 

/* nb: float cf == conversion factor */
float get_ax(float cf);   // MPU6050 au [m/s^2]
float get_ay(float cf);   // MPU6050 av [m/s^2]
float get_az(float cf);   // MPU6050 aw [m/s^2]

float get_gx(float cf);   // MPU6050 omega_P [rad/s]
float get_gy(float cf);   // MPU6050 omega_Q [rad/s]
float get_gz(float cf);   // MPU6050 omega_R [rad/s] 

float get_temp(void); // MPU6050 temp [�C]

void getAccValue(float buffer[], float cf);  // MPU6050 au,av,aw [m/s^2]
void getGyroValue(float buffer[], float cf); // MPU6050 omega_P,omega_Q,omega_R [rad/s]


/*************************************/

void set_i2c_mst(unsigned char c[]);

void set_int_pin_cfg(unsigned char c);

unsigned char get_motion_detection_status(void);

void signal_path_reset(unsigned char);

void set_motion_detection_control(unsigned char);

void set_user_control(unsigned char);
unsigned char get_user_control(void);

void set_power_management_1(unsigned char);
unsigned char get_power_management_1(void);

void set_power_management_2(unsigned char);
unsigned char get_power_management_2(void);

unsigned int get_fifo_count(void);
void get_fifo_data(unsigned char d[], unsigned char);

unsigned char who_am_i(void);
/* ***************************************** */

/* AK8975 Methods Prototipes */

unsigned char wia(void);
unsigned char info(void);
unsigned char get_st_1(void);

int get_mx_raw(void); // AK8975 P
int get_my_raw(void); // AK8975 Q
int get_mz_raw(void); // AK8975 R

void getMagRawValue(int tmp[]);

float get_mx(float cf);   // AK8975 mx [G]
float get_my(float cf);   // AK8975 my [G]
float get_mz(float cf);   // AK8975 mz [G]



unsigned char get_st_2(void);

void set_cntl(unsigned char c);
unsigned char get_cntl(void);

void set_astc(unsigned char c);
unsigned char get_astc(void);

void set_i2cdis(unsigned char c);
unsigned char get_i2cdis(void);

unsigned char get_asax(void);
unsigned char get_asay(void);
unsigned char get_asaz(void);
/* ***************************************** */

/* hw offset register  */
#define XG_OFFS_USRH 0x13
#define XG_OFFS_USRL 0x14
#define YG_OFFS_USRH 0x15
#define YG_OFFS_USRL 0x16
#define ZG_OFFS_USRH 0x17
#define ZG_OFFS_USRL 0x18

#define XA_OFFS_USRH 0x06
#define XA_OFFS_USRL 0x07
#define YA_OFFS_USRH 0x08
#define YA_OFFS_USRL 0x09
#define ZA_OFFS_USRH 0x0A
#define ZA_OFFS_USRL 0x0B


void set_xg_offset(unsigned char *x);// guarda offset register map
void set_yg_offset(unsigned char *x);
void set_zg_offset(unsigned char *x);

void get_xg_offset(unsigned char x[]);
void get_yg_offset(unsigned char x[]);
void get_zg_offset(unsigned char x[]);


void set_xa_offset(unsigned char *x);
void set_ya_offset(unsigned char *x);
void set_za_offset(unsigned char *x);

void get_xa_offset(unsigned char x[]);
void get_ya_offset(unsigned char x[]);
void get_za_offset(unsigned char x[]);

/**********************************************************************/

/* other utils method */
void startAndWaitForMagData(void);
void getNineDataUnit(float *acc, float *gyr, float *mag, float cf_acc, float cf_gyr, float cf_mag);

void startNewMagMeasure(void);
void waitForMagMeasure(void);

void mpu_set_bypass(unsigned char);

void setOrientationMatrix(char m_a[], char m_g[]);


void read_offset_eeprom(void);// da usare

void set_mag_sens(unsigned char); // sensitivity adjustment hw register// non usare

void getSixDataUnits(float *acc, float *gyr, float cf_gyr);
void getMagValues(float *buffer);  // AK8975 mx,my,mz [G]

//only debug


#endif /* MPU9150_H_ */
