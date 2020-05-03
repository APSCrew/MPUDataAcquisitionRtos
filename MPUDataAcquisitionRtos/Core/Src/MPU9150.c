/*
 * MPU9150.c
 *
 * Created: 30/12/2013 10:48:07
 *  Author: Luca
 */ 

#include "MPU9150.h"


static unsigned char address;

//static char mtrx_a[9] = {0, -1, 0, -1, 0, 0, 0, 0, 1};
//static char mtrx_g[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};

short mag_sens_adj[] = {0,0,0};

//float accel_offset[] = { 0.0, 0.0, 0.0,
						 //0.0, 0.0, 0.0,
						 //0.0, 0.0, 0.0,
						 //0.0, 0.0, 0.0 };
//
//float gyro_offset[] = {0.0, 0.0, 0.0};
//
//float mag_offset[] = { 0.0, 0.0, 0.0,
					   //0.0, 0.0, 0.0 }; // hard iron offset

float accel_offset[] = { 0.000060766165247, 0.000002774776681, 0.000000455197471,
						-0.000002767003061, 0.000060750577323, 0.000000683194851,
						-0.000000769833904, 0.000000922465963, 0.000061310426426,
						-0.007089107318299, 0.000649721033477, 0.031653385017728 };
	
float mag_offset[] = {-34.574362939372641, 35.085754860355841, -58.290045354778762,
						133.3385082648147,  136.0388618066675,   130.5346621661737};
	
float gyro_offset[] = { -284.9355, -34.3422, 150.2892};

unsigned char bypass_mode = 0;

void init_MPU9150(unsigned char adr, unsigned char f){
	address = adr;
	//init_i2c(f);
	
	// Configurazione sensore
	set_power_management_1(0x01); //sleep off and clk = x gyro
	//_delay_ms(100);

	unsigned char DLPF_3 = (1<<DLPF_CFG_0)|(1<<DLPF_CFG_1); // DLPF  44Hz acc /  42Hz gyro
	set_DLPF_frequency(DLPF_3);
	unsigned char sample_rate_div_200_DLPF = 4;    // 200 Hz
	set_sample_rate_divider(sample_rate_div_200_DLPF);

	set_accel_config(0x00);
	set_gyro_config(0x00);
		
	mpu_set_bypass(1);
	
}

void set_sample_rate_divider(unsigned char srd){
	unsigned char data[] = {srd};
	write(address, SAMPLE_RATE_DIVIDER, data, 1);
}
unsigned char get_sample_rate_divider(void){
	unsigned char data[]={0};
	read(address, SAMPLE_RATE_DIVIDER, data, 1);
	return data[0];	
}

unsigned char get_DLPF_frequency(void){
	unsigned char data[]={0};
	read(address, CONFIG, data, 1);
	return data[0];	
}

void set_DLPF_frequency(unsigned char f){
	unsigned char data[] = {f};
	write(address, CONFIG, data, 1);
}
void set_DHPF_frequency(unsigned char f){
	unsigned char data[] = {f};
	write(address, ACCEL_CONFIG, data, 1);	
}

void set_gyro_config(unsigned char c){
	unsigned char data[] = {c};
	write(address, GYRO_CONFIG, data, 1);	
}

unsigned char get_gyro_config(void){
	unsigned char data[]={0};
	read(address, GYRO_CONFIG, data, 1);
	return data[0];	
}
	
void set_accel_config(unsigned char c){
	unsigned char data[] = {c};
	write(address, ACCEL_CONFIG, data, 1);	
}
unsigned char get_accel_config(void){
	unsigned char data[]={0};
	read(address, ACCEL_CONFIG, data, 1);
	return data[0];
}

void set_free_fall_accel_threshold(unsigned char t){
	unsigned char data[] = {t};
	write(address, FF_THR, data, 1);	
}
void set_free_fall_duration(unsigned char d){
	unsigned char data[] = {d};
	write(address, FF_DUR, data, 1);	
}

void set_motion_detection_accel_threshold(unsigned char t){
	unsigned char data[] = {t};
	write(address, MOT_THR, data, 1);	
}
void set_motion_detection_duration(unsigned char d){
	unsigned char data[] = {d};
	write(address, MOT_DUR, data, 1);	
}

void set_zero_motion_detection_threshold(unsigned char t){
	unsigned char data[] = {t};
	write(address, ZRMOT_THR, data, 1);	
}
void set_zero_motion_detection_duration(unsigned char d){
	unsigned char data[] = {d};
	write(address, ZRMOT_DUR, data, 1);	
}

void set_fifo(unsigned char c){
	unsigned char data[] = {c};
	write(address, FIFO_EN_REG, data, 1);	
}
unsigned char get_fifo(void){
	unsigned char data[]={0};
	read(address, FIFO_EN_REG, data, 1);
	return data[0];	
}

void set_interrupts(unsigned char c){
	unsigned char data[] = {c};
	write(address, INT_ENABLE, data, 1);
}
unsigned char get_interrupts(void){
	unsigned char data[]={0};
	read(address, INT_ENABLE, data, 1);
	return data[0];		
}
unsigned char get_interrupts_status(void){
	unsigned char data[]={0};
	read(address, INT_STATUS, data, 1);
	return data[0];	
	
}

/* all this function return a raw data*/
int get_ax_raw(void){
	unsigned char data[2] = {0};
	read(address, ACCEL_XOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;
}
int get_ay_raw(void){
	unsigned char data[2] = {0};
	read(address, ACCEL_YOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;	
}
int get_az_raw(void){
	unsigned char data[2] = {0};
	read(address, ACCEL_ZOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;	
}

int get_gx_raw(void){ // picth
	unsigned char data[2] = {0};
	read(address, GYRO_XOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;
}	
int get_gy_raw(void){ // roll
	unsigned char data[2] = {0};
	read(address, GYRO_YOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;
}	
int get_gz_raw(void){ // yaw
	unsigned char data[2] = {0};
	read(address, GYRO_ZOUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;
}	

int get_temp_raw(void){
	unsigned char data[2] = {0};
	read(address, TEMP_OUT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;	
}

void getAccRawValue(int buffer[])
{
	unsigned char data[6] = {0};
	read(address, ACCEL_XOUT_H, data, 6); // use burst read
	
    buffer[0] = (((int)data[0]) << 8) | data[1];
    buffer[1] = (((int)data[2]) << 8) | data[3];
    buffer[2] = (((int)data[4]) << 8) | data[5];
    
}

void getGyroRawValue(int buffer[])
{
	unsigned char data[6] = {0};
	read(address, GYRO_XOUT_H, data, 6); // use burst read

	buffer[0] = (((int)data[0]) << 8) | data[1]; // gx
	buffer[1] = (((int)data[2]) << 8) | data[3]; // gy
	buffer[2] = (((int)data[4]) << 8) | data[5]; // gz
		
}

float get_ax(float cf){
	int x = get_ax_raw();
	return x * cf;
}
float get_ay(float cf){
	int y = get_ay_raw();
	return y * cf;
}
float get_az(float cf){
	int z = get_az_raw();
	return z * cf;
}

float get_gx(float cf){
	int x = get_gx_raw();
	return x * cf;	
}	
float get_gy(float cf){
	int x = get_gy_raw();
	return x * cf;	
}
float get_gz(float cf){
	int x = get_gz_raw();
	return x * cf;	
}

float get_temp(void){
	int t = get_temp_raw();
	return t/340 + 35; 
}

void getAccValue(float buffer[], float cf){
	int buf[3] = {0};
	getAccRawValue(buf);
	
	buffer[0] = buf[0] * cf;
	buffer[1] = buf[1] * cf;
	buffer[2] = buf[2] * cf;
}
void getGyroValue(float buffer[], float cf){
	int buf[3] = {0};
	getGyroRawValue(buf);

	buffer[0] = buf[0] * cf;
	buffer[1] = buf[1] * cf;
	buffer[2] = buf[2] * cf;	
}

/************************************/
//da testare
unsigned char get_motion_detection_status(void){
	unsigned char data;
	read(address, MOT_DETECT_STATUS, &data, 1);
	return data;
}
//da testare
void set_int_pin_cfg(unsigned char c){
	unsigned char data = c;
	write(address, INT_PIN_CFG, &data, 1);	
}
//da testare
void signal_path_reset(unsigned char c){
	unsigned char data = c;
	write(address, SIGNAL_PATH_RESET, &data, 1);
}
//da testare
void set_motion_detection_control(unsigned char c){
	unsigned char data = c;
	write(address, MOT_DETECT_CTRL, &data, 1);
}
void set_i2c_mst(unsigned char c[]){
	write(address, I2C_MST_CNTR, c, 1);
}
//da testare
void set_user_control(unsigned char c){
	unsigned char data = c;
	write(address, USER_CTRL, &data, 1);	
}
unsigned char get_user_control(void){
	unsigned char data[] = {0};
	read(address, USER_CTRL, data, 1);
	return data[0];	
}
//da testare
void set_power_management_1(unsigned char c){ //set on-off sleep
	unsigned char data = c;
	write(address, PWR_MGMT_1, &data, 1);	
}
//da testare
unsigned char get_power_management_1(void){
	unsigned char data;
	read(address, PWR_MGMT_1, &data, 1);
	return data;	
}
//da testare
void set_power_management_2(unsigned char c){
	unsigned char data = c;
	write(address, PWR_MGMT_2, &data, 1);	
}
//da testare
unsigned char get_power_management_2(void){
	unsigned char data;
	read(address, PWR_MGMT_2, &data, 1);
	return data;
}
//da testare
unsigned int get_fifo_count(void){
	unsigned char data[2] = {0};
	read(address, FIFO_COUNT_H, data, 2); // use burst read
	
  //int      = ((MSB<<8)          | LSB);
    int temp = (int)((data[0]<<8) | data[1]);

	return temp;		
}
//da testare
//int get_fifo_data(void){
	//unsigned char data;
	//read(address, FIFO_R_W, &data, 1);
	//
	//unsigned int *int_ptr = (unsigned int*) &data;
	//unsigned int temp = *int_ptr;
//
	//return temp;	
//}

unsigned char who_am_i(void){ 
	unsigned char data;
	read(address, WHO_AM_I, &data, 1);
	return data;		
}

/*  magnetometer function */
unsigned char wia(void){
	unsigned char data;
	read(MPU9150_AK8975_ADR, WIA, &data, 1);
	return data;	
}

int get_mx_raw(void){// AK8975 mx
	unsigned char data[2] = {0};
	unsigned char drdy = (1<<DRDY);
	do
	{
		_delay_us(10);
	} while ( (get_st_1() != drdy) );

	read(MPU9150_AK8975_ADR, HXL, data, 2); // use burst read

	//int      = ((MSB<<8)     | LSB); --> little endian format!
	int temp = ((data[1]<<8) | data[0]);
	
		
	unsigned char st2 = get_st_2();
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	if (st2 != derr && st2 != hofl)
	{
		return temp;
	}
	else
	return 0; // default value
}

int get_my_raw(void){// AK8975 my
	unsigned char data[2] = {0};
	unsigned char drdy = (1<<DRDY);
	do
	{
		_delay_us(10);
	} while ( (get_st_1() != drdy) );

	read(MPU9150_AK8975_ADR, HYL, data, 2); // use burst read

	//int      = ((MSB<<8)     | LSB); --> little endian format!
	int temp = ((data[1]<<8) | data[0]);


	unsigned char st2 = get_st_2();
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	if (st2 != derr && st2 != hofl)
	{
		return temp;
	}
	else
	return 0; // default value	
}

int get_mz_raw(void){// AK8975 mz
	unsigned char data[2] = {0};
	unsigned char drdy = (1<<DRDY);
	do
	{
		_delay_us(10);
	} while ( (get_st_1() != drdy) );

	read(MPU9150_AK8975_ADR, HZL, data, 2); // use burst read

	//int      = ((MSB<<8)     | LSB); --> little endian format!
	int temp = ((data[1]<<8) | data[0]);


	unsigned char st2 = get_st_2();
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	if (st2 != derr && st2 != hofl)
	{
		return temp;
	}
	else
	return 0; // default value
}

void get_mag_3_axis_raw(int tmp[]){
	
	unsigned char data[6] = {0};

	while ( get_st_1() != (1<<DRDY) )
	{
		_delay_us(50);
	}

	unsigned char st2 = get_st_2();

	if (st2 != (1<<DERR) && st2 != (1<<HOFL))
	{
		read(MPU9150_AK8975_ADR, HXL, data, 6); // use burst read

		//int          = ((MSB<<8)     | LSB);
		tmp[0]       = ((data[1]<<8) | data[0]); // mx

		//int          = ((MSB<<8)     | LSB);
		tmp[1]       = ((data[3]<<8) | data[2]); // my

		//int          = ((MSB<<8)     | LSB);
		tmp[2]       = ((data[5]<<8) | data[4]); // mz

	}
	else
	{
		tmp[0] = 0; // default value
		tmp[1] = 0;
		tmp[2] = 0;
	}
			
}

void set_cntl(unsigned char c){
	unsigned char data = c;
	write(MPU9150_AK8975_ADR, CNTL, &data, 1);	
}
unsigned char get_cntl(void){
	unsigned char data;
	read(MPU9150_AK8975_ADR, CNTL, &data, 1);
	return data;	
}

void set_st_1(unsigned char c){
	unsigned char data = c;
	write(MPU9150_AK8975_ADR, ST1, &data, 1);
}

unsigned char get_st_1(void){
	unsigned char data;
	read(MPU9150_AK8975_ADR, ST1, &data, 1);
	return data;	
}

unsigned char get_st_2(void){
	unsigned char data;
	read(MPU9150_AK8975_ADR, ST2, &data, 1);
	return data;
}

void get_fifo_data(unsigned char d[], unsigned char v){
	read(address,FIFO_R_W,d,v); // da testare
}

void get_acc_self_test_reg(unsigned char c[]){
	read(address, SELF_TEST_X, c, 4);
}

void get_gyro_self_test_reg(unsigned char c[]){
	
	read(address, SELF_TEST_X, c, 3);
}

void set_astc(unsigned char c){
	unsigned char data = c;
	write(MPU9150_AK8975_ADR, ASTC, &data, 1);	
}
unsigned char get_astc(void){
	unsigned char data[]={0};
	read(MPU9150_AK8975_ADR, ASTC, data, 1);
	return data[0];	
}
void set_mag_sens(unsigned char c)
{
	
	if(c)
	{
		unsigned char data[4];
		
		data[0] = AKM_POWER_DOWN;
		set_cntl(data[0]);
		_delay_ms(1);

		data[0] = AKM_FUSE_ROM_ACCESS;
		set_cntl(data[0]);
		_delay_ms(1);

		/* Get sensitivity adjustment data from fuse ROM. */
		data[1] = get_asax();
		data[2] = get_asay();
		data[3] = get_asaz();
		
		mag_sens_adj[0] = (long)(((data[0] - 128)>>8)+1);
		mag_sens_adj[1] = (long)(((data[1] - 128)>>8)+1);
		mag_sens_adj[2] = (long)(((data[2] - 128)>>8)+1);		

		data[0] = AKM_POWER_DOWN;
		set_cntl(data[0]);
		_delay_ms(1);
			
	}
	else
	{
		mag_sens_adj[0] = 1;
		mag_sens_adj[1] = 1;
		mag_sens_adj[2] = 1;
	}
}	

unsigned char get_asax(void){
	unsigned char c[] = {0};
	read(MPU9150_AK8975_ADR,ASAX,c,1);
	return c[0];
}
unsigned char get_asay(void){
	unsigned char c[] = {0};
	read(MPU9150_AK8975_ADR,ASAY,c,1);
	return c[0];
}
unsigned char get_asaz(void){
	unsigned char c[] = {0};
	read(MPU9150_AK8975_ADR,ASAZ,c,1);
	return c[0];
}
void getMagRawValue(int data[])
{
	
	////unsigned char tmp[3];
////
	////tmp[0] = get_st_1();
	////_delay_ms(1);
	////tmp[1] = get_st_2();
	////_delay_ms(1);
	//////tmp[2] = AKM_SINGLE_MEASUREMENT;
	//////set_cntl(tmp[2]);
	//////_delay_ms(10);
	//
	//
	//data[0] = get_mx_raw();//(tmp[2] << 8) | tmp[1];
	//data[1] = get_my_raw();//(tmp[4] << 8) | tmp[3];
	//data[2] = get_mz_raw();//(tmp[6] << 8) | tmp[5];
	//
	/////* AK8975 doesn't have the overrun error bit. */
	////if (!(tmp[0] & AKM_DATA_READY))
		////return -2;
	////if ((tmp[1] & AKM_OVERFLOW) || (tmp[1] & AKM_DATA_ERROR))
		////return -3;
	//
	//data[0] = ((long)data[0] * mag_sens_adj[0]);
	//data[1] = ((long)data[1] * mag_sens_adj[1]);
	//data[2] = ((long)data[2] * mag_sens_adj[2]);		
	
	//unsigned char magdata[6] = {0};

	//read(MPU9150_AK8975_ADR, HXL, magdata, 6); // use burst read
	//int      = ((MSB<<8)     | LSB); --> little endian format!
	//data[0] = ( ((magdata[1]<<8) | magdata[0]) * mag_sens_adj[0] );
	//data[1] = ( ((magdata[3]<<8) | magdata[2]) * mag_sens_adj[1] );
	//data[2] = ( ((magdata[5]<<8) | magdata[4]) * mag_sens_adj[2] );
	
	//data[0] = ( ((magdata[1]<<8) | magdata[0]) );
	//data[1] = ( ((magdata[3]<<8) | magdata[2]) );
	//data[2] = ( ((magdata[5]<<8) | magdata[4]) );
	
	
	unsigned char magdata[7] = {0};
	unsigned char st2 = 0;
	
	read(MPU9150_AK8975_ADR, HXL, magdata, 7); // use burst read
	//int      = ((MSB<<8)     | LSB); --> little endian format!
	st2 = magdata[6];
	
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	
	//int mx, my, mz = 0;
	
	if (st2 != derr && st2 != hofl)
	{
		data[0] = ( ((magdata[1]<<8) | magdata[0]) );
		data[1] = ( ((magdata[3]<<8) | magdata[2]) );
		data[2] = ( ((magdata[5]<<8) | magdata[4]) );
	}
	else
	{
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
	}		
	
	//data[0] = my;
	//data[1] = mx;
	//data[2] = mz;
	
}

float get_mx(float cf){
	int x = get_mx_raw();
	return x*cf;
}
float get_my(float cf){
	int x = get_my_raw();
	return x*cf;	
}

float get_mz(float cf){
	int x = get_mz_raw();
	return x*cf;
}

void getMagValues(float *buffer){
	
	unsigned char magdata[7] = {0};
	unsigned char st2 = 0;
	
	read(MPU9150_AK8975_ADR, HXL, magdata, 7); // use burst read
	//int      = ((MSB<<8)     | LSB); --> little endian format!
	st2 = magdata[6];
	
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	
	int mx, my, mz = 0;
	
	if (st2 != derr && st2 != hofl)
	{
		mx = ( ((magdata[1]<<8) | magdata[0]) );
		my = ( ((magdata[3]<<8) | magdata[2]) );
		mz = ( ((magdata[5]<<8) | magdata[4]) );
	}
	else
	{
		mx = 0;
		my = 0;
		mz = 0;
	}
	
	// apply calibration, see LSM303DLH-compass-app-note
	buffer[0] = (float) ( ((mx - mag_offset[0])/mag_offset[3]) ); // in [gauss]
	buffer[1] = (float) ( ((my - mag_offset[1])/mag_offset[4]) ); // in [gauss]
	buffer[2] = (float) ( ((mz - mag_offset[2])/mag_offset[5]) ); // in [gauss]
}


void set_xg_offset(unsigned char *x)
{
	write(address, XG_OFFS_USRH, x, 2);
}

void set_yg_offset(unsigned char *x)
{
	write(address, YG_OFFS_USRH, x, 2);
}

void set_zg_offset(unsigned char *x)
{
	write(address, ZG_OFFS_USRH, x, 2);
}

void set_xa_offset(unsigned char *x)
{
	write(address, XA_OFFS_USRH, x, 2);
}

void set_ya_offset(unsigned char *x)
{
	write(address, YA_OFFS_USRH, x, 2);
}

void set_za_offset(unsigned char *x)
{
	write(address, ZA_OFFS_USRH, x, 2);
}

void get_xa_offset(unsigned char data[]){
	read(address, XA_OFFS_USRH, data, 2);
}

void get_ya_offset(unsigned char data[]){
	read(address, YA_OFFS_USRH, data, 2);
}

void get_za_offset(unsigned char data[]){
	read(address, ZA_OFFS_USRH, data, 2);
}

void get_xg_offset(unsigned char data[]){
	read(address, XG_OFFS_USRH, data, 2);
}

void get_yg_offset(unsigned char data[]){
	read(address, YG_OFFS_USRH, data, 2);
}

void get_zg_offset(unsigned char data[]){
	read(address, ZG_OFFS_USRH, data, 2);
}

void mpu_set_bypass(unsigned char bypass_on)
{
	unsigned char tmp;
	
	if (bypass_mode == bypass_on)
	return;
	
	if (bypass_on) {
		tmp = 0x02; //(1<<I2C_BYPASS_EN)
		set_int_pin_cfg(tmp);
		} else {
		tmp = 0;
		set_int_pin_cfg(tmp);
	}
	
	bypass_mode = bypass_on;
}

void startAndWaitForMagData(void)
{
	// start new magnetometer measure
	unsigned char start_new_meas = (1<<MODE_0); // Single measurement mode
	write(MPU9150_AK8975_ADR, CNTL, &start_new_meas, 1);
	
	////wait until the mag data is ready
	unsigned char drdy = (1<<DRDY);
	unsigned char st1 = 0;
	do
	{
		read(MPU9150_AK8975_ADR, ST1, &st1, 1); // use burst read
		_delay_us(100);
	} while (st1 != drdy);
}

void getNineDataUnit(float *acc, float *gyr, float *mag, float cf_acc, float cf_gyr, float cf_mag)
{
	unsigned char data[14] = {0};
	read(address, ACCEL_XOUT_H, data, 12); // use burst read
	
	int buffer[9] = {0};
	
	buffer[0] = (((int)data[0]) << 8) | data[1]; // ax
	buffer[1] = (((int)data[2]) << 8) | data[3]; // ay
	buffer[2] = (((int)data[4]) << 8) | data[5]; // az
	
	//temp = (((int)data[6]) << 8) | data[7]; // temperature not used!
	
	buffer[3] = (((int)data[8]) << 8) | data[9];   // gx
	buffer[4] = (((int)data[10]) << 8) | data[11]; // gy
	buffer[5] = (((int)data[12]) << 8) | data[13]; // gz
	
	unsigned char magdata[7] = {0};
	unsigned char st2 = 0;
	
	read(MPU9150_AK8975_ADR, HXL, magdata, 7); // use burst read
	//int      = ((MSB<<8)     | LSB); --> little endian format!
	buffer[6] = ((magdata[1]<<8) | magdata[0]);
	buffer[7] = ((magdata[3]<<8) | magdata[2]);
	buffer[8] = ((magdata[5]<<8) | magdata[4]);
	st2 = magdata[6];
	
	unsigned char derr = (1<<DERR);
	unsigned char hofl = (1<<HOFL);
	if (st2 != derr && st2 != hofl)
	{
		mag[0] = (float) (buffer[6]*cf_mag);
		mag[1] = (float) (buffer[7]*cf_mag);
		mag[2] = (float) (buffer[8]*cf_mag);
	}
	else
	{
		mag[0] = 0.0;
		mag[1] = 0.0;
		mag[2] = 0.0;
	}
	
	acc[0] = (float) (buffer[0]*cf_acc);
	acc[1] = (float) (buffer[1]*cf_acc);
	acc[2] = (float) (buffer[2]*cf_acc);
	
	gyr[0] = (float) (buffer[3]*cf_gyr);
	gyr[1] = (float) (buffer[4]*cf_gyr);
	gyr[2] = (float) (buffer[5]*cf_gyr);
	
}

void getSixDataUnits(float *acc, float *gyr, float cf_gyr)
{
	unsigned char data[14] = {0};
	read(address, ACCEL_XOUT_H, data, 14); // use burst read
	
	int buffer[6] = {0};
	int ax, ay, az, gx, gy ,gz = 0;
	
	ax = (((int)data[0]) << 8) | data[1]; // ax raw
	ay = (((int)data[2]) << 8) | data[3]; // ay raw
	az = (((int)data[4]) << 8) | data[5]; // az raw
	
	//temp = (((int)data[6]) << 8) | data[7]; // temperature not used!
	
	gx = (((int)data[8]) << 8)  | data[9];  // gx raw
	gy = (((int)data[10]) << 8) | data[11]; // gy raw
	gz = (((int)data[12]) << 8) | data[13]; // gz raw
	
	buffer[0] = (int)(-ay);
	buffer[1] = (int)(-ax);
	buffer[2] = (int)(az);
	
	buffer[3] = (int)(gy);
	buffer[4] = (int)(gx);
	buffer[5] = (int)(-gz);
	
	
	// apply calibration, see LSM303DLH-compass-app-note
	acc[0] = (float) (accel_offset[9]  + (accel_offset[0]*buffer[0] + accel_offset[1]*buffer[1] + accel_offset[2]*buffer[2])); // ax calibrated in [G]
	acc[1] = (float) (accel_offset[10] + (accel_offset[3]*buffer[0] + accel_offset[4]*buffer[1] + accel_offset[5]*buffer[2])); // ax calibrated in [G]
	acc[2] = (float) (accel_offset[11] + (accel_offset[6]*buffer[0] + accel_offset[7]*buffer[1] + accel_offset[8]*buffer[2])); // ax calibrated in [G]
	
	// only offset remove, gyr = (gyr_raw - mean(g_raw_ss))*cf_gyr;
	gyr[0] = (float) ((buffer[3] - gyro_offset[0])*cf_gyr); // gx calibrated in [rad/sec]
	gyr[1] = (float) ((buffer[4] - gyro_offset[1])*cf_gyr); // gy calibrated in [rad/sec]
	gyr[2] = (float) ((buffer[5] - gyro_offset[2])*cf_gyr); // gz calibrated in [rad/sec]
	
}

void getSixRawValue(int buf[])
{
	unsigned char data[14] = {0};
	read(address,ACCEL_XOUT_H,data,14);
	
	int ax, ay, az, gx, gy, gz = 0;
	
    ax = (((int)data[0]) << 8) | data[1];
    ay = (((int)data[2]) << 8) | data[3];
    az = (((int)data[4]) << 8) | data[5];
	
    gx = (((int)data[8]) << 8) | data[9];
    gy = (((int)data[10]) << 8) | data[11];
    gz = (((int)data[12]) << 8) | data[13];
	
	
	//buf[0] = (int)(mtrx_a[0]*ax + mtrx_a[1]*ay + mtrx_a[2]*az);
	//buf[1] = (int)(mtrx_a[3]*ax + mtrx_a[4]*ay + mtrx_a[5]*az);
	//buf[2] = (int)(mtrx_a[6]*ax + mtrx_a[7]*ay + mtrx_a[8]*az);
	//
	//buf[3] = (int)(mtrx_g[0]*gx + mtrx_g[1]*gy + mtrx_g[2]*gz);
	//buf[4] = (int)(mtrx_g[3]*gx + mtrx_g[4]*gy + mtrx_g[5]*gz);
	//buf[5] = (int)(mtrx_g[6]*gx + mtrx_g[7]*gy + mtrx_g[8]*gz);
	
	buf[0] = (int)(-ay);
	buf[1] = (int)(-ax);
	buf[2] = (int)(az);
	
	buf[3] = (int)(gy);
	buf[4] = (int)(gx);
	buf[5] = (int)(-gz);		
		
}

void startNewMagMeasure(void)
{
	unsigned char start_new_meas = (1<<MODE_0); // Single measurement mode
	write(MPU9150_AK8975_ADR, CNTL, &start_new_meas, 1);
}

void waitForMagMeasure(void)
{
	////wait until the mag data is ready
	unsigned char drdy = (1<<DRDY);
	unsigned char st1 = 0;
	do
	{
		read(MPU9150_AK8975_ADR, ST1, &st1, 1);
		_delay_us(10);
	} while (st1 != drdy);
}

void setOrientationMatrix(char m_a[], char m_g[])
{
	//mtrx_a = &m_a;
	//mtrx_g = &m_g;
}

//void read_offset_eeprom(void)
//{
	//eeprom_read_block(accel_offset, 0, 36);
	//eeprom_write_block(mag_offset, 37, 24);
	//eeprom_write_block(gyro_offset, 60, 12);
//}
