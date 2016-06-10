/*
Author: Kiran Kumar Lekkala
Created: 8 June 2016
Description: Unit testing API for IMU kernel driver
*/

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes

#include <bb_blue.h>


/**************************************
* local function declarations
***************************************/
int is_cape_loaded();
int (*imu_interrupt_func)();
void* imu_interrupt_handler(void* ptr);

/**************************************
* local thread structs
***************************************/
pthread_t imu_interrupt_thread;





/*****************************************************************
* int initialize_imu(int sample_rate, signed char orientation[9])
* 
* set up the imu and interrupt handler
*****************************************************************/
int initialize_imu(int sample_rate, signed char orientation[9]){
	printf("Initializing IMU\n");
	//set up gpio interrupt pin connected to imu
	if(gpio_export(INTERRUPT_PIN)){
		printf("can't export gpio %d \n", INTERRUPT_PIN);
		return (-1);
	}
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_set_edge(INTERRUPT_PIN, "falling");  // Can be rising, falling or both
		
	//linux_set_i2c_bus(1);
	linux_set_i2c_bus(2);

	
	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}
 
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_sample_rate(sample_rate);
	
	// compass run at 100hz max. 
	if(sample_rate > 100){
		// best to sample at a fraction of the gyro/accel
		mpu_set_compass_sample_rate(sample_rate/2);
	}
	else{
		mpu_set_compass_sample_rate(sample_rate);
	}
	mpu_set_lpf(188); //as little filtering as possible
	dmp_load_motion_driver_firmware(sample_rate);

	dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation));
  	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO);

	dmp_set_fifo_rate(sample_rate);
	
	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}
	if(loadGyroCalibration()){
		printf("\nGyro Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_gyro example to create one\n");
		printf("Using 0 offset for now\n");
	};
	
	// set the imu_interrupt_thread to highest priority since this is used
	// for real-time control with time-sensitive IMU data
	set_imu_interrupt_func(&null_func);
	
	struct sched_param params;
	pthread_create(&imu_interrupt_thread, NULL, imu_interrupt_handler, (void*) NULL);
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(imu_interrupt_thread, SCHED_FIFO, &params);
	
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setXGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_XG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setYGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_YG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int setZGyroOffset(int16_t offset) {
	uint16_t new = offset;
	const unsigned char msb = (unsigned char)((new&0xff00)>>8);
	const unsigned char lsb = (new&0x00ff);//get LSB
	//printf("writing: 0x%x 0x%x 0x%x \n", new, msb, lsb);
	linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRH, 1, &msb);
	return linux_i2c_write(MPU_ADDR, MPU6050_RA_ZG_OFFS_USRL, 1, &lsb);
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
int loadGyroCalibration(){
	FILE *cal;
	char file_path[100];

	// construct a new file path string
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, GYRO_CAL_FILE);
	
	// open for reading
	cal = fopen(file_path, "r");
	if (cal == 0) {
		// calibration file doesn't exist yet
		return -1;
	}
	else{
		int xoffset, yoffset, zoffset;
		fscanf(cal,"%d\n%d\n%d\n", &xoffset, &yoffset, &zoffset);
		if(setXGyroOffset((int16_t)xoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setYGyroOffset((int16_t)yoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
		if(setZGyroOffset((int16_t)zoffset)){
			printf("problem setting gyro offset\n");
			return -1;
		}
	}
	fclose(cal);
	return 0;
}

// IMU interrupt stuff
// see test_imu and calibrate_gyro for examples
/*****************************************************************
* 
* 
* 
*****************************************************************/
int set_imu_interrupt_func(int (*func)(void)){
	imu_interrupt_func = func;
	return 0;
}

/*****************************************************************
* 
* 
* 
*****************************************************************/
void* imu_interrupt_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int imu_gpio_fd = gpio_fd_open(INTERRUPT_PIN);
	fdset[0].fd = imu_gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until IMU FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			// user selectable with set_inu_interrupt_func() defined above
			imu_interrupt_func(); 
		}
	}
	gpio_fd_close(imu_gpio_fd);
	return 0;
}