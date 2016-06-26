/*******************************************************************************
*Author: Kiran Kumar Lekkala
*Created: 19 June 2016 
*Description: This is a collection of API functions to control the MPU9250
*from userspace using a linux based kernel driver.
*******************************************************************************/

#include "bb_blue_api.h"
#include "useful_includes.h"
#include "sensor_config.h"
// #define DEBUG
#define WARNINGS

#define INTERRUPT_PIN 117  //gpio3.21 P9.25
#define min(a, b) 	((a < b) ? a : b)


// error threshold checks
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)


int last_read_successful;
int last_interrupt_timestamp_micros;

/*******************************************************************************
*	Local variables
*******************************************************************************/
imu_config_t config; 
int (*imu_interrupt_func)();

/*******************************************************************************
*	config functions for internal use only
*******************************************************************************/
int reset_mpu9250();
int initialize_magnetometer(imu_data_t* data);
int dmp_load_motion_driver_firmware();
int dmp_set_orientation(unsigned short orient);
//unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(signed char row[]);
int dmp_enable_lp_quat(unsigned char enable);
int dmp_enable_6x_lp_quat(unsigned char enable);
int mpu_set_sample_rate(int rate);
int dmp_set_fifo_rate(unsigned short rate);
int dmp_enable_feature(unsigned short mask);
int set_int_enable(unsigned char enable);
int dmp_set_interrupt_mode(unsigned char mode);
int read_dmp();

void* imu_interrupt_handler(void* ptr);
int (*imu_interrupt_func)(); // pointer to user-defined function


/*******************************************************************************
* imu_config_t get_default_imu_config()
*
* returns reasonable default configuration values
*******************************************************************************/
imu_config_t get_default_imu_config(){
	imu_config_t conf;
	
	conf.accel_fsr = A_FSR_4G;
	conf.gyro_fsr = G_FSR_1000DPS;
	conf.gyro_dlpf = GYRO_DLPF_184;
	conf.accel_dlpf = ACCEL_DLPF_184;
	conf.enable_magnetometer = 0;
	conf.dmp_sample_rate = 100;
	conf.orientation = ORIENTATION_Z_UP;
	
	// conf.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO) -5;
	conf.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO);
	
	return conf;
}

/*******************************************************************************
* int set_imu_config_to_defaults(*imu_config_t);
*
* resets an imu_config_t struct to default values
*******************************************************************************/
int set_imu_config_to_defaults(imu_config_t *conf){
	*conf = get_default_imu_config();
	return 0;
}

/*******************************************************************************
* int initialize_imu(imu_config_t conf)
*
* Set up the imu for one-shot sampling of sensor data by user
*******************************************************************************/
int initialize_imu(imu_data_t *data, imu_config_t conf){  
	uint8_t c;
	return 0;
}


int16_t read_raw_data(const char* directory){

	int fd;
	int16_t val;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), directory);
	fd = open(buf, O_RDONLY);

	if (fd < 0) {
		perror("error openning raw sysfs entries");
		/*Still some changs to make in the error handling*/
		return fd;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	read(fd, &val, 2);
	close(fd);

	return val;
}
/*******************************************************************************
* int read_accel_data(imu_data_t* data)
* 
* Always reads in latest accelerometer values. The sensor 
* self-samples at 1khz and this retrieves the latest data.
*******************************************************************************/
int read_accel_data(imu_data_t *data){
	// new register data stored here
	
	data->raw_accel[0] = read_raw_data(SYSFS_IMU_DIR "/in_accel_x_raw");
	data->raw_accel[1] = read_raw_data(SYSFS_IMU_DIR "/in_accel_y_raw");
	data->raw_accel[2] = read_raw_data(SYSFS_IMU_DIR "/in_accel_z_raw");

	// Fill in real unit values
	data->accel[0] = data->raw_accel[0] * data->accel_to_ms2;
	data->accel[1] = data->raw_accel[1] * data->accel_to_ms2;
	data->accel[2] = data->raw_accel[2] * data->accel_to_ms2;
	
	return 0;
}

/*******************************************************************************
* int read_gyro_data(imu_data_t* data)
*
* Always reads in latest gyroscope values. The sensor self-samples
* at 1khz and this retrieves the latest data.
*******************************************************************************/
int read_gyro_data(imu_data_t *data){

	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_gyro[0] = read_raw_data(SYSFS_IMU_DIR "/in_gyro_x_raw");
	data->raw_gyro[1] = read_raw_data(SYSFS_IMU_DIR "/in_gyro_y_raw");
	data->raw_gyro[2] = read_raw_data(SYSFS_IMU_DIR "/in_gyro_z_raw");


	// Fill in real unit values
	data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
	data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
	data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
	
	return 0;
}

/*******************************************************************************
* int read_mag_data(imu_data_t* data)
*
* Checks if there is new magnetometer data and reads it in if true.
* Magnetometer only updates at 100hz, if there is no new data then
* the values in imu_data_t struct are left alone.
*******************************************************************************/
int read_mag_data(imu_data_t* data){

	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_mag[0] = read_raw_data(SYSFS_IMU_DIR "/in_magn_x_raw");
	data->raw_mag[1] = read_raw_data(SYSFS_IMU_DIR "/in_magn_y_raw");
	data->raw_mag[2] = read_raw_data(SYSFS_IMU_DIR "/in_magn_z_raw");

	// multiply by the sensitivity adjustment and convert to
	// units of uT micro Teslas
	data->mag[0] = data->raw_mag[0] * MAG_RAW_TO_uT;
	data->mag[1] = data->raw_mag[1] * MAG_RAW_TO_uT;
	data->mag[2] = data->raw_mag[2] * MAG_RAW_TO_uT;
	
	return 0;
}

/*******************************************************************************
* int read_imu_temp(imu_data_t* data)
*
* reads the latest temperature of the imu. 
*******************************************************************************/
int read_imu_temp(imu_data_t* data){
	
	int16_t temp_val = read_raw_data(SYSFS_IMU_DIR "/in_temp_raw");;

	// convert to real units
	data->temp = ((float)(temp_val)/TEMP_SENSITIVITY) + 21.0;
	return 0;
	
}



int set_offset(const char* directory, int16_t offset){

	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), directory);
	fd = open(buf, O_WRONLY);

	if (fd < 0) {
		perror("unable to set offset");
		return fd;
	}

	write(fd, offset, 2);
	close(fd);

	return 0;
}



int write_to_disk(const char* CAL_FILE, int16_t matrix[3][3]){

	FILE *cal;
	char file_path[100];

	// construct a new file path string and open for writing
	strcpy(file_path, CONFIG_DIRECTORY);
	strcat(file_path, CAL_FILE);
	cal = fopen(file_path, "w");
	// if opening for writing failed, the directory may not exist yet
	if (cal == 0) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w");
		if (cal == 0){
			printf("could not open config directory\n");
			printf(CONFIG_DIRECTORY);
			printf("\n");
			return -1;
		}
	}
	
	// write to the file, close, and exit
	if(fprintf(cal,"%d %d %d\n%d %d %d\n%d %d %d\n", matrix[0][0], matrix[0][1], matrix[0][2],
													matrix[1][0], matrix[1][1], matrix[1][2],
													matrix[2][0], matrix[2][1], matrix[2][2])<0){

		printf("Failed to write offsets to %s\n", CAL_FILE);
		fclose(cal);
		return -1;
	}
	fclose(cal);
	return 0;	
}

/*******************************************************************************
* int write_accel_offsets_to_disk(int16_t offsets[3])
*
* Reads steady state accel offsets from the disk and puts them in the IMU's 
* accel offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int accel_offets_scale_matrix(int16_t offsets[3]){

	int16_t matrix[3][3];

	if(set_offset(SYSFS_IMU_DIR "/in_anglvel_x_calibbias", offsets[0]) != 0
		|| set_offset(SYSFS_IMU_	DIR "/in_anglvel_y_calibbias", offsets[1] != 0)
		|| set_offset(SYSFS_IMU_DIR "/in_anglvel_z_calibbias", offsets[2]) != 0){
		printf("Loading Ofsets to the sysfs entries failed");
		return -1;
	}

	if(write_to_disk(MAG_CAL_FILE, matrix) != 0){
		printf("Saving accel Caliberation file to the disk failed\n");
		return -1;
	}
	return 0;	
	
}


/*******************************************************************************
* int write_gyro_offsets_to_disk(int16_t offsets[3])
*
* Reads steady state gyro offsets from the disk and puts them in the IMU's 
* gyro offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int gyro_offsets_scale_matrix(int16_t offsets[3], float scale){
	int16_t matrix[3][3];

	if(set_offset(SYSFS_IMU_DIR "/in_anglvel_x_calibbias", offsets[0]) != 0
		|| set_offset(SYSFS_IMU_DIR "/in_anglvel_y_calibbias", offsets[1] != 0)
		|| set_offset(SYSFS_IMU_DIR "/in_anglvel_z_calibbias", offsets[2]) != 0){
		printf("Loading Ofsets to the sysfs entries failed");
		return -1;
	}

	if(write_to_disk(GYRO_CAL_FILE, matrix) != 0){
		printf("Saving gyro Caliberation file to the disk failed\n");
		return -1;
	}
	return 0;	
	
}
 

/*******************************************************************************
* int write_mag_cal_to_disk(float offsets[3], float scale[3])
*
* Reads steady state gyro offsets from the disk and puts them in the IMU's 
* gyro offset register. If no calibration file exists then make a new one.
*******************************************************************************/
int write_mag_cal_to_disk(float offsets[3], float scale){
	
	FILE *cal;
	char file_path[100];
	int ret;
	
	if(set_offset(SYSFS_IMU_DIR "/in_mag_x_calibbias", offsets[0]) != 0
		|| set_offset(SYSFS_IMU_DIR "/in_mag_y_calibbias", offsets[1] != 0)
		|| set_offset(SYSFS_IMU_DIR "/in_mag_z_calibbias", offsets[2]) != 0){
		return -1;
	}

	if(write_to_disk(MAG_CAL_FILE, matrix) != 0){
		printf("Saving mag Caliberation file to the disk failed\n");
		return -1;
	}
	return 0;	
	
}

/*******************************************************************************
* int reset_mpu9250()
*
* sets the reset bit in the power management register which restores
* the device to defualt settings. a 0.1 second wait is also included
* to let the device compelete the reset process.
*******************************************************************************/
int reset_mpu9250(){
	return 0;
}



/*******************************************************************************
* int initialize_magnetometer()
*
* configure the magnetometer for 100hz reads, also reads in the factory
* sensitivity values into the global variables;
*******************************************************************************/
int initialize_magnetometer(imu_data_t* data){
	uint8_t raw[3];  // calibration data stored here
	
	return 0;
}


/*******************************************************************************
* int dmp_load_motion_driver_firmware()
*
* loads pre-compiled firmware binary from invensense onto dmp
*******************************************************************************/



int dmp_load_motion_driver_firmware(){
	
	unsigned short ii;
    unsigned short this_write;
    return 0;
}




/*******************************************************************************
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     0 if successful.
*******************************************************************************/



int dmp_set_fifo_rate(unsigned short rate){
    unsigned short div;
    unsigned char tmp[8];

    if (rate > DMP_SAMPLE_RATE){
        return -1;
	}
	
	// set the samplerate divider
    div = 1000 / rate - 1;
    return 0;
}



/*******************************************************************************
* int dmp_enable_feature(unsigned short mask)
*
* This is mostly taken from the Invensense DMP code and serves to turn on and
* off DMP features based on the feature mask. We modified to remove some 
* irrelevant features and set our own fifo-length variable. This probably
* isn't necessary to remain in its current form as initialize_imu_dmp uses
* a fixed set of features but we keep it as is since it works fine.
*******************************************************************************/
int dmp_enable_feature(unsigned short mask){
    unsigned char tmp[10];

    /* Set integration scale factor. */
    return 0;
}


/*******************************************************************************
* int dmp_enable_6x_lp_quat(unsigned char enable)
*
* Taken straight from the Invensense DMP code. This enabled quaternion filtering
* with accelerometer and gyro filtering.
*******************************************************************************/
int dmp_enable_6x_lp_quat(unsigned char enable){
	return 0;
}

/*******************************************************************************
* int dmp_enable_lp_quat(unsigned char enable)
*
* sets the DMP to do gyro-only quaternion filtering. This is not actually used
* here but remains as a vestige of the Invensense DMP code.
*******************************************************************************/
int dmp_enable_lp_quat(unsigned char enable){
	return 0;
}


/*******************************************************************************
* int dmp_set_interrupt_mode(unsigned char mode)
* 
* This is from the Invensense open source DMP code. It configures the DMP
* to trigger an interrupt either every sample or only on gestures. Here we
* only ever configure for continuous sampling.
*******************************************************************************/
int dmp_set_interrupt_mode(unsigned char mode){
	return 0;
}

/*******************************************************************************
* int set_int_enable(unsigned char enable)
* 
* This is a vestige of the invensense mpu open source code and is probably
* not necessary but remains here anyway.
*******************************************************************************/
int set_int_enable(unsigned char enable){
    return 0;
}

/*******************************************************************************
int mpu_set_sample_rate(int rate)
Sets the clock rate divider for sensor sampling
*******************************************************************************/
int mpu_set_sample_rate(int rate){
	if(rate>1000 || rate<4){
		printf("ERROR: sample rate must be between 4 & 1000\n");
		return -1;
	}
	 /* Keep constant sample rate, FIFO rate controlled by DMP. */
	uint8_t div = (1000/rate) - 1;
	return 0;
}

/*******************************************************************************
*  int mpu_set_dmp_state(unsigned char enable)
* 
* This turns on and off the DMP interrupt and resets the FIFO. This probably
* isn't necessary as initialize_imu_dmp sets these registers but it remains 
* here as a vestige of the invensense open source dmp code.
*******************************************************************************/
int mpu_set_dmp_state(unsigned char enable){

    return 0;
}



/*******************************************************************************
* int read_dmp_fifo()
*
* Reads the FIFO buffer and populates the data struct. Here is where we see 
* bad/empty/double packets due to i2c bus errors and the IMU failing to have
* data ready in time. enabling warnings in the config struct will let this
* function print out warnings when these conditions are detected. If write
* errors are detected then this function tries some i2c transfers a second time.
*******************************************************************************/
int read_dmp_fifo(){
    return 0;
}


/*******************************************************************************
* int data_fusion()
*
* This fuses the magnetometer data with the quaternion straight from the DMP
* to correct the yaw heading to a compass heading. Much thanks to Pansenti for
* open sourcing this routine. In addition to the Pansenti implementation I also
* correct the magnetometer data for DMP orientation, initialize yaw with the
* magnetometer to prevent initial rise time, and correct the yaw_mixing_factor
* with the sample rate so the filter rise time remains constant with different
* sample rates.
*******************************************************************************/
d_filter_t low_pass, high_pass; // for magnetometer Yaw filtering

int data_fusion(){
	float fusedEuler[3], magQuat[4], unfusedQuat[4];
	static float newMagYaw = 0;
	static float newDMPYaw = 0;
	float lastDMPYaw, lastMagYaw, newYaw; 
	static float dmp_spin_counter = 0;
	static float mag_spin_counter = 0;
	static int first_run = 1; // set to 0 after first call to this function
	
	
	// start by filling in the roll/pitch components of the fused euler
	// angles from the DMP generated angles. Ignore yaw for now, we have to
	// filter that later. 
	fusedEuler[TB_PITCH_X] = data_ptr->dmp_TaitBryan[TB_PITCH_X];
	//fusedEuler[TB_ROLL_Y] = -(data_ptr->dmp_TaitBryan[TB_ROLL_Y]);
	fusedEuler[TB_ROLL_Y] = (data_ptr->dmp_TaitBryan[TB_ROLL_Y]);
	fusedEuler[TB_YAW_Z] = 0;

	// generate a quaternion rotation of just roll/pitch
	TaitBryanToQuaternion(fusedEuler, unfusedQuat);

	// create a quaternion vector from the current magnetic field vector
	// in IMU body coordinate frame. Since the DMP quaternion is aligned with
	// a particular orientation, we must be careful to orient the magnetometer
	// data to match.
	magQuat[QUAT_W] = 0;
	switch(config.orientation){
	case ORIENTATION_Z_UP:
		magQuat[QUAT_X] = data_ptr->mag[TB_PITCH_X];
		magQuat[QUAT_Y] = data_ptr->mag[TB_ROLL_Y];
		magQuat[QUAT_Z] = data_ptr->mag[TB_YAW_Z];
		break;
	case ORIENTATION_Z_DOWN:
		magQuat[QUAT_X] = -data_ptr->mag[TB_PITCH_X];
		magQuat[QUAT_Y] = data_ptr->mag[TB_ROLL_Y];
		magQuat[QUAT_Z] = -data_ptr->mag[TB_YAW_Z];
		break;
	case ORIENTATION_X_UP:
		magQuat[QUAT_X] = data_ptr->mag[TB_YAW_Z];
		magQuat[QUAT_Y] = data_ptr->mag[TB_ROLL_Y];
		magQuat[QUAT_Z] = data_ptr->mag[TB_PITCH_X];
		break;
	case ORIENTATION_X_DOWN:
		magQuat[QUAT_X] = -data_ptr->mag[TB_YAW_Z];
		magQuat[QUAT_Y] = data_ptr->mag[TB_ROLL_Y];
		magQuat[QUAT_Z] = -data_ptr->mag[TB_PITCH_X];
		break;
	case ORIENTATION_Y_UP:
		magQuat[QUAT_X] = data_ptr->mag[TB_PITCH_X];
		magQuat[QUAT_Y] = -data_ptr->mag[TB_YAW_Z];
		magQuat[QUAT_Z] = data_ptr->mag[TB_ROLL_Y];
		break;
	case ORIENTATION_Y_DOWN:
		magQuat[QUAT_X] = data_ptr->mag[TB_PITCH_X];
		magQuat[QUAT_Y] = data_ptr->mag[TB_YAW_Z];
		magQuat[QUAT_Z] = -data_ptr->mag[TB_ROLL_Y];
		break;
	default:
		printf("ERROR: invalid orientation\n");
		return -1;
	}

	// tilt that vector by the roll/pitch of the IMU to align magnetic field
	// vector such that Z points vertically
	tilt_compensate(magQuat, unfusedQuat, magQuat);

	// from the aligned magnetic field vector, find a yaw heading
	// check for validity and make sure the heading is positive
	lastMagYaw = newMagYaw; // save from last loop
	newMagYaw = -atan2f(magQuat[QUAT_Y], magQuat[QUAT_X]);
	if (newMagYaw != newMagYaw) {
		#ifdef WARNINGS
		printf("newMagYaw NAN\n");
		#endif
		return -1;
	}
	data_ptr->compass_heading = newMagYaw;
	// save DMP last from time and record newDMPYaw for this time
	lastDMPYaw = newDMPYaw;
	newDMPYaw = data_ptr->dmp_TaitBryan[TB_YAW_Z];
	
	// the outputs from atan2 and dmp are between -PI and PI.
	// for our filters to run smoothly, we can't have them jump between -PI
	// to PI when doing a complete spin. Therefore we check for a skip and 
	// increment or decrement the spin counter
	if(newMagYaw-lastMagYaw < -PI) mag_spin_counter++;
	else if (newMagYaw-lastMagYaw > PI) mag_spin_counter--;
	if(newDMPYaw-lastDMPYaw < -PI) dmp_spin_counter++;
	else if (newDMPYaw-lastDMPYaw > PI) dmp_spin_counter--;
	
	// if this is the first run, set up filters
	if(first_run){
		lastMagYaw = newMagYaw;
		lastDMPYaw = newDMPYaw;
		mag_spin_counter = 0;
		dmp_spin_counter = 0;
		
		// generate complementary filters
		float dt = 1.0/config.dmp_sample_rate;
		low_pass  = generateFirstOrderLowPass(dt,config.compass_time_constant);
		high_pass = generateFirstOrderHighPass(dt,config.compass_time_constant);
		prefill_filter_inputs(&low_pass,newMagYaw);
		prefill_filter_outputs(&low_pass,newMagYaw);
		prefill_filter_inputs(&high_pass,newDMPYaw);
		first_run = 0;
	}
	
	// new Yaw is the sum of low and high pass complementary filters.
	newYaw = march_filter(&low_pass,newMagYaw+(TWO_PI*mag_spin_counter)) \
			+ march_filter(&high_pass,newDMPYaw+(TWO_PI*dmp_spin_counter));
			
	newYaw = fmodf(newYaw,TWO_PI); // remove the effect of the spins
	if (newYaw > PI) newYaw -= TWO_PI; // bound between +- PI
	else if (newYaw < -PI) newYaw += TWO_PI; // bound between +- PI

	// Euler angles expect a yaw between -pi to pi so slide it again and
	// store in the user-accessible fused euler angle
	
	data_ptr->fused_TaitBryan[TB_YAW_Z] = newYaw;
	data_ptr->fused_TaitBryan[TB_PITCH_X] = data_ptr->dmp_TaitBryan[TB_PITCH_X];
	data_ptr->fused_TaitBryan[TB_ROLL_Y] = data_ptr->dmp_TaitBryan[TB_ROLL_Y];

	// Also generate a new quaternion from the filtered euler angles
	TaitBryanToQuaternion(data_ptr->fused_TaitBryan, data_ptr->fused_quat);
	return 0;
}

/*******************************************************************************
* unsigned short inv_row_2_scale(signed char row[])
*
* takes a single row on a rotation matrix and returns the associated scalar
* for use by inv_orientation_matrix_to_scalar.
*******************************************************************************/
unsigned short inv_row_2_scale(signed char row[]){
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

/*******************************************************************************
* unsigned short inv_orientation_matrix_to_scalar(signed char mtx[])
*
* This take in a rotation matrix and returns the corresponding 16 bit short
* which is sent to the DMP to set the orientation. This function is actually
* not used in normal operation and only served to retrieve the orientation
* scalars once to populate the imu_orientation_t enum during development.
*******************************************************************************/
unsigned short inv_orientation_matrix_to_scalar(signed char mtx[]){
    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

/*******************************************************************************
* void print_orientation_info()
*
* this function purely serves to print out orientation values and rotation
* matrices which form the imu_orientation_t enum. This is not called inside
* this C file and is not exposed to the user.
*******************************************************************************/
void print_orientation_info(){
	printf("\n");
	//char mtx[9];
	unsigned short orient;
	
	// Z-UP (identity matrix)
	signed char zup[] = {1,0,0, 0,1,0, 0,0,1};
	orient = inv_orientation_matrix_to_scalar(zup);
	printf("Z-UP: %d\n", orient);
	
	// Z-down
	signed char zdown[] = {-1,0,0, 0,1,0, 0,0,-1};
	orient = inv_orientation_matrix_to_scalar(zdown);
	printf("Z-down: %d\n", orient);
	
	// X-up
	signed char xup[] = {0,0,-1, 0,1,0, 1,0,0};
	orient = inv_orientation_matrix_to_scalar(xup);
	printf("x-up: %d\n", orient);
	
	// X-down
	signed char xdown[] = {0,0,1, 0,1,0, -1,0,0};
	orient = inv_orientation_matrix_to_scalar(xdown);
	printf("x-down: %d\n", orient);
	
	// Y-up
	signed char yup[] = {1,0,0, 0,0,-1, 0,1,0};
	orient = inv_orientation_matrix_to_scalar(yup);
	printf("y-up: %d\n", orient);
	
	// Y-down
	signed char ydown[] = {1,0,0, 0,0,1, 0,-1,0};
	orient = inv_orientation_matrix_to_scalar(ydown);
	printf("y-down: %d\n", orient);
}


/*******************************************************************************
* int was_last_read_successful()
*
* Occasionally bad data is read from the IMU, but the user's imu interrupt 
* function is always called on every interrupt to keep discrete filters
* running at a steady clock. In the event of a bad read, old data is always
* available in the user's imu_data_t struct and the user can call 
* was_last_read_successful() to see if the data was updated or not.
*******************************************************************************/
int was_last_read_successful(){
	return last_read_successful;
}

/*******************************************************************************
* uint64_t micros_since_last_interrupt()
*
* Immediately after the IMU triggers an interrupt saying new data is ready,
* a timestamp is logged in microseconds. The user's imu_interrupt_function
* will be called after all data has been read in through the I2C bus and 
* the user's imu_data_t struct has been populated. If the user wishes to see
* how long it has been since that interrupt was received they may use this
* function.
*******************************************************************************/
uint64_t micros_since_last_interrupt(){
	return micros_since_epoch() - last_interrupt_timestamp_micros;
}

imu_data_t testdata;
int main(){
	read_accel_data(testdata);
	printf("%d %d %d\n", data->accel[0], data->accel[1], data->accel[2]);
}