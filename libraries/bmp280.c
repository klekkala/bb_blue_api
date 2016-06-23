/***************************************************************************
  This is a library for the BMP280 pressure sensor

 ***************************************************************************/

#include "bb_blue_api.h"
#include "sensor_config.h"
#include "useful_includes.h"

#include <stdio.h>
#include <math.h>


typedef struct bmp280_data_t{
	float temp;
	float alt;
	float pressure;
	float sea_level_pa;
}bmp280_data_t;


// one global instance of the struct
bmp280_data_t data;


/*******************************************************************************
* int initialize_barometer()
*
* Reads the factory-set coefficients
*******************************************************************************/
int initialize_barometer(bmp_oversample_t oversampling){
	return 0;
}


int power_down_barometer(){
	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	return 0;
}


/*******************************************************************************
* int read_barometer()
*
* Reads the status bit followed by the temperature and pressure data registers.
* If the status bit indicates no new data is available, this function returns 1.
* Old data will still be available with the get_pressure, get_temperature, and
* get_altitude functions.If new data was read then this function returns 0.
* If an error occurred return -1. 
*******************************************************************************/
int read_barometer(){
	int fd;
	float ch;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_BARO_DIR "/in_temp_input");

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("in_temprature_raw error");
		return fd;
	}

	read(fd, &ch, 4);
	close(fd);
	data.temp = ch;

	snprintf(buf, sizeof(buf), SYSFS_BARO_DIR "/in_pressure_input");

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("in_pressure_raw error");
		return fd;
	}

	read(fd, &ch, 4);
	close(fd);

	data.pressure = ch;
	

	data.alt = 44330.0*(1.0 - pow((data.pressure/data.sea_level_pa), 0.1903));
	return 0;
	
}



/*******************************************************************************

*******************************************************************************/
float bmp_get_temperature_c(){
	return data.temp;
}

float bmp_get_pressure_pa(){
	return data.pressure;
}

float bmp_get_altitude_m(){
	return data.alt;
}

int set_sea_level_pressure_pa(float pa){
	if(pa<80000 || pa >120000){
		printf("ERROR: Please enter a reasonable sea level pressure\n");
		printf("between 80,000 & 120,000 pascals.\n");
		return -1;
	}
	data.sea_level_pa = pa;
	return 0;
}
