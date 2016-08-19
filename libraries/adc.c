/**Author: Kiran Kumar Lekkala
Created: 20 June 2016
Description: Userspace API for TSC-ADC module in beaglebone black**/


#include "bb_blue_api.h"
#include "sensor_config.h"


int adc_read_raw(int ch){

	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_ADC_DIR "/in_voltage%d_raw", ch);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("in_voltage_raw");
		printf("faulty adc is %d", ch);
		return fd;
	}

	read(fd, &ch, 4);
	close(fd);
	return ch;

}


int get_adc_raw(int ch){

	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = adc_read_raw((uint8_t)ch);
	return raw_adc * 1.8 / 4095.0;
}



float get_adc_volt(int ch){

	if(ch<0 || ch>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = adc_read_raw((uint8_t)ch);
	return raw_adc * 1.8 / 4095.0;

}


/*******************************************************************************
* float get_battery_voltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float get_battery_voltage(){
	float v = (get_adc_volt(LIPO_ADC_CH)*V_DIV_RATIO)-LIPO_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}

/*******************************************************************************
* float get_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*******************************************************************************/
float get_dc_jack_voltage(){
	float v = (get_adc_volt(DC_JACK_ADC_CH)*V_DIV_RATIO)-DC_JACK_OFFSET; 
	if(v<0.3) v = 0.0;
	return v;
}
