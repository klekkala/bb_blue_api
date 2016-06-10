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


//// ADC Functions
/*****************************************************************
* int get_adc_raw(int p)
*
* returns the raw adc reading
*****************************************************************/
int get_adc_raw(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	return analogRead((uint8_t)p);
}

/*****************************************************************
* float get_adc_volt(int p)
* 
* returns an actual voltage for an adc channel
*****************************************************************/
float get_adc_volt(int p){
	if(p<0 || p>6){
		printf("analog pin must be in 0-6\n");
		return -1;
	}
	int raw_adc = analogRead((uint8_t)p);
	return raw_adc * 1.8 / 4095.0;
}

/*****************************************************************
* float get_battery_voltage()
* 
* returns the LiPo battery voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*****************************************************************/
float get_battery_voltage(){
	float v_adc = get_adc_volt(6);
	return v_adc*11.0; 
}

/*****************************************************************
* float get_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*****************************************************************/
float get_dc_jack_voltage(){
	float v_adc = get_adc_volt(5);
	return v_adc*11.0; 
}

/*****************************************************************
* float get_dc_jack_voltage()
* 
* returns the DC power jack voltage on the robotics cape
* this accounts for the voltage divider ont he cape
*****************************************************************/
float get_dc_jack_voltage(){
	float v_adc = get_adc_volt(5);
	return v_adc*11.0; 
}
