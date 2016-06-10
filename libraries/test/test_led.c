/*
Author: Kiran Kumar Lekkala
Created: 8 June 2016
Description: Unit testing API for LED.
*/

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes

#include <bb_blue.h>

/*****************************************************************
* int set_led(led_t led, int state)
* 
* turn on or off the green or red LED on robotics cape
* if state is 0, turn led off, otherwise on.
* we suggest using the names HIGH or LOW
*****************************************************************/
int set_led(led_t led, int state){
	int val;
	if(state) val = HIGH;
	else val = LOW;
	
	switch(led){
	case GREEN:
		return digitalWrite(GRN_LED, val);
		break;
	case RED:
		return digitalWrite(RED_LED, val);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		break;
	}
	return -1;
}

/*****************************************************************
* int get_led_state(led_t led)
* 
* returns the state of the green or red LED on robotics cape
* state is LOW(0), or HIGH(1)
*****************************************************************/
int get_led_state(led_t led){
	switch(led){
	case GREEN:
		return digitalRead(GRN_LED);
		break;
	case RED:
		return digitalRead(RED_LED);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		break;
	}
	return -1;
}

