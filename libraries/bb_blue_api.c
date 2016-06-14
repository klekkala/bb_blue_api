/*
Description: Unit testing API for buttons and leds
*/

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "bb_blue_api.h"
#include "sensor_config.h"
#include "useful_includes.h"
#include "simple_gpio/simple_gpio.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
enum state_t state = UNINITIALIZED;
int pause_btn_state, mode_btn_state;
static unsigned int *prusharedMem_32int_ptr;
int pru_initialized; // set to 1 by initialize_cape, checked by cleanup_cape


/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();
int initialize_button_handlers();
int (*pause_released_func)();
int (*pause_pressed_func)();
int (*mode_released_func)();
int (*mode_pressed_func)();
int initialize_pru();
void shutdown_signal_handler(int signo);

/*******************************************************************************
* local thread function declarations
*******************************************************************************/
void* pause_pressed_handler(void* ptr);
void* pause_released_handler(void* ptr);
void* mode_pressed_handler(void* ptr);
void* mode_released_handler(void* ptr);

/*******************************************************************************
* local thread structs
*******************************************************************************/
pthread_t pause_pressed_thread;
pthread_t pause_released_thread;
pthread_t mode_pressed_thread;
pthread_t mode_released_thread;



/*******************************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/


int initialize_led_handlers(){
	// initialize io libs
	printf("Initializing: ");
	printf("GPIO");
	fflush(stdout);

	//export all GPIO output pins
	gpio_export(RED_LED);
	gpio_set_dir(RED_LED, OUTPUT_PIN);
	gpio_export(GRN_LED);
	gpio_set_dir(GRN_LED, OUTPUT_PIN);
}

/*******************************************************************************
* enum state_t get_state()
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
enum state_t get_state(){
	return state;
}


/*******************************************************************************
* int set_state(enum state_t new_state)
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int set_state(enum state_t new_state){
	state = new_state;
	return 0;
}




/*******************************************************************************
* blink_led()
*	
* Flash an LED at a set frequency for a finite period of time.
* This is a blocking call and only returns after flashing.
*******************************************************************************/
int blink_led(led_t led, float hz, float period){
	const int delay_us = 1000000.0/(2.0*hz); 
	const int blinks = period*2.0*hz;
	int i;
	int toggle = 0;
	
	for(i=0;i<blinks;i++){
		toggle = !toggle;
		if(get_state()==EXITING) break;
		set_led(led,toggle);
		// wait for next blink
		usleep(delay_us);
	}
	
	set_led(led, 0); // make sure it is left off
	return 0;
}

/*******************************************************************************
* @ int set_led(led_t led, int state)
* 
* turn on or off the green or red LED on robotics cape
* if state is 0, turn led off, otherwise on.
* we suggest using the names HIGH or LOW
*******************************************************************************/
int set_led(led_t led, int state){
	int val;
	if(state) val = HIGH;
	else val = LOW;
	
	switch(led){
	case GREEN:
		return gpio_set_value(GRN_LED, val);
		break;
	case RED:
		return gpio_set_value(RED_LED, val);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		break;
	}
	return -1;
}

/*******************************************************************************
* int get_led_state(led_t led)
* 
* returns the state of the green or red LED on robotics cape
* state is LOW(0), or HIGH(1)
*******************************************************************************/
int get_led_state(led_t led){
	int ret= -1;
	switch(led){
	case GREEN:
		gpio_get_value(GRN_LED, &ret);
		break;
	case RED:
		gpio_get_value(RED_LED, &ret);
		break;
	default:
		printf("LED must be GREEN or RED\n");
		ret = -1;
		break;
	}
	return ret;
}


/*******************************************************************************
*	int initialize_button_interrups()
*
*	start 4 threads to handle 4 interrupt routines for pressing and
*	releasing the two buttons.
*******************************************************************************/
int initialize_button_handlers(){
	
	#ifdef DEBUG
	printf("setting up mode & pause gpio pins\n");
	#endif
	//set up mode pi
	if(gpio_export(MODE_BTN)){
		printf("can't export gpio %d \n", MODE_BTN);
		return (-1);
	}
	gpio_set_dir(MODE_BTN, INPUT_PIN);
	gpio_set_edge(MODE_BTN, "both");  // Can be rising, falling or both
	
	//set up pause pin
	if(gpio_export(PAUSE_BTN)){
		printf("can't export gpio %d \n", PAUSE_BTN);
		return (-1);
	}
	gpio_set_dir(PAUSE_BTN, INPUT_PIN);
	gpio_set_edge(PAUSE_BTN, "both");  // Can be rising, falling or both
	
	#ifdef DEBUG
	printf("starting button handling threads\n");
	#endif
	struct sched_param params;
	pthread_attr_t attr;
	params.sched_priority = sched_get_priority_max(SCHED_FIFO)/2;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
   
	set_pause_pressed_func(&null_func);
	set_pause_released_func(&null_func);
	set_mode_pressed_func(&null_func);
	set_mode_released_func(&null_func);
	
	
	pthread_create(&pause_pressed_thread, &attr,			 \
				pause_pressed_handler, (void*) NULL);
	pthread_create(&pause_released_thread, &attr,			 \
				pause_released_handler, (void*) NULL);
	pthread_create(&mode_pressed_thread, &attr,			 \
					mode_pressed_handler, (void*) NULL);
	pthread_create(&mode_released_thread, &attr,			 \
					mode_released_handler, (void*) NULL);
	
	// apply medium priority to all threads
	pthread_setschedparam(pause_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(pause_released_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_released_thread, SCHED_FIFO, &params);
	 
	return 0;
}


/*******************************************************************************
*	void* pause_pressed_handler(void* ptr)
* 
*	wait on falling edge of pause button
*******************************************************************************/
void* pause_pressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(PAUSE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_pause_button()==PRESSED){
				pause_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}


/*******************************************************************************
* @ void* pause_released_handler(void* ptr) 
*
* wait on rising edge of pause button
*******************************************************************************/
void* pause_released_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(PAUSE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_pause_button()==RELEASED){
				pause_released_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	void* mode_pressed_handler(void* ptr) 
*	wait on falling edge of mode button
*******************************************************************************/
void* mode_pressed_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(MODE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_mode_button()==PRESSED){
				mode_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	void* mode_released_handler(void* ptr) 
*	wait on rising edge of mode button
*******************************************************************************/
void* mode_released_handler(void* ptr){
	struct pollfd fdset[1];
	char buf[MAX_BUF];
	int gpio_fd = gpio_fd_open(MODE_BTN);
	fdset[0].fd = gpio_fd;
	fdset[0].events = POLLPRI; // high-priority interrupt
	// keep running until the program closes
	while(get_state() != EXITING) {
		// system hangs here until FIFO interrupt
		poll(fdset, 1, POLL_TIMEOUT);        
		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);  
			read(fdset[0].fd, buf, MAX_BUF);
			if(get_mode_button()==RELEASED){
				mode_released_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/*******************************************************************************
*	button function assignments
*******************************************************************************/
int set_pause_pressed_func(int (*func)(void)){
	pause_pressed_func = func;
	return 0;
}
int set_pause_released_func(int (*func)(void)){
	pause_released_func = func;
	return 0;
}
int set_mode_pressed_func(int (*func)(void)){
	mode_pressed_func = func;
	return 0;
}
int set_mode_released_func(int (*func)(void)){
	mode_released_func = func;
	return 0;
}

/*******************************************************************************
*	button_state_t get_pause_button()
*******************************************************************************/
button_state_t get_pause_button(){

	int ret=-1;
	gpio_get_value(PAUSE_BTN, &ret);
	if(ret==HIGH){
		return RELEASED;
	}
	else{
		return PRESSED;
	}
}

/********************************************************************************
*	button_state_t get_mode_button()
*******************************************************************************/
button_state_t get_mode_button(){
	int ret=-1;
	gpio_get_value(MODE_BTN, &ret);
	if(ret==HIGH){
		return RELEASED;
	}
	else{
		return PRESSED;
	}
}

/*******************************************************************************
* int null_func()
* function pointers for events initialized to null_func()
* instead of containing a null pointer
*******************************************************************************/
int null_func(){
	return 0;
}