/*
Description: Unit testing API for buttons and leds
*/

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes

#include <bb_blue.h>

int initialize_button_handlers();

/*********************************************************************************
*	int initialize_button_interrups()
*
*	start 4 threads to handle 4 interrupt routines for pressing and
*	releasing the two buttons.
**********************************************************************************/
int initialize_button_handlers(){
	
	#ifdef DEBUG
	printf("setting up mode & pause gpio pins\n");
	#endif
	//set up mode pin
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
	set_pause_unpressed_func(&null_func);
	set_mode_pressed_func(&null_func);
	set_mode_unpressed_func(&null_func);
	
	
	pthread_create(&pause_pressed_thread, &attr,			 \
				pause_pressed_handler, (void*) NULL);
	pthread_create(&pause_unpressed_thread, &attr,			 \
				pause_unpressed_handler, (void*) NULL);
	pthread_create(&mode_pressed_thread, &attr,			 \
					mode_pressed_handler, (void*) NULL);
	pthread_create(&mode_unpressed_thread, &attr,			 \
					mode_unpressed_handler, (void*) NULL);
	
	// apply medium priority to all threads
	pthread_setschedparam(pause_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(pause_unpressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_pressed_thread, SCHED_FIFO, &params);
	pthread_setschedparam(mode_unpressed_thread, SCHED_FIFO, &params);
	 
	return 0;
}

/******************************************************************
*	void* pause_pressed_handler(void* ptr)
* 
*	wait on falling edge of pause button
*******************************************************************/
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
			if(get_pause_button_state()==PRESSED){
				pause_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/******************************************************************
*	void* pause_unpressed_handler(void* ptr) 
*
*	wait on rising edge of pause button
*******************************************************************/
void* pause_unpressed_handler(void* ptr){
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
			if(get_pause_button_state()==UNPRESSED){
				pause_unpressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/******************************************************************
*	void* mode_pressed_handler(void* ptr) 
*	wait on falling edge of mode button
*******************************************************************/
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
			if(get_mode_button_state()==PRESSED){
				mode_pressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

/******************************************************************
*	void* mode_unpressed_handler(void* ptr) 
*	wait on rising edge of mode button
*******************************************************************/
void* mode_unpressed_handler(void* ptr){
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
			if(get_mode_button_state()==UNPRESSED){
				mode_unpressed_func(); 
			}
		}
	}
	gpio_fd_close(gpio_fd);
	return 0;
}

int set_pause_pressed_func(int (*func)(void)){
	pause_pressed_func = func;
	return 0;
}
int set_pause_unpressed_func(int (*func)(void)){
	pause_unpressed_func = func;
	return 0;
}
int set_mode_pressed_func(int (*func)(void)){
	mode_pressed_func = func;
	return 0;
}
int set_mode_unpressed_func(int (*func)(void)){
	mode_unpressed_func = func;
	return 0;
}

int get_pause_button_state(){
	if(digitalRead(PAUSE_BTN)==HIGH){
		return UNPRESSED;
	}
	else{
		return PRESSED;
	}
}

int get_mode_button_state(){
	if(digitalRead(MODE_BTN)==HIGH){
		return UNPRESSED;
	}
	else{
		return PRESSED;
	}
}