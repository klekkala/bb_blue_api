/*
Author: Kiran Kumar Lekkala
Created: 8 June 2016
Description: Unit testing API for PRUSS kernel driver
*/

//#define DEBUG

#define _GNU_SOURCE 	// to enable macros in pthread
#include <pthread.h>    // multi-threading
#include <errno.h>		// pthread error codes

#include <bb_blue.h>

int cleanup(){
	set_state(EXITING);
	
	// allow up to 3 seconds for thread cleanup
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 3;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_pressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_unpressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_unpressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_pressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_unpressed_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_unpressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(imu_interrupt_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: imu_interrupt_thread exit timeout\n");
	}
	
	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	set_led(GREEN,LOW);
	set_led(RED,LOW);	
	disable_motors();
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	disable_servo_power_rail();
	
	#ifdef DEBUG
	printf("turning off PRU\n");
	#endif
	prussdrv_pru_disable(0);
	prussdrv_pru_disable(1);
    prussdrv_exit();
	
	#ifdef DEBUG
	printf("deleting PID file\n");
	#endif
	FILE* fd;
	// clean up the pid_file if it still exists
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	
	printf("\nExiting Cleanly\n");
	return 0;
}



int initialize_pru(){
	
	// start pru
	#ifdef DEBUG
	printf("calling prussdrv_init()\n");
	#endif
    prussdrv_init();
	
    // Open PRU Interrupt
	#ifdef DEBUG
	printf("calling prussdrv_open\n");
	#endif
    if (prussdrv_open(PRU_EVTOUT_0)){
        printf("prussdrv_open open failed\n");
        return -1;
    }

    // Get the interrupt initialized
	#ifdef DEBUG
	printf("calling prussdrv_pruintc_init\n");
	#endif
	
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
    prussdrv_pruintc_init(&pruss_intc_initdata);

	// get pointer to PRU shared memory
	void* sharedMem = NULL;
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    prusharedMem_32int_ptr = (unsigned int*) sharedMem;
	memset(prusharedMem_32int_ptr, 0, 9*4);
	
	// launch binaries
	prussdrv_exec_program(SERVO_PRU_NUM, PRU_SERVO_BIN);
	prussdrv_exec_program(ENCODER_PRU_NUM, PRU_ENCODER_BIN);

    return(0);
}