#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <linux/remoteproc.h>

#include "bb_blue_api.h"
#include "sensor_config.h"
#include "useful_includes.h"



/*******************************************************************************
* int initialize_pru()
* 
* Configures the PRU and loads the servo and encoder binaries. Importantly 
* it stores locally a pointer to the PRU shared memory which is used by the 
* servo and encoder functions in this C file.
*******************************************************************************/
int initialize_pru(){
	

	// start pru
    //prussdrv_init();
	
    // Open PRU Interrupt
    /*// Get the interrupt initialized
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
    prussdrv_pruintc_init(&pruss_intc_initdata);
	// get pointer to PRU shared memory
	void* sharedMem = NULL;
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    prusharedMem_32int_ptr = (unsigned int*) sharedMem;
	memset(prusharedMem_32int_ptr, 0, 9*4);
	// launch binaries
	prussdrv_exec_program(SERVO_PRU_NUM, PRU_SERVO_BIN);
	prussdrv_exec_program(ENCODER_PRU_NUM, PRU_ENCODER_BIN);*/


	int ret;

	/* let's power on and boot our remote processor */
	ret = rproc_boot(rproc_pru);
	if (ret) {
		printf("prussdrv boot failed\n");
        return -1;
	}

	/*
	 * our remote processor is now powered on... give it some work
	 */

	/* let's shut it down now */
	rproc_shutdown(rproc_pru);
	
    return 0;
}

/*******************************************************************************
* int enable_servo_power_rail()
* 
* Turns on the 6V power regulator to the servo power rail.
*******************************************************************************/
int enable_servo_power_rail(){
	return gpio_set_value(SERVO_PWR, HIGH);
}

/*******************************************************************************
* int disable_servo_power_rail()
* 
* Turns off the 6V power regulator to the servo power rail.
*******************************************************************************/
int disable_servo_power_rail(){
	return gpio_set_value(SERVO_PWR, LOW);
}

/*******************************************************************************
* int send_servo_pulse_us(int ch, int us)
* 
* Sends a single pulse of duration us (microseconds) to a single channel (ch)
* This must be called regularly (>40hz) to keep servo or ESC awake.
*******************************************************************************/
int send_servo_pulse_us(int ch, int us){
	// Sanity Checks
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	
	// PRU runs at 200Mhz. find #loops needed
	unsigned int num_loops = ((us*200.0)/PRU_SERVO_LOOP_INSTRUCTIONS); 
	// write to PRU shared memory
	
	int gpio_set_dir(int gpio, PIN_DIRECTION out_flag)
	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), "/dev/in_servo%i", gpio);
	fd = open(buf, O_WRONLY);
	//printf("%d\n", gpio);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}
	
	write(fd, &num_loops, 4);
	close(fd);
	
	return 0;
}

/*******************************************************************************
* int send_servo_pulse_us_all(int us)
* 
* Sends a single pulse of duration us (microseconds) to all channels.
* This must be called regularly (>40hz) to keep servos or ESCs awake.
*******************************************************************************/
int send_servo_pulse_us_all(int us){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_us(i, us);
	}
	return 0;
}

/*******************************************************************************
* int send_servo_pulse_normalized(int ch, float input)
* 
*
*******************************************************************************/
int send_servo_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input<-1.5 || input>1.5){
		printf("ERROR: normalized input must be between -1 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + (input*(SERVO_NORMAL_RANGE/2));
	return send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int send_servo_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int send_servo_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_servo_pulse_normalized(i, input);
	}
	return 0;
}

/*******************************************************************************
* int send_esc_pulse_normalized(int ch, float input)
* 
* 
*******************************************************************************/
int send_esc_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input<0.0 || input>1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + ((input-0.5)*SERVO_NORMAL_RANGE);
	return send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int send_esc_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int send_esc_pulse_normalized_all(float input){
	int i;
	for(i=1;i<=SERVO_CHANNELS; i++){
		send_esc_pulse_normalized(i, input);
	}
	return 0;
}
