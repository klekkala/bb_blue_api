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

/*****************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*****************************************************************/
int initialize_cape(){
	FILE *fd; 		
	printf("\n");

	// check if another project was using resources
	// kill that process cleanly with sigint if so
	#ifdef DEBUG
		printf("checking for existing PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "r");
	if (fd != NULL) {
		int old_pid;
		fscanf(fd,"%d", &old_pid);
		if(old_pid != 0){
			printf("warning, shutting down existing robotics project\n");
			kill((pid_t)old_pid, SIGINT);
			sleep(1);
		}
		// close and delete the old file
		fclose(fd);
		remove(PID_FILE);
	}
	
	// create new pid file with process id
	#ifdef DEBUG
		printf("opening PID_FILE\n");
	#endif
	fd = fopen(PID_FILE, "ab+");
	if (fd < 0) {
		printf("\n error opening PID_FILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	printf("Current Process ID: %d\n", (int)current_pid);
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);
	
	// check the device tree overlay is actually loaded
	if (is_cape_loaded() != 1){
		printf("ERROR: Device tree overlay not loaded by cape manager\n");
		return -1;
	}
	
	// initialize mmap io libs
	printf("Initializing GPIO\n");
	if(initialize_gpio()){
		printf("mmap_gpio_adc.c failed to initialize gpio\n");
		return -1;
	}
	//export all GPIO output pins
	gpio_export(RED_LED);
	gpio_set_dir(RED_LED, OUTPUT_PIN);
	gpio_export(GRN_LED);
	gpio_set_dir(GRN_LED, OUTPUT_PIN);
	gpio_export(MDIR1A);
	gpio_set_dir(MDIR1A, OUTPUT_PIN);
	gpio_export(MDIR1B);
	gpio_set_dir(MDIR1B, OUTPUT_PIN);
	gpio_export(MDIR2A);
	gpio_set_dir(MDIR2A, OUTPUT_PIN);
	gpio_export(MDIR2B);
	gpio_set_dir(MDIR2B, OUTPUT_PIN);
	gpio_export(MDIR3A);
	gpio_set_dir(MDIR3A, OUTPUT_PIN);
	gpio_export(MDIR3B);
	gpio_set_dir(MDIR3B, OUTPUT_PIN);
	gpio_export(MDIR4A);
	gpio_set_dir(MDIR4A, OUTPUT_PIN);
	gpio_export(MDIR4B);
	gpio_set_dir(MDIR4B, OUTPUT_PIN);
	gpio_export(MOT_STBY);
	gpio_set_dir(MOT_STBY, OUTPUT_PIN);
	gpio_export(PAIRING_PIN);
	gpio_set_dir(PAIRING_PIN, OUTPUT_PIN);
	gpio_export(SPI1_SS1_GPIO_PIN);
	gpio_set_dir(SPI1_SS1_GPIO_PIN, OUTPUT_PIN);
	gpio_export(SPI1_SS2_GPIO_PIN);
	gpio_set_dir(SPI1_SS2_GPIO_PIN, OUTPUT_PIN);
	gpio_export(INTERRUPT_PIN);
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_export(SERVO_PWR);
	gpio_set_dir(SERVO_PWR, OUTPUT_PIN);
	
	printf("Initializing ADC\n");
	if(initialize_adc()){
		printf("mmap_gpio_adc.c failed to initialize adc\n");
		return -1;
	}
	printf("Initializing eQEP\n");
	if(init_eqep(0)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(1)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(2)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	
	// setup pwm driver
	printf("Initializing PWM\n");
	if(simple_init_pwm(1,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 0\n");
		return -1;
	}
	if(simple_init_pwm(2,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 1\n");
		return -1;
	}
	
	// start some gpio pins at defaults
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);
	disable_motors();
	
	//set up function pointers for button press events
	printf("Initializing button interrupts\n");
	initialize_button_handlers();
	
	// Load binary into PRU
	printf("Initializing PRU\n");
	if(initialize_pru()){
		printf("ERROR: PRU init FAILED\n");
		return -1;
	}
	
	// Start Signal Handler
	printf("Initializing exit signal handler\n");
	signal(SIGINT, ctrl_c);	
	signal(SIGTERM, ctrl_c);	
	
	// Print current battery voltage
	printf("Battery Voltage: %2.2fV\n", get_battery_voltage());
	
	// all done
	set_state(PAUSED);
	printf("\nRobotics Cape Initialized\n");

	return 0;
}

/*****************************************************************
*	is_cape_loaded()
*
*	check to make sure robotics cape overlay is loaded
*	return 1 if cape is loaded
*	return -1 if cape_mgr is missing
* 	return 0 if mape_mgr is present but cape is missing
******************************************************************/
int is_cape_loaded(){
	int ret;
	
	// first check if the old (Wheezy) location of capemanager exists
	if(system("ls /sys/devices/ | grep -q \"bone_capemgr\"")==0){
		#ifdef DEBUG
		printf("checking /sys/devices/bone_capemgr*/slots\n");
		#endif
		ret = system("grep -q "CAPE_NAME" /sys/devices/bone_capemgr*/slots");
	}
	else if(system("ls /sys/devices/platform/ | grep -q \"bone_capemgr\"")==0){
		#ifdef DEBUG
		printf("checking /sys/devices/platform/bone_capemgr*/slots\n");
		#endif
		ret = system("grep -q "CAPE_NAME" /sys/devices/platform/bone_capemgr*/slots");
	}
	else{
		printf("Cannot find bone_capemgr*/slots\n");
		return -1;
	}
	
	if(ret == 0){
		#ifdef DEBUG
		printf("Cape Loaded\n");
		#endif
		return 1;
	} 
	
	#ifdef DEBUG
	printf("Cape NOT Loaded\n");
	printf("grep returned %d\n", ret);
	#endif
	
	return 0;
}


/******************************************************************
*	int cleanup_cape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************/
int cleanup_cape(){
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