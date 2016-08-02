

#include "bb_blue_api.h"
#include "sensor_config.h"

state_t state = UNINITIALIZED;
//static unsigned int *prusharedMem_32int_ptr;
int pru_initialized; // set to 1 by initialize_cape, checked by cleanup_cape
void shutdown_signal_handler(int signo);
int is_cape_loaded();



/*******************************************************************************
* int initialize_board()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/
int initialize_board(){
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
			printf("warning, shutting down any existing BB Blue APIs\n");
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
	fprintf("%d",(int)current_pid, fd);
	fflush(fd);
	fclose(fd);
	
	// check the device tree overlay is actually loaded
	if (is_cape_loaded() != 1){
		printf("ERROR: Device tree overlay not loaded by cape manager\n");
		return -1;
	}
	
	// initialize mmap io libs
	printf("Initializing: ");
	printf("GPIO");
	fflush(stdout);

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
	gpio_export(INTERRUPT_PIN);
	gpio_set_dir(INTERRUPT_PIN, INPUT_PIN);
	gpio_export(SERVO_PWR);
	gpio_set_dir(SERVO_PWR, OUTPUT_PIN);
	

	printf(" eQEP");
	fflush(stdout);
	if(init_eqep(0, 0)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(1, 0)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	if(init_eqep(2, 0)){
		printf("mmap_pwmss.c failed to initialize eQEP\n");
		return -1;
	}
	
	// setup pwm driver
	printf(" PWM");
	fflush(stdout);
	if(init_pwm(1,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 0\n");
		return -1;
	}
	if(init_pwm(2,PWM_FREQ)){
		printf("simple_pwm.c failed to initialize PWMSS 1\n");
		return -1;
	}
	
	// start some gpio pins at defaults
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);
	disable_motors();
	
	//set up function pointers for button press events
	printf(" Buttons");
	fflush(stdout);
	initialize_button_handlers();
	
	// Load binary into PRU
	printf(" PRU\n");
	fflush(stdout);

/*#ifdef DEBUG  	// if in debug mode print everything
	ret=initialize_pru();
#else  // otherwise supress the annoying prints
	ret=suppress_stderr(&initialize_pru); 
#endif

	if(ret<0){
		printf("ERROR: PRU init FAILED\n");
		pru_initialized = 0;
		return -1;
	}
	pru_initialized=1;*/
		
	// Start Signal Handler
	#ifdef DEBUG
	printf("Initializing exit signal handler\n");
	#endif
	signal(SIGINT, shutdown_signal_handler);	
	signal(SIGTERM, shutdown_signal_handler);	
	
	// Print current battery voltage
	printf("Battery: %2.2fV  ", get_battery_voltage());
	printf("Process ID: %d\n", (int)current_pid);

	// all done
	set_state(PAUSED);
	printf("Robotics Cape Initialized\n\n");

	return 0;
}

/*******************************************************************************
*	int cleanup_cape()
*	shuts down library and hardware functions cleanly
*	you should call this before your main() function returns
*******************************************************************************/
int cleanup_board(){
	// just in case the user forgot, set state to exiting
	set_state(EXITING);
	
	// announce we are starting cleanup process
	printf("\nExiting Cleanly\n");
	
	//allow up to 3 seconds for thread cleanup
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 3;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_pressed_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(pause_released_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: pause_released_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_pressed_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_pressed_thread exit timeout\n");
	}
	thread_err = 0;
	thread_err = pthread_timedjoin_np(mode_released_thread, NULL, \
															&thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: mode_released_thread exit timeout\n");
	}
	
	
	#ifdef DEBUG
	printf("turning off GPIOs & PWM\n");
	#endif
	set_led(GREEN,LOW);
	set_led(RED,LOW);	
	disable_motors();
	deselect_spi1_slave(1);	
	deselect_spi1_slave(2);	
	//disable_servo_power_rail();
	
	
	#ifdef DEBUG
	printf("stopping dsm2 service\n");
	#endif
	stop_dsm2_service();
	
	/* only turn off pru if it was enbaled, otherwise segfaults
	if(pru_initialized){	
		#ifdef DEBUG
		printf("turning off PRU\n");
		#endif
		prussdrv_pru_disable(0);
		prussdrv_pru_disable(1);
		prussdrv_exit();
	}*/
	
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
	return 0;
}



/*******************************************************************************
* @ state_t get_state()
*
* returns the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
state_t get_state(){
	return state;
}


/*******************************************************************************
* @ int set_state(state_t new_state)
*
* sets the high-level robot state variable
* use this for managing how your threads start and stop
*******************************************************************************/
int set_state(state_t new_state){
	state = new_state;
	return 0;
}

/*******************************************************************************
* @ int print_state()
* 
* Prints the textual name of the state to the screen.
*******************************************************************************/
int print_state(){
	switch(state){
	case UNINITIALIZED:
		printf("UNINITIALIZED");
		break;
	case PAUSED:
		printf("PAUSED");
		break;
	case RUNNING:
		printf("RUNNING");
		break;
	case EXITING:
		printf("EXITING");
		break;
	default:
		printf("ERROR: invalid state\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* shutdown_signal_handler(int signo)
*
* catch Ctrl-C signal and change system state to EXITING
* all threads should watch for get_state()==EXITING and shut down cleanly
*******************************************************************************/
void shutdown_signal_handler(int signo){
	if (signo == SIGINT){
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}else if (signo == SIGTERM){
		set_state(EXITING);
		printf("\nreceived SIGTERM\n");
 	}
}


/*******************************************************************************
*	is_cape_loaded()
*
*	check to make sure robotics cape overlay is loaded
*	return 1 if cape is loaded
*	return -1 if cape_mgr is missing
* 	return 0 if mape_mgr is present but cape is missing
*******************************************************************************/
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


/*******************************************************************************
* @ int kill_robot()
*
* This function is used by initialize_cape to make sure any existing program
* using the robotics cape lib is stopped. The user doesn't need to integrate
* this in their own program as initialize_cape calls it. However, the user may
* call the kill_robot example program from the command line to close whatever
* program is running in the background.
*
* return values: 
* -2 : invalid contents in PID_FILE
* -1 : existing project failed to close cleanly and had to be killed
*  0 : No existing program is running
*  1 : An existing program was running but it shut down cleanly.
*******************************************************************************/
int kill_robot(){
	FILE* fd;
	int old_pid, i;
	
	// attempt to open PID file
	fd = fopen(PID_FILE, "r");
	// if the file didn't open, no proejct is runnning in the background
	// so return 0
	if (fd == NULL) {
		return 0;
	}
	
	// otherwise try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE);
		return -2;
	}
		
	// attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(access(PID_FILE, F_OK ) != -1) usleep(100000);
		else return 1; // succcess, it shut down properly
	}
	
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);

	// close and delete the old file
	fclose(fd);
	remove(PID_FILE);
	
	// return -1 indicating the program had to be killed
	return -1;
}