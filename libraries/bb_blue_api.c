#include "useful_includes.h"
#include "bb_blue_api.h"
#include "sensor_config.h"


enum state_t current_state = UNINITIALIZED;

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
