/*******************************************************************************
* bare_minimum.c
*
* James Strawson 2016
* This is meant to be a skeleton program for robotics cape projects. 
*******************************************************************************/

#include <bb_blue_api.h>

int main(){
	// always initialize cape library first
	initialize_board();
	
	printf("\nHello BeagleBone\n");
	
	// Keep Running until program state changes to EXITING
	while(get_state() != EXITING){
		// handle other states
		if(get_state() == RUNNING){
			// do things
		}
		else if(get_state() == PAUSED){
			// do other things
		}
		
		// always sleep at some point in your loops to avoid locking the CPU
		usleep(100000);
	}
	
	// exit cleanly
	cleanup_board();
	return 0;
}