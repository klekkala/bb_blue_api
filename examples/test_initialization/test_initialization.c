/*******************************************************************************
* test_initialization.c
*
* James Strawson 2016
* Simple check to make sure the cape library initializes without errors.
* This is called by the Auto_Run_Script.sh on boot to make sure the cape works
* before loading your project. You don't need to use this directly.
*******************************************************************************/

#include <bb_blue_api.h>

int main(){
	if(initialize_board()<0){
		printf("FAILURE: initialize_board() failed\n");
		cleanup_board();
		return -1;
	}
	else{
		printf("SUCCESS: initialize_board() worked\n");
		cleanup_board();
		return 0;
	}
}
