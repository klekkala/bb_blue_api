/*
Description: Userspace API for board initialization
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


/*******************************************************************************
* local function declarations
*******************************************************************************/
int is_cape_loaded();


/*******************************************************************************
* int initialize_cape()
* sets up necessary hardware and software
* should be the first thing your program calls
*******************************************************************************/