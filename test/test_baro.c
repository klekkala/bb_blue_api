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
