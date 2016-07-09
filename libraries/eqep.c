/*
Copyright (c) 2014, James Strawson
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

//#define DEBUG

#include "bb_blue_api.h"
#include "sensor_config.h"
#include "useful_includes.h"
#include "tipwmss.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

volatile char *cm_per_base;
int cm_per_mapped=0;
volatile char *pwm_base[3]; // pwm subsystem pointers for eQEP
int pwmss_mapped[3] = {0,0,0}; // to record which subsystems have been mapped
int eqep_initialized[3] = {0,0,0};
int pwm_initialized[3] = {0,0,0};


/********************************************
*  eQEP
*********************************************/

// init_eqep takes care of sanity checks and returns quickly
// if nothing is to be initialized.
int init_eqep(int ss, int mode){
	// range sanity check
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}

	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), SYSFS_EQEP_DIR"/%d/mode", ss);
	fd = open(buf, O_WRONLY);

	if (fd < 0) {
		perror("writing value to the channel failed.");
		return fd;
	}

	write(fd, &mode, 4);
	close(fd);

	eqep_initialized[ss] = 1;
	return 0;
}

int is_eqep_init(int ss){
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	// see if eQEP already got initialized
	if(eqep_initialized[ss]){
		return 0;
	}
	
	else{
		return 1;
	}
}

// read a value from eQEP counter
int read_eqep(int ch){
	if(is_init_eqep(ch)) return -1;
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_EQEP_DIR"/%d/position", ch);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("counter reading error");
		return fd;
	}

	read(fd, &ch, 4);
	close(fd);
	return ch;
}

// write a value to the eQEP counter
int write_eqep(int ch, int val){
	if(is_init_eqep(ch)) return -1;
	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), SYSFS_EQEP_DIR"/%d/position", ch);
	fd = open(buf, O_WRONLY);

	if (fd < 0) {
		perror("writing value to the channel failed. May be absolute mode set?");
		return fd;
	}

	write(fd, &val, 4);

	close(fd);
	return 0;
}


/*******************************************************************************
* int get_encoder_pos(int ch)
* 
* returns the encoder counter position
*******************************************************************************/
int get_encoder_pos(int ch){
	if(ch<1 || ch>4){
		printf("Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4){
		//return (int) prusharedMem_32int_ptr[8];
	}
	
	// first 3 channels counted by eQEP
	return  read_eqep(ch-1);
}


/*******************************************************************************
* int set_encoder_pos(int ch, int val)
* 
* sets the encoder counter position
*******************************************************************************/
int set_encoder_pos(int ch, int val){
	if(ch<1 || ch>4){
		printf("Encoder Channel must be from 1 to 4\n");
		return -1;
	}
	// 4th channel is counted by the PRU not eQEP
	if(ch==4){
		//prusharedMem_32int_ptr[8] = val;
		return 0;
	}
	// else write to eQEP
	return write_eqep(ch-1, val);
}
