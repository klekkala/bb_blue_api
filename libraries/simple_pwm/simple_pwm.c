/*
Copyright (c) 2016 James Strawson

Permission is hereby granted, free of charge, to any person 
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or 
sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following 
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "simple_pwm.h"
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

//#define DEBUG
#define MAXBUF 64
#define DEFAULT_FREQ 40000 // 40khz pwm freq
#define SYSFS_PWM_DIR "/sys/class/pwm"

int duty_fd[6]; 	// pointers to duty cycle file descriptor
int period_ns[3]; 	//one period (frequency) per subsystem
char simple_pwm_initialized[3] = {0,0,0};

int simple_init_pwm(int subsystem, int frequency){
	int export_fd, len;
	char buf[MAXBUF];
	DIR* dir;
	int periodA_fd; // pointers to frequency file pointer
	int periodB_fd;
	int runA_fd;  	// run (enable) file pointers
	int runB_fd;
	int polarityA_fd;
	int polarityB_fd;
	
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	
	// unexport the channels first
	simple_uninit_pwm(subsystem);
	
	export_fd = open(SYSFS_PWM_DIR "/export", O_WRONLY);
	if (export_fd < 0) {
		printf("error opening pwm export file\n");
		return -1;
	}
	// export just the A channel for that subsystem
	len = snprintf(buf, sizeof(buf), "%d", 2*subsystem);
	write(export_fd, buf, len);
	
	//check that the right pwm directories were created
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/", 2*subsystem);
	dir = opendir(buf);
	if (dir!=NULL) closedir(dir); //success
	else{
		printf("failed to export pwm%d\n",2*subsystem);
		return -1;
	}
	
	// set up file descriptors for A channel
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/run", (2*subsystem));
	runA_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/period_ns", (2*subsystem));
	periodA_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/duty_ns", (2*subsystem));
	duty_fd[(2*subsystem)] = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/polarity", (2*subsystem)+1);
	polarityA_fd = open(buf, O_WRONLY);

	
	// disable A channel and set polarity before setting frequency
	write(runA_fd, "0", 1);
	write(duty_fd[(2*subsystem)], "0", 1); // set duty cycle to 0
	write(polarityA_fd, "0", 1); // set the polarity

	// set the period in nanoseconds
	period_ns[subsystem] = 1000000000/frequency;
	len = snprintf(buf, sizeof(buf), "%d", period_ns[subsystem]);
	write(periodA_fd, buf, len);

	
	// now we can set up the 'B' channel since the period has been set
	// the driver will not let you change the period when both are exported
	
	// export the B channel
	len = snprintf(buf, sizeof(buf), "%d", (2*subsystem)+1);
	write(export_fd, buf, len);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/", (2*subsystem)+1);
	dir = opendir(buf);
	if (dir!=NULL) closedir(dir); //success
	else{
		printf("failed to export pwm%d\n",(2*subsystem)+1);
		return -1;
	}
	// set up file descriptors for B channel
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/run", (2*subsystem)+1);
	runB_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/period_ns", (2*subsystem)+1);
	periodB_fd = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/duty_ns", (2*subsystem)+1);
	duty_fd[(2*subsystem)+1] = open(buf, O_WRONLY);
	len = snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/polarity", (2*subsystem)+1);
	polarityB_fd = open(buf, O_WRONLY);
	
	// disable the run value and set polarity before period
	write(runB_fd, "0", 1);
	write(polarityB_fd, "0", 1);
	write(duty_fd[(2*subsystem)+1], "0", 1);
	
	// set the period to match the A channel
	len = snprintf(buf, sizeof(buf), "%d", period_ns[subsystem]);
	write(periodB_fd, buf, len);
	
	// enable A&B channels
	write(runA_fd, "1", 1);
	write(runB_fd, "1", 1);
	
	// close all the files
	close(export_fd);
	close(runA_fd);
	close(runB_fd);
	close(periodA_fd);
	close(periodB_fd);
	close(polarityA_fd);
	close(polarityB_fd);
	
	// everything successful
	simple_pwm_initialized[subsystem] = 1;
	return 0;
}

int simple_uninit_pwm(int subsystem){
	int fd, len;
	char buf[MAXBUF];
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	fd = open(SYSFS_PWM_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		printf("error opening pwm export file\n");
		return -1;
	}
	len = snprintf(buf, sizeof(buf), "%d", 2*subsystem);
	write(fd, buf, len);
	len = snprintf(buf, sizeof(buf), "%d", (2*subsystem)+1);
	write(fd, buf, len);
	close(fd);
	simple_pwm_initialized[subsystem] = 0;
	return 0;
	
}


int simple_set_pwm_duty(int subsystem, char ch, float duty){
	// start with sanity checks
	if(duty>1.0 || duty<0.0){
		printf("duty must be between 0.0 & 1.0\n");
		return -1;
	}
	
	// set the duty
	int duty_ns = duty*period_ns[subsystem];
	return simple_set_pwm_duty_ns(subsystem, ch, duty_ns);
}

int simple_set_pwm_duty_ns(int subsystem, char ch, int duty_ns){
	int len;
	char buf[MAXBUF];
	// start with sanity checks
	if(subsystem<0 || subsystem>2){
		printf("PWM subsystem must be between 0 and 2\n");
		return -1;
	}
	// initialize subsystem if not already
	if(simple_pwm_initialized[subsystem]==0){
		printf("initializing PWMSS%d with default PWM frequency\n", subsystem);
		simple_init_pwm(subsystem, DEFAULT_FREQ);
	}
	// boundary check
	if(duty_ns>period_ns[subsystem] || duty_ns<0){
		printf("duty must be between 0 & period_ns\n");
		return -1;
	}
	
	// set the duty
	len = snprintf(buf, sizeof(buf), "%d", duty_ns);
	switch(ch){
	case 'A':
		write(duty_fd[(2*subsystem)], buf, len);
		break;
	case 'B':
		write(duty_fd[(2*subsystem)+1], buf, len);
		break;
	default:
		printf("pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
	
}

