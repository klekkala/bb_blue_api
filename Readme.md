### BB Blue APIs

[![Build Status](https://travis-ci.org/kiran4399/bb_blue_api.svg?branch=master)](https://travis-ci.org/kiran4399/bb_blue_api)


This repository consists of easy­-to-­use APIs for the hardware on the ​Beaglebone Blue. These APIs will be rewritten from the original Strawson APIs which were written for the Strawson Robotics Cape which used the Cape's hardware by the userspace drivers and mmap. On the other hand, Beaglebone Blue APIs uses kernel-API approach to use the Beaglebone Blue hardware. Currently testing for the sensors with the kernel drivers is under progress.


##Testing using Beaglebone Black and Robotics Cape

Robotics Cape mainly uses the following sensors:

1. MPU-9150 9 axix IMU
2. BMP-180 Pressure sensor
3. TI-eQEP Rotatory encoders

More info and detailed approach on how these sensors can be installed on the 4.4 mainline is given in the wiki page.

###Library functions

`int pruss_write(unsigned const int mem_name, int wordoffset, int *data, size_t bytelength)` 

Writes buffer pointed at by 'data' pointer to PRU memory.  
@mem_name : can take  PRU_DRAM0 / PRU_DRAM1 / PRU_SHRAM as values  
@wordoffset : offset within specified memory type.  
@bytelength : length of data to be copied (in bytes).  

Returns 0 on success.  

___

`int initialize_imu(int sample_rate, signed char orientation[9]);` 

initializes the imu(mpu9250) with a sample rate and an initial 3X3 orientation matrix
@sample_rate : takes in the any of the available sample-rates supported by MPU-9250 
@orientation : Initial 3X3 orientation matrix 

Returns 0 on success.  

___

`int setXGyroOffset(int16_t offset);`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`int setYGyroOffset(int16_t offset);` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`int setZGyroOffset(int16_t offset);` 

This function can either be blocking or non-blocking depending on timeout provided by user ( in libpru.h )  

@hostevt : EVTOUT0 – EVTOUT7  
@callback : user provided callback function. Host event int is argument. No return.  
@TIMEOUT (in libpru.h): -1 for indefinite block  
						>0 wait time before releasing poll  

___

`int loadGyroCalibration();`

Boots the PRU core.  

@fwname : path to PRU firmware  
@pru_num: PRU0 / PRU1  

Returns 0 on success

___

`int set_imu_interrupt_func(int(*func)(void));`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success

___


`int initialize_baro(void);`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`float get_pressure(void);` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`float get_altitude(float pressure, float baseline);` 

This function can either be blocking or non-blocking depending on timeout provided by user ( in libpru.h )  

@hostevt : EVTOUT0 – EVTOUT7  
@callback : user provided callback function. Host event int is argument. No return.  
@TIMEOUT (in libpru.h): -1 for indefinite block  
						>0 wait time before releasing poll  

___

`int get_adc_raw(int p)`

Boots the PRU core.  

@fwname : path to PRU firmware  
@pru_num: PRU0 / PRU1  

Returns 0 on success

___

`float get_adc_volt(int p)`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success


`float get_battery_voltage()`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`float get_dc_jack_voltage()` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`int set_led(led_t led, int state)` 

This function can either be blocking or non-blocking depending on timeout provided by user ( in libpru.h )  

@hostevt : EVTOUT0 – EVTOUT7  
@callback : user provided callback function. Host event int is argument. No return.  
@TIMEOUT (in libpru.h): -1 for indefinite block  
						>0 wait time before releasing poll  

___

`int get_led_state(led_t led)`

Boots the PRU core.  

@fwname : path to PRU firmware  
@pru_num: PRU0 / PRU1  

Returns 0 on success

___

`int set_pause_pressed_func(int (*func)(void))`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success

___


`int set_pause_unpressed_func(int (*func)(void))`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`int set_mode_pressed_func(int (*func)(void))` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`int set_mode_unpressed_func(int (*func)(void))` 

This function can either be blocking or non-blocking depending on timeout provided by user ( in libpru.h )  

@hostevt : EVTOUT0 – EVTOUT7  
@callback : user provided callback function. Host event int is argument. No return.  
@TIMEOUT (in libpru.h): -1 for indefinite block  
						>0 wait time before releasing poll  

___

`int get_pause_button_state()`

Boots the PRU core.  

@fwname : path to PRU firmware  
@pru_num: PRU0 / PRU1  

Returns 0 on success

___

`int get_mode_button_state()`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success

___

`int set_motor(int motor, float duty)`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`int set_motor_all(float duty)` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`int set_motor_free_spin(int motor)` 

This puts one or all motor outputs in high-impedance state which lets the motor spin freely as if it wasn't connected to anything.
 

___

`int set_motor_free_spin_all` 

set_motor_free_spin to all the motors

___

`int set_motor_brake(int motor)` 

These will connect one or all motor terminal pairs together which makes the motor fight against its own back EMF turning it into a brake.

@motor : id of the motor

___

`int set_motor_brake_all()` 

sets motor brake to all the motors

@hostevt : EVTOUT0 – EVTOUT7  
@callback : user provided callback function. Host event int is argument. No return.  
@TIMEOUT (in libpru.h): -1 for indefinite block  
						>0 wait time before releasing poll  

___

`int enable_motors()`

turns on the standby pin to enable the h-bridge ICs

Returns 0 on success

___

`int disable_motors()`

turns off the standby pin to disable the h-bridge ICs and disables PWM output signals.

Returns 0 on success