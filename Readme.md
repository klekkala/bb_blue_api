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

`int initialize_board();` 

initializes beaglebone blue by loading and initializing the necessary variables.

Returns 0 on success.     

___

`int initialize_imu(int sample_rate, signed char orientation[9]);` 

initializes the imu(mpu9250) with a sample rate and an initial 3X3 orientation matrix
@sample_rate : takes in any of the available sample-rates supported by MPU-9250 
@orientation : Initial 3X3 orientation matrix 

Returns 0 on success.  

___

`int read_accel_data(int16_t offset);`  

Send user provided sysevent to PRU INTC.  

@sysevent : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`int read_gyro_data(int16_t offset);` 

@pru_num : PRU core id ( 0 or 1 )  

Returns true if specified core is powered up. Otherwise false.  

___

`int read_mag_data(int16_t offset);` 

Sets the Gyro Offset for the MPU-9250

@offset : Offset value

___

`int read_imu_temp();`

Reads the temperature values given by the MPU-9250  

Returns the read temperature on success

___

`int set_imu_interrupt_func(int(*func)(void));`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success

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

Sets the Gyro Offset for the MPU-9250

@offset : Offset value

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

initializes the BMP280 pressure sensor. 

Returns 0 on success.  

___

`float get_pressure(void);` 


Returns the read pressure from the sensor on success.  

___

`float get_altitude(float pressure, float baseline);` 

Upon given the ground level baseline and the pressure at a specific altitude, it returns the altitude value given by the sensor.  

@pressure : pressure value at a specific altitude 
@baseline : pressure value at the ground level  

___

`int get_adc_raw(int p)`

Read in from an analog pin with oneshot mode 

@p : Value of the analog pin(0, 1, 2, 3)  

Returns the raw voltage value on success

___

`float get_adc_volt(int p)`

Shutdown PRU core.

@pru_num: PRU0 / PRU1  

Returns 0 on success


`float get_battery_voltage()`  

Get the 2-cell LiPo battery voltage value connected to the board.

Returns the value of the voltage on success.  

___

`float get_dc_jack_voltage()` 

Gets the dc voltage supplied to the jack 

Returns the value of the dc voltage on success. 

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


`int blink_led(led_t led, float hz, float period)` 

blinks the specificed led for a frequency of `hz` and a period of `period`
@hz : Frequency of the led which should be blinked(in Hz)
@period : period of the led to which it should be blinked(in sec)

___

`int set_motor(int motor, float duty)`  

set a motor direction and power motor is from 1 to 4, duty is from -1.0 to +1.0 

@motor : Sysevent number ( 0 – 63 )
@duty : Sysevent number ( 0 – 63 )  

Returns 0 on success.  

___

`int set_motor_all(float duty)` 

applies the same duty cycle argument to all 4 motors

@duty : duty cycle of the motor which needs to be set  

___

`int set_motor_free_spin(int motor)` 

This puts one or all motor outputs in high-impedance state which lets the motor spin freely as if it wasn't connected to anything.

@motor : id of teh motor which needs to be put to free-spin

___

`int set_motor_free_spin_all()` 

set_motor_free_spin to all the motors

___

`int set_motor_brake(int motor)` 

These will connect one or all motor terminal pairs together which makes the motor fight against its own back EMF turning it into a brake.

@motor : id of the motor

___

`int set_motor_brake_all()` 

sets motor brake to all the motors

___

`int enable_motors()`

turns on the standby pin to enable the h-bridge ICs

Returns 0 on success

___

`int disable_motors()`

turns off the standby pin to disable the h-bridge ICs and disables PWM output signals.

Returns 0 on success