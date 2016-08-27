# BB Blue APIs

[![Build Status](https://travis-ci.org/kiran4399/bb_blue_api.svg?branch=master)](https://travis-ci.org/kiran4399/bb_blue_api)


This repository consists of easy­-to-­use APIs for the hardware on the ​Beaglebone Blue. These APIs will be rewritten from the original Strawson APIs which were written for the Strawson Robotics Cape which used the Cape's hardware by the userspace drivers and mmap. On the other hand, Beaglebone Blue APIs uses kernel-API approach to use the Beaglebone Blue hardware. Currently testing for the sensors with the kernel drivers is under progress.


Beaglebone Blue comes with the following features:

1. 2+2 LED and Button for easy operation
2. 9-Axis IMU: Invensense MPU-9250
3. Barometer: Bosch BMP280
4. 4 H-Bridge DC Motors Controllers 1.2A each(PWM supported)
5. I2C, SPI, UART support for external devices
6. Supports Orange and Spektrum DSM2 Satellite Receivers
7. 3 channel ADC support
8. 3 TI-eQEP Rotatory encoders + 1 PRU enabled PRU
9. 8-Channel Servo/ESC Output enabled by PRU
10. 6V 4A Regulated Power Supply to Protect Servos
11. 2 Cell Lipo Charging, Balancing, and Protection
12. 6-16V DC Input Jack to Power BBB and Charge LiPo Battery

Note: For installing the servo firmware for PRU and the kernel driver for servo control, please visit the repository: https://github.com/kiran4399/bbb_pru_firmware  

More info and detailed approach on how these sensors can be installed on the 4.4.x mainline is given in the wiki page. Following are the links for reference:

Source Code: https://github.com/kiran4399/bb_blue_api  
Wiki: https://github.com/kiran4399/bb_blue_api/wiki  
Blog: http://linuxcreatures.com/api-support-bbb  
Documentation: https://github.com/kiran4399/bb_blue_api#library-functions  
Issue Tracker: https://github.com/kiran4399/bb_blue_api/issues  
Commit History: https://github.com/kiran4399/bb_blue_api/commits/master 


##Library functions:
___

###Board API

`int initialize_board();` 

initializes beaglebone blue by loading and initializing the necessary variables.

Returns 0 on success. 


`int cleanup_board();` 

shuts down beaglebone blue by deleting and cleaning the necessary variables.

Returns 0 on success. 

___

###IMU API

`int initialize_imu(int sample_rate, signed char orientation[9]);` 

initializes the imu(mpu9250) with a sample rate and an initial 3X3 orientation matrix
@sample_rate : takes in any of the available sample-rates supported by MPU-9250 
@orientation : Initial 3X3 orientation matrix 

Returns 0 on success.  

___

`int read_accel_data(int16_t offset);`  

Always reads in latest accelerometer values. The sensor self-samples at 1khz and this retrieves the latest data.

@offset : gets the latest values ccording the offset given 

Returns the value on success.  

___

`int read_gyro_data(int16_t offset);` 

Always reads in latest gyroscope values. The sensor self-samples at 1khz and this retrieves the latest data.

@offset : gets the latest values ccording the offset given 

Returns the value on success

___

`int read_mag_data(int16_t offset);` 

Always reads in latest magnetometer values. The sensor self-samples at 1khz and this retrieves the latest data.

@offset : gets the latest values ccording the offset given 

Returns the value on success
___

`int read_imu_temp();`

Reads the temperature values given by the MPU-9250  

Returns the read temperature on success

___


`int loadGyroCalibration();`

Loads up the Gyro Calibration matrix into the driver  

Returns 0 on success

___

`int set_imu_interrupt_func(int(*func)(void));`

sets a user function to be called when new data is read

@*func : user function which is triggered when new data is read 

Returns 0 on success

___

###Baro API

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

###ADC API

`int get_adc_raw(int p)`

Read in from an analog pin with oneshot mode 

@p : Value of the analog pin(0, 1, 2, 3)  

Returns the raw voltage value on success

___

`float get_adc_volt(int p)`

gets the voltage from adc pin

@p : input the voltage channel

returns an actual voltage for an adc channel


`float get_battery_voltage()`  

returns the LiPo battery voltage on the robotics cape which accounts for the voltage divider on the cape

Returns the value of the voltage on success.  

___

`float get_dc_jack_voltage()` 

returns the DC power jack voltage on the robotics cape this accounts for the voltage divider ont he cape

Returns the value of the dc voltage on success. 

___

###LED and Button API

`int set_led(led_t led, int state)` 

Sets the state of the LED on the board

@led : Type of led which either red or green
@state : State of the led it should be in, which is 1 or 0

Returns 0 if success

___

`int get_led_state(led_t led)`

Gets the state of the given led

@p : Value of the analog pin(0, 1, 2, 3) 

Returns 0 if it is off or returns 1 if on

___

`int set_pause_pressed_func(int (*func)(void))`

Callback function to goto the given function in `func` when the pause button is pressed

@func: function which the callback function should access

Returns 0 on success

___


`int set_pause_unpressed_func(int (*func)(void))`  

Callback function to goto the given function in `func` when the pause button is unpressed

@func: function which the callback function should access

Returns 0 on success 

___

`int set_mode_pressed_func(int (*func)(void))` 

Callback function to goto the given function in `func` when the mode button is pressed

@func: function which the callback function should access

Returns 0 on success
___

`int set_mode_unpressed_func(int (*func)(void))` 

Callback function to goto the given function in `func` when the mode button is unpressed

@func: function which the callback function should access

Returns 0 on success 

___

`int get_pause_button_state()`

Returns the present state of the pause button.

Returns 0 on success

___

`int get_mode_button_state()`

Returns the present state of the pause button.

Returns 0 on success

___


`int blink_led(led_t led, float hz, float period)` 

blinks the specificed led for a frequency of `hz` and a period of `period`
@hz : Frequency of the led which should be blinked(in Hz)
@period : period of the led to which it should be blinked(in sec)

___

###PWM API

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

@motor : id of the motor which needs to be put to free-spin

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

___

###I2C API

`int i2c_init(int bus, uint8_t devAddr)`  

Initialize the I2C bus

@bus : ID of the bus used
@devAddr : Address of the device 

Returns 0 on success.  

___

`int i2c_set_device_address(int bus, uint8_t devAddr)` 

sets the device address

@bus : ID of the bus
@devAddr : Address of the device

___

`int i2c_close(int bus)` 

closes a specific i2c bus
@bus : id of the bus

___

`int i2c_claim_bus(int bus)` 

claims the i2c bus
@bus : id of the bus

___

`int i2c_release_bus(int bus)` 

Release the used I2C bus

@bus : ID of the bus

___

`int i2c_get_in_use_state(int bus)` 

Checks the used state of the bus

@bus : ID of the bus

___

`int i2c_read_bytes(int bus, uint8_t regAddr, uint8_t length, uint8_t *data)`
`int i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data)`

I2C read functions

Returns 0 on success

___

###SPI API

`int initialize_spi1(int mode, int speed_hz)`  

Functions for interfacing with SPI1 on the beaglebone and Robotics Cape

@mode : SPI mode
@speed_hz : Operating frequency 

Returns 0 on success.  

___

`int get_spi1_fd()` 

Returns the file descriptor for spi1 once initialized. Use this if you want to do your own reading and writing to the bus instead of the basic functions defined here. If the bus has not been initialized, 

Return -1

___

`int close_spi1()` 

Closes the file descriptor and sets initialized to 0.

Return 0 on success

___

`int select_spi1_slave(int slave)` 

Selects a slave by pulling the corresponding slave select pin to ground. It also ensures the other slave is not selected.

@slave: 1 or 2

Return 0 on success

___

`int deselect_spi1_slave(int slave)`

Deselects a slave (1 or 2) by pulling the corresponding slave select pin to to 3.3V.

___

`int spi1_read_bytes(char* data, int bytes)` 

Like uart_send_bytes, this lets you send any byte sequence you like.

Return 0 on success

___

`int spi1_send_bytes(char* data, int bytes)` 

Like uart_read_bytes, this lets you read a byte sequence without sending.

___

`int spi1_transfer(char* tx_data, int tx_bytes, char* rx_data)`

This is a generic wrapper for the ioctl spi transfer function. It lets the user send any sequence of bytes and read the response.

Return value is the number of bytes received or -1 on error.

___

`int spi1_write_reg_byte(char reg_addr, char data)`

Used for writing a byte value to a register. This sends in order the address and byte to be written. It also sets the MSB of the register to 1 which indicates a write operation on many ICs. If you do not want this particular functionality, use spi1_send_bytes() to send a byte string of your choosing.

Returns 0 on success

___

`char spi1_read_reg_byte(char reg_addr)` 

Reads a single character located at address reg_addr. This is accomplished by sending the reg_addr with the MSB set to 0 indicating a read on many ICs. 

@reg_addr: Address of the register

Returns the read byte

___

`int spi1_read_reg_bytes(char reg_addr, char* data, int bytes)` 

Reads multiple bytes located at address reg_addr. This is accomplished by sending the reg_addr with the MSB set to 0 indicating a read on many ICs. 

Returns 0 on successful read
___

###UART API

`int initialize_uart(int bus, int baudrate, float timeout_s)`  

Initalizes UART

@bus: needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.
@baudrate: must be one of the standard speeds in the UART spec. 115200 and 57600 are most common.
@timeout: this is in seconds and must be >=0.1

Returns -1 for failure or 0 for success  

___

`int close_uart(int bus)` 
If the bus is open and has been initialized, close it and return 0. If the bus in uninitialized, just return right away.

@bus: needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.

Return -1 if bus is out of bounds. 

___

`int get_uart_fd(int bus)` 

Returns the file descriptor for a uart bus once it has been initialized. Use this if you want to do your own reading and writing to the bus instead of the basic functions defined here. If the bus has not been initialized, 

@bus: needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.

Return -1 on success

___

`int flush_uart(int bus)` 

flushes (discards) any data received but not read. Or written but not sent.

@bus: needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.

Return -1 on success

___

`int uart_send_bytes(int bus, int bytes, char* data)` 

This is essentially a wrapper for the linux write() function with some sanity checks. Returns -1 on error, otherwise returns number of bytes sent.

@motor : id of the motor

___

`int uart_send_byte(int bus, char data)` 

This is essentially a wrapper for the linux write() function with some sanity checks. Returns -1 on error, otherwise returns number of bytes sent.

@bus: needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.

___

`int uart_read_bytes(int bus, int bytes, char* buf)`

This is a blocking function call. It will only return once the desired number of bytes has been read from the buffer or if the global flow state defined in bb_blue_api.h is set to EXITING. Due to the Sitara's UART FIFO buffer, MAX_READ_LEN (128bytes) is the largest packet that can be read with a single call to read(). For reads larger than 128bytes, we run a loop instead.

Returns 0 on success

___

`int uart_read_line(int bus, int max_bytes, char* buf)`

Function for reading a line of characters ending in '\n' newline character. This is a blocking function call. It will only return on these conditions:
* a '\n' new line character was read, this is discarded.
* max_bytes were read, this prevents overflowing a user buffer.
* timeout declared in initialize_uart() is reached
* Global flow state in robotics_cape.h is set to EXITING.

Returns 0 on success

___
