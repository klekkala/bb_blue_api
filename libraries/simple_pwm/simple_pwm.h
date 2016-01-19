/*
C library for interfacing with the TI PWM userspace driver
This relies on a device tree overlay enabling hrpwm 

James Strawson 2016
*/

// default frequency used if user doesn't initialize manually
#define DEFAULT_FREQ 10000

/*****************************************************************
* int simple_init_pwm(int subsystem, int frequency)
* 
* Choose from 0,1,2 for the Beaglebone's 3 pwm subsystems.
* This exports and configures both A and B channels for that
* subsystem and leaves the duty cycle at 0 (off).
*****************************************************************/
int simple_init_pwm(int subsystem, int frequency);

/*****************************************************************
* int simple_set_pwm_duty(int subsystem, char ch, float duty)
* 
* Choose from 0,1,2 for the Beaglebone's 3 pwm subsystems.
* duty is a normalized duty cycle from 0 to 1 corresponding
* to always off or always on.
*****************************************************************/
int simple_set_pwm_duty(int subsystem, char ch, float duty);

/*****************************************************************
* int simple_set_pwm_duty_ns(int subsystem, char ch, float duty_ns)
* 
* Choose from 0,1,2 for the Beaglebone's 3 pwm subsystems.
* duty_ns is the absolute width of each pulse in nanoseconds.
* This function is useful for faking servo PDM pulses.
*****************************************************************/
int simple_set_pwm_duty_ns(int subsystem, char ch, int duty_ns);

/*****************************************************************
* int simple_uninit_pwm(int subsystem)
* 
* Choose from 0,1,2 for the Beaglebone's 3 pwm subsystems.
* This unexports both A and B channels for that subsystem
* leaving them open for use by other programs.
*****************************************************************/
int simple_uninit_pwm(int subsystem);