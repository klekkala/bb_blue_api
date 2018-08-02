/*******************************************************************************
* robotics_cape_defs.h
*
* This is a colection of definitions outlining the hardware mapping of the
* robotics Cape. 
*******************************************************************************/


#ifndef ROBOTICS_CAPE_DEFS
#define ROBOTICS_CAPE_DEFS


/*******************************************************************************
* Useful Constants
*******************************************************************************/
#define DEG_TO_RAD 		0.0174532925199
#define RAD_TO_DEG 	 	57.295779513
#define PI				(float)M_PI
#define TWO_PI			(2.0f * (float)M_PI)

/*******************************************************************************
* Useful Macros
*******************************************************************************/
#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])
#define min(a, b) 	((a < b) ? a : b)


#define ON 		1
#define OFF		0

// I2C bus associations
#define IMU_BUS 	2
#define BMP_BUS 	2

// Calibration File Locations
#define CONFIG_DIRECTORY "/etc/robotics/"
#define DSM2_CAL_FILE	"dsm2.cal"
#define ACCEL_CAL_FILE 	"accel.cal"
#define GYRO_CAL_FILE 	"gyro.cal"
#define MAG_CAL_FILE	"mag.cal"

// Cape name for device tree overlay
#define CAPE_NAME 	"RoboticsCape"

// PID file location
// file created to indicate running process
// contains pid of current process
#define PID_FILE "/run/robotics_cape.pid"

//// Mavlink UDP input buffer size
#define MAV_BUF_LEN 512 

//// PRU Servo Control
#define SERVO_CHANNELS			8
// Most servos will keep moving out to 600-2400	
#define SERVO_EXTENDED_RANGE	1800
// normal range is from 900 to 2100 for 120 degree servos
#define SERVO_NORMAL_RANGE		1200 
// servo center at 1500us
#define SERVO_MID_US			1500 

#define MOTOR_CHANNELS	4
#define PWM_FREQ 25000

//// input pins
// gpio # for gpio_a.b = (32*a)+b
#define PAUSE_BTN 69 	// gpio2.5
#define MODE_BTN  68	// gpio2.4
#define IMU_INTERRUPT_PIN 117  // gpio3.21

//// gpio output pins 
#define RED_LED     66   // gpio2.2, header GP1.5
#define GRN_LED     67   // gpio2.3, header GP1.6
#define MDIR1A      60   // gpio1.28
#define MDIR1B      31   // gpio0.31
#define MDIR2A      48   // gpio1.16
#define MDIR2B      79   // gpio2.15
#define MDIR4A      70   // gpio2.6
#define MDIR4B      71   // gpio2.7
#define MDIR3B      72   // gpio2.8
#define MDIR3A      73   // gpio2.9
#define GPIO1_25    57   // gpio1.25, header GP0.3
#define GPIO1_17    49   // gpio1.17, header GP0.4
#define GPIO3_20    116  // gpio3.20, header GP3.5
#define GPIO3_17    113  // gpio3.17, header GP0.6
#define GPIO3_2     98   // gpio3.2, header GP1.3
#define GPIO3_1     97   // gpio3.1, header GP1.4
#define MOT_STBY    20   // gpio0.20
#define PAIRING_PIN 30   // gpio0.30
#define SERVO_PWR   80   // gpio2.16
#define SPI1_SS1_GPIO_PIN   113 // gpio3.17
#define SPI1_SS2_GPIO_PIN   49  // gpio1.17

// Battery Indicator LEDs
#define BATT_LED_1	27 // gpio0.27
#define BATT_LED_2	11 // gpio0.11
#define BATT_LED_3	61 // gpio1.29
#define BATT_LED_4	26 // gpio0.26

#define DC_JACK_OFFSET 0.35
#define LIPO_OFFSET 0.32
#define LIPO_ADC_CH 6
#define DC_JACK_ADC_CH  5
#define V_DIV_RATIO 11.0

#define POLL_TIMEOUT 100 /* 0.1 seconds */
#define INTERRUPT_PIN 117  // gpio3.21

#define UART4_PATH "/dev/ttyO4"

// PRU Servo & encoder Control parameters
#define SERVO_PRU_NUM 	 1
#define ENCODER_PRU_NUM 	 0
#define PRU_SERVO_BIN "/usr/bin/pru_1_servo.bin"
#define PRU_ENCODER_BIN "/usr/bin/pru_0_encoder.bin"
#define PRU_SERVO_LOOP_INSTRUCTIONS	48	// instructions per PRU servo timer loop 


// sysfs File declaration for the onboard evices
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define SYSFS_PWM_DIR "/sys/class/pwm"
#define SYSFS_IMU_DIR "/sys/bus/iio/devices/iio:device0"
#define SYSFS_BARO_DIR "/sys/bus/iio/devices/iio:device0"
#define SYSFS_ADC_DIR "/sys/bus/iio/devices/iio:device0"
#define SYSFS_EQEP_DIR "/sys/devices/ocp.*/48304000.epwmss/48304180.eqep"
#define SYSFS_SERVO_DIR "/dev/servo_drv"

#define MAX_BUF 64
#define SYSFS_OMAP_MUX_DIR "/sys/kernel/debug/omap_mux/"

typedef enum {
	INPUT_PIN,
	OUTPUT_PIN
}PIN_DIRECTION;

typedef enum {
	LOW,
	HIGH
} PIN_VALUE;

int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(int gpio, PIN_DIRECTION out_flag);
int gpio_set_value(unsigned int gpio, PIN_VALUE value);
int gpio_get_value(unsigned int gpio, int *value);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);
int gpio_omap_mux_setup(const char *omap_pin0_name, const char *mode);
#endif //ROBOTICS_CAPE_DEFS
