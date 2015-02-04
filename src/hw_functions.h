#ifndef HW_FUNCTIONS
#define HW_FUNCTIONS
#include "Arduino.h"
#include "PID_v1.h"
#include "hw_resources.h"

/*
	Accelerometer ADXL345 for orientation measurement
*/

#define  ADXL345_ADDRESS (0xA6 >> 1)
/* 
	There are 6 data registers, they are sequential starting 
	with the LSB of X axis.  All 6 data are read in a burst 
*/
#define ADXL345_REGISTER_XLSB (0x32)

/* 	Set power control bit to wake up the adxl345 */
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)

/* 	ADXL345 measures accerlations on X, Y and Z axes */
#define ADXL_N_ACCEL_AXES	3
/* 	Each of the axes needs two bytes to store data, therefore in total 6 bytes are needed */
#define ADXL_N_ACCEL_DATA_BYTES		ADXL_N_ACCEL_AXES*2

/*
	Global Variables:
	Some variables are defined as global since they are used in different functions and files. 

	The following global variables are used in hw_functions.cpp.
	The ones declared as "extern" are shared between files.
*/
static int 	MA_cnt;				/*A counter for counting moving average*/
static char	MA_filter_buf1[MA_MAX_ACCEL];	/*Moving average buffer 1*/
static char 	MA_filter_buf2[MA_MAX_ACCEL];	/*Moving average buffer 2*/

/* 	
	Q: Using moving average buffers debounces the measured data and reduces noise, what is 
	the disadvantage? 
*/
extern int 	flag_break;			/*Flag for collision avoidance*/
extern int 	flag_timer2_interrupt;		/*Flag to indicate a timer2 interrupt*/

/*
	Timer2 interrrupt counter
	Using an interrupt counter allows to call functions in different rate
	e.g. every 2 interrupts call function A, and every 5 interrupts call funcB.
	TIMER2_INT_CNT_MAX is used here as a counting up limit.
*/
extern int 	Timer2IntrCounter;


/*
	Timer 2
	Timer2Handler(): React when a timer2 interrupt is detected. 

	Q: Usually this kind of functions are very short (a few lines), and normally it only set
	and/or clear flags. Why? 
*/
void Timer2Handler();

/* 
	ADXL345 Functions:
	I2C protocol is needed to configure the sensor and read the measured data.
	http://en.wikipedia.org/wiki/I2C
*/
void 	i2c_write(int address, byte reg, byte data);
void 	i2c_read(int address, byte reg, int count, byte* data);
void 	init_adxl345();	/* The chip must be initialized before reading. */
void 	read_adxl345(int accel_data[ADXL_N_ACCEL_AXES]);

/*
	Ultrasonic sensor: HC-SR04
	To start measurement, Trig of SR04 must receive a pulse of high (5V) for at least 10us,
	this will initiate the sensor will transmit out 8 cycle of ultrasonic burst at 40kHz 
	and wait for the reflected ultrasonic burst. When the sensor detected ultrasonic from 
	receiver, it will set the Echo pin to high (5V) and delay for a period (width) which 
	proportion to distance. To obtain the distance, measure the width (Ton) of Echo pin. 
	
	Doc: https://docs.google.com/document/d/1Y-yZnNhMYy7rwhAgyL_pfa39RsB-x2qR4vP8saG73rE/edit?pli=1
*/
int 	ultrasonic_distance();

/*
	A photoresistor is a light-controlled variable sensor.
	http://en.wikipedia.org/wiki/Photoresistor
	
	In this project, the photoresistor triggers the LED in front of the car, which demonstrates
	an automatic light-on system for dark environment. If the light intensity is lower than a 
	pre-defined threshold, turn the LED on.
*/
int 	photoresistor_val(int pin);
void 	light_on(int light_intensity);

/*
	Simple object detection and collision avoidance:
	If an object appears in front of the car (flag_break==1), it should ignore the "moving forward" command.
*/
int 	obj_detection(int distance);

/*
	Motor driving functions.
	Code source: http://playground.arduino.cc/Main/AdafruitMotorShield
	Part of the instructions are in the comments in the implementation (hw_functions.cpp)
*/
void 	motor_drive(int nMotor, int command, int speed);
void 	PWM_output(int output, int high_low, int speed);
void 	shiftWrite(int output, int high_low);


/*
	Pan-tilt 2DOF gadget.
	The pan-tilt gadget consists of two servo motors. The goal is to keep the gadget always "balanced", which
	means, parallel to the ground, so that the object (a ball) could be held and does not fall when the car
	is moving despite road condition.

	The input to this balancing function is the measured data from the accelerometer.
	Q: What is the best way to use the data? 

	The output of the function are the positions of the servo motors. PID controllers are used in this function.
*/
void 	balance_pan_tilt(int accel_data[3], float *servo0_val, float *servo1_val);

/*
	The car is driven without motor feedback. The only task for the car is to keep the 2DOF gadget stable and 
	balanced, do not drop the ball! 
	The calculated servo motor setpoints are given to the car driving event handler, as well as the command 
	from serial port (if any).
*/
void 	car_driving_event_handler(float servo0_val, float servo1_val, char cmd);
#endif
