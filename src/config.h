#ifndef CONFIG_H
#define CONFIG_H
/*
Timer 2

Code source: http://playground.arduino.cc/Main/MsTimer2

TIMER2_INT_DURATION:
Timer2 generates interrupt every [TIMER2_INT_DURATION] microseconds.

TIMER2_INT_CNT_MAX:
A counter is implemented in the interrupt routine to count the number
of interrupts generated by timer2. 
*/
#define USE_TIMER2

#define TIMER2_INT_DURATION 		50
#define TIMER2_INT_CNT_MAX 		5

/*
@SEN_LIGHT_DARK_THRESHOLD:
If the environment gets dark, the LED would be turned on automatically.
SEN_LIGHT_DARK_THRESHOLD defines the threshold of darkness, it ranges from 
0 to 1023, representing 0 to 5V. 
*/
#define SEN_LIGHT_DARK_THRESHOLD 	200

/*
@SEN_USONIC_COLLISION_THRESHOLD
The ultrasonic range finder is used to detect obstacles in front of the car.
When the car is too close to the obstacle, a FORWARD moving command will be
ignored.
*/
#define SEN_USONIC_COLLISION_THRESHOLD	22

/*
@SERVO_PWM_90D
In general, a servo motor rotates between 0 to 180 degrees. The position at
90 degree is often used as the middle point.

The servo motor is controlled by PWM signal with a constant frequency. The
position of the motor is proportional to the duty cycle of the PWM. 

The HS485-HB servo motor keeps stays at 90 degree by applying a PWM with 
1442 us duty cycle.
*/
#define SERVO_PWM_90D 			1442

/*
The pan-tilt gadget is controlled by the accelerometer. ACCEL_CALIB_X and 
ACCEL_CALIB_Y are used in some functions to calibrate the sensor for a 
better control result. 
*/
#define ACCEL_CALIB_X 			-13 /* Q: Where to use these calibration factors? */
#define ACCEL_CALIB_Y 			35	

/*
Moving average filter
Smoothen the signal received from the accelerometer to remove the noise.
Moving average introduces delay into the system, there is a trade off between
response delay and noise reduction.
*/

#define MA_MAX_ACCEL	 		10	

/*
MOTOR_BASE_SPEED: defines the start up speed for the wheels. 
MOTOR_SPEED_INCR: defines how much the motor may speed-up if a moving forward/backward command is received continuously.
MOTOR_MAX_SPEED : defines the max speed of a motor. (0-255 PWM)
LEFT_TURN_SPEED and RIGHT_TURN_SPEED defines the speed when turing left or right.
*/
#define MOTOR_BASE_SPEED		160	
#define MOTOR_SPEED_INCR		4
#define MOTOR_MAX_SPEED			240
#define LEFT_TURN_SPEED			200
#define RIGHT_TURN_SPEED		200

/*
The PID controller is triggered every PID_UPDATE_TIME milliseconds.
*/
#define PID_UPDATE_TIME			50 	/* milliseconds */
#define PID_OUTPUT_LIMIT_L		-255	/* set lower limit to -255 since PWM is between 0-255, with sign for direction */
#define PID_OUTPUT_LIMIT_U		255	/* set upper limit to 255 */

#endif