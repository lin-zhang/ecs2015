#ifndef HW_RESOURCES
#define HW_RESOURCES
/* Parameters are defined in config.h */
#include "config.h"

/* Include headers for necessary libraries */
#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <MsTimer2.h>
#include <PID_v1.h>
/* 
All digital IOs are occupied by motors or other functions,
but the 6 analog IOs are still available for sensors and 
indicators. 
*/
#define UR_ECHO_PIN 		A0
#define UR_TRIG_PIN 		A1
#define LIGHT_SENSOR_PIN	A2
#define LED_PIN			A3
#define I2C_SDA_PIN		A4	//The I2C SDA and SCL Pins are defined in <Wire.h>
#define I2C_SCL_PIN		A5	//They are listed here for code completeness	

// Arduino pins for the shift register
#define MOTORLATCH      12
#define MOTORCLK        4
#define MOTORENABLE     7
#define MOTORDATA       8

// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A        2
#define MOTOR1_B        3
#define MOTOR2_A        1
#define MOTOR2_B        4
#define MOTOR3_A        5
#define MOTOR3_B        7
#define MOTOR4_A        0
#define MOTOR4_B        6

// Arduino (digital) pins for the PWM signals.
#define MOTOR1_PWM      11
#define MOTOR2_PWM      3
#define MOTOR3_PWM      6
#define MOTOR4_PWM      5
#define SERVO1_PWM      10
#define SERVO2_PWM      9

#define MOTOR_ERROR     -3333

// Codes for the motor function.
#define BACKWARD        1
#define FORWARD         2
#define BRAKE           3
#define RELEASE         4


/*
LCD Configuration
The LCD used on the car can display 16x2 characters, it's
I2C slave register is 0x27.

I2C devices can be cascaded but the slave register must be
different for each of the devices on the I2C bus. 
*/

#define LCD_I2C_SLAVE 		0x27
#define LCD_N_CHARS_PER_ROW 	16
#define LCD_N_ROWS 		2


/* Two servo motors are available in this project. */
extern Servo servo0;
extern Servo servo1;

/* One I2C controlled LCD is used for indication purpose. */
//LiquidCrystal_I2C lcd(LCD_I2C_SLAVE,LCD_N_CHARS_PER_ROW,LCD_N_ROWS); 

extern double Setpoint1, Input1, Output1;
extern double Setpoint2, Input2, Output2;

extern PID myPID1;
extern PID myPID2;


#endif
