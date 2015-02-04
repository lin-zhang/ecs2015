#include "hw_functions.h"

void Timer2Handler(){
  if (Timer2IntrCounter<TIMER2_INT_CNT_MAX)
	Timer2IntrCounter++;
  else{
	Timer2IntrCounter=0;
  }
flag_timer2_interrupt=1;
}

void i2c_write(int address, byte reg, byte data) {
  Wire.beginTransmission(address);
  /* Send output register address */
  Wire.write(reg);
  /* Connect to device and send byte */
  Wire.write(data);
  Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data) {
 int i = 0;
 /* Send input register address */
 Wire.beginTransmission(address);
 Wire.write(reg);
 Wire.endTransmission();
 /* Connect to device and request bytes */
 Wire.beginTransmission(address);
 Wire.requestFrom(address,count);
 while(Wire.available()) 
 {
   char c = Wire.read(); /* receive a byte as character */
   data[i] = c;
   i++;
 }
 Wire.endTransmission();
}

void init_adxl345() {
  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
}

void read_adxl345(int accel_data[ADXL_N_ACCEL_AXES]) {
 byte bytes[ADXL_N_ACCEL_DATA_BYTES];
 memset(bytes,0,ADXL_N_ACCEL_DATA_BYTES);
 i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, ADXL_N_ACCEL_DATA_BYTES, bytes);
 for (int i=0;i<ADXL_N_ACCEL_AXES;++i) {
 accel_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
 }
}


int ultrasonic_distance(){
/* Attach pin A1 to Trig */
  int duration=0;
  pinMode(UR_TRIG_PIN, OUTPUT);
  digitalWrite(UR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(UR_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(UR_TRIG_PIN, LOW);
/* Attach pin A0 to Echo */
  pinMode (UR_ECHO_PIN, INPUT);
  duration = pulseIn(UR_ECHO_PIN, HIGH);

/* The speed of sound is 340 m/s or 29 microseconds per centimeter.
   The ping travels out and back, so to find the distance of the
   object we take half of the distance travelled. */

 /* Q: This is not accurate because of integer type. How do you improve it? */
  return duration/29/2;	
}

int photoresistor_val(int pin){
	return analogRead(pin);
}

/*
        Setpoints, inputs and outputs are used for the PID controllers.
        Setpoint:       Desired sensor values
        Input:          Measured sensor values
        Output:         Between (PID_OUTPUT_LIMIT_L,PID_OUTPUT_LIMIT_U), used as output to the servo motors for balance keeping.
*/
void balance_pan_tilt(int accel_data[3], float *servo0_val, float *servo1_val){
if(MA_cnt<MA_MAX_ACCEL){
	/*MA_filter_buf1 and MA_filter_buf2 are declared in hw_functions.h*/
        MA_filter_buf1[MA_cnt]=accel_data[0];	
        MA_filter_buf2[MA_cnt]=accel_data[1];
        MA_cnt++;
}
else{
        MA_cnt=0;
}
        Input1=0;
        Input2=0;
	for(int i=0;i<MA_MAX_ACCEL;i++){
	        Input1+=MA_filter_buf1[i];
	        Input2+=MA_filter_buf2[i];
	}

        Input1/=MA_MAX_ACCEL;
        Input2/=MA_MAX_ACCEL;

        myPID1.Compute();
        myPID2.Compute();

	/* Outputs from the PID controllers are used as offsets */
        *servo0_val=SERVO_PWM_90D-Output1;	
        *servo1_val=SERVO_PWM_90D+Output2;
}

void light_on(int light_intensity){
	if(light_intensity<SEN_LIGHT_DARK_THRESHOLD)
	    	digitalWrite(LED_PIN,HIGH);
    	else
    		digitalWrite(LED_PIN,LOW);
}

int obj_detection(int distance){
	if(distance<SEN_USONIC_COLLISION_THRESHOLD)
        	return 1;
	else 
		return 0;
}

int MA_cnt_servo=0;
int motor_speed=MOTOR_BASE_SPEED;

/*Q: Motor M1 and M2 are not used because of the usage of Timer2, why? */
void car_driving_event_handler(float servo0_val, float servo1_val, char cmd){
      if(flag_break==1){
        if(cmd=='w')
          cmd='b';
          flag_break=0;
        }
   switch (cmd){            
      case 'w':               
        //motor_drive(1, BACKWARD, motor_speed);	
        //motor_drive(2, BACKWARD, motor_speed);
        motor_drive(3, BACKWARD, motor_speed);
        motor_drive(4, BACKWARD, motor_speed);
        if(motor_speed<MOTOR_MAX_SPEED)
        motor_speed+=MOTOR_SPEED_INCR;
        else
        motor_speed=MOTOR_MAX_SPEED; 
       break;
      case 'a':
	motor_speed=LEFT_TURN_SPEED;             
        //motor_drive(1, FORWARD, motor_speed);
        //motor_drive(2, BACKWARD, motor_speed);
        motor_drive(3, BACKWARD, motor_speed);
        motor_drive(4, FORWARD, motor_speed);
	
        break;
      case 's':  
      case 'b':              
        //motor_drive(1, RELEASE, 0);
        //motor_drive(2, RELEASE, 0);
        motor_drive(3, RELEASE, 0);
        motor_drive(4, RELEASE, 0);
        motor_speed=MOTOR_BASE_SPEED;
	break;
      case 'd':              
	motor_speed=RIGHT_TURN_SPEED;
        //motor_drive(1, BACKWARD, motor_speed);
        //motor_drive(2, FORWARD, motor_speed);
        motor_drive(3, FORWARD, motor_speed);
        motor_drive(4, BACKWARD, motor_speed);
        break;
      case 'x':             
        //motor_drive(1, FORWARD, motor_speed);
        //motor_drive(2, FORWARD, motor_speed);  
        motor_drive(3, FORWARD, motor_speed);
        motor_drive(4, FORWARD, motor_speed);
        if(motor_speed<MOTOR_MAX_SPEED)
        motor_speed+=MOTOR_SPEED_INCR;
        else
        motor_speed=MOTOR_MAX_SPEED;
        break;
    }	
    servo0.writeMicroseconds(servo0_val);  
    servo1.writeMicroseconds(servo1_val);  

      
}

// ---------------------------------
// motor_drive
//
// Select the motor (1-4), the command, 
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.
//

void motor_drive(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      PWM_output (motorA, HIGH, speed);
      PWM_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      PWM_output (motorA, LOW, speed);
      PWM_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling 
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      PWM_output (motorA, LOW, speed); // 255: fully on.
      PWM_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      PWM_output (motorA, LOW, 0);  // 0: output floating.
      PWM_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// PWM_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids, 
// DC motors (but not in reverse).
//
// It is also used as an internal helper function 
// for the motor() function.
//
// The high_low variable should be set 'HIGH' 
// to drive lights, etc.
// It can be set 'LOW', to switch it off, 
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for 
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the 
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void PWM_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    speed = MOTOR_ERROR;
    break;
  }

  if (speed != MOTOR_ERROR)
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}

// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind 
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}

