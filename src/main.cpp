#include "lcsm.h"
#include "hw_resources.h"
#include "hw_functions.h"

#define LCD_DEBUG
/*	
	The available hardware resources are defined as "extern" in hw_resources.h	
*/
Servo servo0; 
Servo servo1;

/*
	Setpoints, inputs and outputs are used for the PID controllers.
	Setpoint:	Desired sensor values
	Input:		Measured sensor values
	Output:		Between (PID_OUTPUT_LIMIT_L,PID_OUTPUT_LIMIT_U), used as output to the servo motors for balance keeping.
*/
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;

PID myPID1(&Input1, &Output1, &Setpoint1,1.5,4,0, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2,1.5,4,0, DIRECT);

int 	flag_break=0;
char 	status_reg=0x00;
float 	servo0_val=SERVO_PWM_90D;
float 	servo1_val=SERVO_PWM_90D;
int 	accel_data[3];
int 	flag_timer2_interrupt;
int 	Timer2IntrCounter=0;
int 	sen_ultrasonic_distance=0;
int 	sen_light_intensity=0;

char 	f_lc_state=0x00;
char 	serial_cmd;

/* An I2C controlled LCD is used for indication purpose. */
LiquidCrystal_I2C lcd(LCD_I2C_SLAVE,LCD_N_CHARS_PER_ROW,LCD_N_ROWS);

/*
Life Cycle State Machine:

Only set or clear flags in s_lc_update(). 
State related executions are carried 
out in s_lc_actions().

@lc_curr_state: Current life cycle state
@lc_next_state: Next life cycle state
*/

//xtern int lc_curr_state=S_LC_DEP_CREATING;
// int lc_next_state=S_LC_DEP_CREATING;

/*
Functional state machine 0:
Note: multiple functional state machines may
exist in one LCSM.

Only set or clear flags in s_update(). 
State related executions are carried 
out in s_actions().

*/

/* 
	Note:
	States and hieracheis are defined explicitly in this example due to hardware limitation.
	This is definitely _NOT_ a good way to describe state machines since the scalability (code resue)
	is very low. Think about how to improve it. 
*/

/*
 * Hierachy 0
 * Root of life cycle state machine
 * Single state.
 * @LCSM_root
 */

 state LCSM_root;

/*
 * Hierachy 1
 * Deploying and active states of life cycle state machine
 * Two states.
 * @LCSM_deploying
 * @LCSM_active
 */

 state LCSM_deploying;
 state LCSM_active;

/*
 * Hierachy 2 (1/2)
 * States in deploying (../LCSM)
 * 3 states.
 * @DEP_creating
 * @DEP_deleting
 * @DEP_res_conf
 */
 state DEP_creating;
 state DEP_deleting;
 state DEP_res_conf;
/*
 * Hierachy 2 (2/2)
 * States in active (../LCSM)
 * 3 states.
 * @ACT_cap_conf
 * @ACT_pausing
 * @ACT_running
 */
 state ACT_cap_conf;
 state ACT_pausing;
 state ACT_running;
/*
 * Hierachy 3 (1/1)
 * States in RUNNING state (../Active, ../LCSM)
 * 4 states.
 * @RUN_idle
 * @RUN_acquire
 * @RUN_compute
 * @RUN_action
 */
 state RUN_idle;
 state RUN_acquire;
 state RUN_compute;
 state RUN_action;

/*
	Define state and event variables
	Typically a state machine needs two state variables for current state and next state.
	An event triggers the transition from one state to another.
*/

/* Q: Make a schematic for the hierachical state machine */ 

int LC_Event, FUNC0_Event;

/*current and next state variables for life cycle state machine*/
state lc0_current_state;
state lc0_next_state;

/*current and next state variables for Deployment state machine in LCSM */
state lc0_dep_current_state;
state lc0_dep_next_state;

/*current and next state variables for Active state machine in LCSM */
state lc0_act_current_state;
state lc0_act_next_state;

/*current and next state variables for Running state machine in Active */
state lc0_act_run_current_state;
state lc0_act_run_next_state;

/*current and next state variables for user state machine in Running */
state user_func_current_state;
state user_func_next_state;

/* user defined state machine update (states in Running in Active) */
void user_func_state_update(){
	if(lc0_act_current_state.idx==S_LC_ACT_RUNNING){	/*user state machine is updated only if the Active state machine is in Running state. */
	    switch(user_func_current_state.idx){
	      case S_FUNC0_RUN_IDLE:
		switch(FUNC0_Event){
			case EV_DRIVING_CMD:
                        case EV_TIME_OUT:	/* Both events trigger moving from IDLE state to RUN state */
				user_func_next_state=RUN_action;
			break;
			default:
				user_func_next_state=user_func_current_state;
			break;
		}
	      break;
	      case S_FUNC0_RUN_DEMO_ACT:
			user_func_next_state=RUN_idle;
	      break;  
	    }  
	}
    user_func_current_state=user_func_next_state;
}

/* user defined state machine action (states in Running in Active) */
void user_func_state_action(){
	if(lc0_act_current_state.idx==S_LC_ACT_RUNNING){
	    switch(user_func_current_state.idx){
	      case S_FUNC0_RUN_IDLE:
	      break;
	      case S_FUNC0_RUN_DEMO_ACT:
	      	car_driving_event_handler(servo0_val,servo1_val,serial_cmd);
	      break;  
	    }  
	}
    user_func_current_state=user_func_next_state;
}

void setup()
{
	/* Resource initialization */
	Serial.begin(9600);
        lcd.init();
        lcd.backlight();
	servo0.attach(SERVO1_PWM);
	servo1.attach(SERVO2_PWM);
	pinMode(LED_PIN, OUTPUT);
	pinMode(LIGHT_SENSOR_PIN, INPUT);
        Setpoint1=0.0;	
        Setpoint2=13.0;
       
	/* PID controllers initialization */
        myPID1.SetMode(AUTOMATIC);
        myPID1.SetSampleTime(PID_UPDATE_TIME);
        myPID1.SetOutputLimits(PID_OUTPUT_LIMIT_L,PID_OUTPUT_LIMIT_U);
        myPID2.SetMode(AUTOMATIC);
        myPID2.SetSampleTime(PID_UPDATE_TIME);
        myPID2.SetOutputLimits(PID_OUTPUT_LIMIT_L,PID_OUTPUT_LIMIT_U);

	/* Initialize Timer2 as the heart beat */	
#ifdef USE_TIMER2
	MsTimer2::set(TIMER2_INT_DURATION, Timer2Handler);
	MsTimer2::start();
#endif
	/* Assign state ID to each state and sub state */
	LCSM_root.idx		=	S_LC_ROOT;
	LCSM_deploying.idx	=	S_LC_DEP;
	LCSM_active.idx		=	S_LC_ACT;
	
	DEP_creating.idx	=	S_LC_DEP_CREATING;
	DEP_deleting.idx	=	S_LC_DEP_DELETING;
	DEP_res_conf.idx	=	S_LC_DEP_RES_CONF;
	
	ACT_cap_conf.idx	=	S_LC_ACT_CAP_CONF;
	ACT_pausing.idx		=	S_LC_ACT_PAUSING;
	ACT_running.idx		=	S_LC_ACT_RUNNING;
	
	RUN_idle.idx		=	S_FUNC0_RUN_IDLE;
	RUN_acquire.idx		=	S_FUNC0_RUN_DATA_ACQ;
	RUN_compute.idx		=	S_FUNC0_RUN_DATA_CMP;
	RUN_action.idx		=	S_FUNC0_RUN_DEMO_ACT;
	
	/* Initialize current and next states for each state machine */
	lc0_current_state	=	LCSM_deploying;
	lc0_dep_current_state	=	DEP_creating;
	lc0_act_current_state	=	ACT_cap_conf;
	user_func_current_state	=	RUN_idle;
	lc0_next_state		=	LCSM_deploying;
	lc0_dep_next_state	=	DEP_creating;
	lc0_act_next_state	=	ACT_cap_conf;
	user_func_next_state	=	RUN_idle;
}


void lc_event_handler(){
	/*The transition of the states in Deployment*/
	switch(status_reg){
	case 0x01:
		LC_Event = EV_LC_CRE_RES_CONF;
	break;
	case 0x03:
		LC_Event = EV_LC_RES_CONF_CAP_CONF;
	break;
	case 0x0B:
		LC_Event = EV_LC_CAP_CONF_PAU;
	break;
	}
	
/* 
	The life cycle state machine starts up when the Arduino board is powered on, the 
	states move from Creating to Resource Configuration in Deployment state machine,
	and then to the Capability Configuration in Active state machine, and it stops at
	Pausing state of Active in the LCSM.

	To start the car, you should trigger the state machine to state "Running".
	Steps:
	1. Place an object or your hand in front of the ultrasonic sensor, you can read the 
	   detected distance on the LCD if you enable LCD_DEBUG. Keep the distance at "10" cm
	2. Use a light source to trigger the light sensor, if sen_light_intensity is greater
	   than EVENT_LIGHT_TRIGGER_VAL, LC_Event will get the value to trigger the state 
	   machine from Pausing state to Run. Notice that EVENT_LIGHT_TRIGGER_VAL should be 
	   sufficiently high to distinguish the room light intensity, here it is set to 800 
	   (0-1023 for analog reading)
	
	Other events can be triggered in the same way but a different object distance. See below.
*/
#define EVENT_LIGHT_TRIGGER_VAL 800
	if(sen_light_intensity>EVENT_LIGHT_TRIGGER_VAL){
		switch(sen_ultrasonic_distance){
			case 10:
			LC_Event = EV_LC_PAU_RUN;	
			break;	
			case 20:
			LC_Event = EV_LC_RUN_PAU;
			break;	
			case 30:
			LC_Event = EV_LC_PAU_CAP_CONF;
			break;		
			case 40:
			LC_Event = EV_LC_CAP_CONF_RES_CONF;
			break;
			case 50:
			LC_Event = EV_LC_RES_CONF_DEL;
			break;	
			case 60:
			LC_Event = EV_LC_DEL_CRE;
			break;										
		}
	}
}

/* User function handler */
void user_func_event_handler(){
	if(Serial.available() > 0){
		FUNC0_Event = EV_DRIVING_CMD;
		serial_cmd=Serial.read();
	}
	if(flag_timer2_interrupt==1)
		FUNC0_Event = EV_TIME_OUT;
}

void loop(){
	char debug_txt[64]; /* Define a string container for the debug information */

	/*Life Cycle State Machine state update*/
	LC0_state_update();
	LC0_DEP_state_update();
	LC0_ACT_state_update();

	/*User State Machine state update*/
	user_func_state_update();

	/*Life Cycle State Machine state actions*/
	LC0_state_action();
	LC0_DEP_state_action();
	LC0_ACT_state_action();

	if(flag_timer2_interrupt==1){
	      sen_ultrasonic_distance=ultrasonic_distance();
              flag_break=obj_detection(sen_ultrasonic_distance);
              read_adxl345(accel_data);
              balance_pan_tilt(accel_data, &servo0_val, &servo1_val);
              switch(Timer2IntrCounter%4){
                case 0:
                          sen_light_intensity=photoresistor_val(LIGHT_SENSOR_PIN);
	                  light_on(sen_light_intensity);
                break;
              }
              flag_timer2_interrupt=0;
	}

	
	/*User State Machine state actions*/
	user_func_state_action();

	/*Handle event in LCSM and user state machine */
	lc_event_handler();
	user_func_event_handler();
	

/*
	Options to print debug information on the LCD, e.g. the current states for 
	each of the state machine.
*/
#ifdef LCD_DEBUG
        lcd.setCursor(0,0);
        sprintf(debug_txt, "%4d,%4d", sen_light_intensity, sen_ultrasonic_distance);
        lcd.print(debug_txt);
        lcd.setCursor(0,1);
        sprintf(debug_txt, "%x %x %x %x %x", 
              lc0_dep_current_state.idx,lc0_act_current_state.idx,lc0_current_state.idx, user_func_current_state.idx, LC_Event);
        lcd.print(debug_txt);
        lcd.setCursor(15,1);
        lcd.print(serial_cmd);
        lcd.setCursor(13,0);
	sprintf(debug_txt,"%2d",Timer2IntrCounter);
        lcd.print(debug_txt);
#endif
}
