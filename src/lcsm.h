#ifndef LCSM_H
#define LCSM_H

#include "Arduino.h"

extern void init_adxl345(void);

/*State definitions*/
#define S_LC_ROOT 		0xFF

#define S_LC_ACT  		0xFE
#define S_LC_DEP  		0xFD  

#define S_LC_DEP_CREATING 	0x01
#define S_LC_DEP_DELETING 	0x02
#define S_LC_DEP_RES_CONF 	0x03

#define S_LC_ACT_CAP_CONF 	0x10
#define S_LC_ACT_PAUSING 	0x11
#define S_LC_ACT_RUNNING 	0x12

#define S_FUNC0_RUN_IDLE 	0x120
#define S_FUNC0_RUN_DATA_ACQ 	0x121
#define S_FUNC0_RUN_DATA_CMP 	0x122
#define S_FUNC0_RUN_DEMO_ACT 	0x123

#define EV_NULL 		0x00
#define EV_LC_CRE_RES_CONF 	0x10
#define EV_LC_RES_CONF_DEL 	0x11
#define EV_LC_RES_CONF_CAP_CONF 0x12
#define EV_LC_CAP_CONF_RES_CONF 0x13
#define EV_LC_CAP_CONF_PAU 	0x14
#define EV_LC_PAU_CAP_CONF 	0x15
#define EV_LC_PAU_RUN 		0x16
#define EV_LC_RUN_PAU 		0x17
#define EV_LC_DEL_CRE 		0x18


/*
Flag register f_lc_state: Yes/Set=1, No/Clear=0.
Each state is associated with 1 bit:

#---Deploying---#
bit 0: Creation is done?
bit 1: Resource configration done?
bit 2: Deletion is done?

#----Active----#
bit 3: Capability configuration done?
bit 4: Paused?
bit 5: Running?
*/

#define BIT_CRE  0
#define BIT_RCF  1
#define BIT_DEL  2
#define BIT_CCF  3
#define BIT_PAU  4
#define BIT_RUN  5


struct state{
	int 	idx;
	state* 	substate;
};


/*
Life Cycle State Machine:

Only set or clear flags in s_lc_update(). 
State related executions are carried 
out in s_lc_actions().

@lc_curr_state: Current life cycle state
@lc_next_state: Next life cycle state
*/

//xtern int lc_curr_state=S_LC_DEP_CREATING;
//extern int lc_next_state=S_LC_DEP_CREATING;

/*
Functional state machine 0:
Note: multiple functional state machines may
exist in one LCSM.

Only set or clear flags in s_update(). 
State related executions are carried 
out in s_actions().

@func_curr_state: Current functional state
@func_next_state: Next functional state
*/

/*
 * Hierachy 0
 * Root of life cycle state machine
 * Single state.
 * @LCSM_root
 */

extern state LCSM_root;

/*
 * Hierachy 1
 * Deploying and active states of life cycle state machine
 * Two states.
 * @LCSM_deploying
 * @LCSM_active
 */

extern state LCSM_deploying;
extern state LCSM_active;

/*
 * Hierachy 2 (1/2)
 * States in deploying (../LCSM)
 * 3 states.
 * @DEP_creating
 * @DEP_deleting
 * @DEP_res_conf
 */
extern state DEP_creating;
extern state DEP_deleting;
extern state DEP_res_conf;
/*
 * Hierachy 2 (2/2)
 * States in active (../LCSM)
 * 3 states.
 * @ACT_cap_conf
 * @ACT_pausing
 * @ACT_running
 */
extern state ACT_cap_conf;
extern state ACT_pausing;
extern state ACT_running;
/*
 * Hierachy 3 (1/1)
 * States in RUNNING state (../Active, ../LCSM)
 * 4 states.
 * @RUN_idle
 * @RUN_acquire
 * @RUN_compute
 * @RUN_action
 */
extern state RUN_idle;
extern state RUN_acquire;
extern state RUN_compute;
extern state RUN_action;
extern int LC_Event, FUNC0_Event;

void LC0_state_update();

void LC0_state_action();
void LC0_DEP_state_update();
void LC0_DEP_state_action();

void lc_dep_creating();
void lc_dep_deleting();
void lc_dep_res_conf();

void LC0_ACT_state_update();
void LC0_ACT_state_action();
void lc_act_cap_conf();
void lc_act_pausing();
void lc_act_running();

void func0_state_update(state lc0_act_current_state, state func0_current_state, state func0_next_state);
void func0_state_action(state lc0_act_current_state, state func0_current_state);

#define EV_DRIVING_CMD 	0x20
#define EV_TIME_OUT	0x21


#endif
