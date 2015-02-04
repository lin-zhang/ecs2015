#include "lcsm.h"

extern char status_reg;
extern int LC_Event;
extern state lc0_current_state;
extern state lc0_next_state;
extern state lc0_dep_current_state;
extern state lc0_dep_next_state;
extern state lc0_act_current_state;
extern state lc0_act_next_state;

extern state func0_current_state;
extern state func0_next_state;

void LC0_state_update(){
	switch(lc0_current_state.idx){
		case S_LC_DEP:
		if(LC_Event==EV_LC_RES_CONF_CAP_CONF)
			lc0_next_state=LCSM_active;
		else
			lc0_next_state=lc0_current_state;
		break;
		case S_LC_ACT:
		if(LC_Event==EV_LC_CAP_CONF_RES_CONF)
			lc0_next_state=LCSM_deploying;
		else
			lc0_next_state=lc0_current_state;
		break;
	}
	lc0_current_state=lc0_next_state;
}

void LC0_state_action(){
	switch(lc0_current_state.idx){
		case S_LC_DEP:
		break;
		case S_LC_ACT:
		break;
	}
}

void LC0_DEP_state_update(){
	if(lc0_current_state.idx==S_LC_DEP){
		switch(lc0_dep_current_state.idx){
		      case S_LC_DEP_CREATING:
			if(LC_Event==EV_LC_CRE_RES_CONF)
				lc0_dep_next_state=DEP_res_conf;
			else
				lc0_dep_next_state=lc0_dep_current_state;
		      break;
		      case S_LC_DEP_DELETING:
			if(LC_Event==EV_LC_DEL_CRE)
				lc0_dep_next_state=DEP_creating;
			else
				lc0_dep_next_state=lc0_dep_current_state;
		      break;
		      case S_LC_DEP_RES_CONF:
			if(LC_Event==EV_LC_RES_CONF_DEL)
				lc0_dep_next_state=DEP_deleting;
			else
				lc0_dep_next_state=lc0_dep_current_state;
		      break;
		}
	}
	lc0_dep_current_state=lc0_dep_next_state;
}

void lc_dep_creating(){
	/*Initialize all resources and set bit_cre flag */
	bitSet(status_reg,BIT_CRE);
}

void lc_dep_deleting(){
	/*Release all resources and clear all status flags */
	bitSet(status_reg,BIT_DEL);
}

void lc_dep_res_conf(){
	/*Config the used resources and set flag resource_conf*/
	init_adxl345();
	bitSet(status_reg,BIT_RCF);
}

void LC0_DEP_state_action(){
	if(lc0_current_state.idx==S_LC_DEP){
		switch(lc0_dep_current_state.idx){
		      case S_LC_DEP_CREATING:
			lc_dep_creating();
		      break;
		      case S_LC_DEP_DELETING:
			lc_dep_deleting();
		      break;
		      case S_LC_DEP_RES_CONF:
			lc_dep_res_conf();
		      break;
		}
	}
}
void LC0_ACT_state_update(){
	if(lc0_current_state.idx==S_LC_ACT){
		switch(lc0_act_current_state.idx){
		      case S_LC_ACT_CAP_CONF:
			switch(LC_Event){
				case EV_LC_CAP_CONF_PAU:
					lc0_act_next_state=ACT_pausing;
				break;
				default:
					lc0_act_next_state=lc0_act_current_state;
				break;
			}
		      break;
		      case S_LC_ACT_PAUSING:
			switch(LC_Event){
				case EV_LC_PAU_CAP_CONF:
					lc0_act_next_state=ACT_cap_conf;
				break;
				case EV_LC_PAU_RUN:
					lc0_act_next_state=ACT_running;
				break;
				default:
					lc0_act_next_state=lc0_act_current_state;
				break;
			}
		      break;
		      case S_LC_ACT_RUNNING:
			switch(LC_Event){
				case EV_LC_RUN_PAU:
					lc0_act_next_state=ACT_pausing;
				break;
				default:
					lc0_act_next_state=lc0_act_current_state;
				break;
		        }
		      break;     
		}
	}
	lc0_act_current_state=lc0_act_next_state;
}

void lc_act_cap_conf(){
	/*set bit_capability_conf flag */
	bitSet(status_reg,BIT_CCF);
	LC_Event = EV_LC_CAP_CONF_PAU;
}

void lc_act_pausing(){
	/*Pause activities, set bit_pausing flag and clear bit_running flag*/
	bitClear(status_reg,BIT_RUN);
	bitSet(status_reg,BIT_PAU);
}

void lc_act_running(){
	/*Run activities, set bit_running flag and clear bit_pausing flag*/
	bitClear(status_reg,BIT_PAU);
	bitSet(status_reg,BIT_RUN);

}

void LC0_ACT_state_action(){
	if(lc0_current_state.idx==S_LC_ACT){
		switch(lc0_act_current_state.idx){
		      case S_LC_ACT_CAP_CONF:
				lc_act_cap_conf();
		      break;
		      case S_LC_ACT_PAUSING:
				lc_act_pausing();
		      break;
		      case S_LC_ACT_RUNNING:
				lc_act_running();
		      break;     
		}
	}
}
