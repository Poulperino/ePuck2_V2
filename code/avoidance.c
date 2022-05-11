#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <sensors/proximity.h>
#include <selector.h>
#include <motors.h>
#include <arm_math.h>

#include <avoidance.h>
#include <state_machine.h>

#define OFFSET_SPEED 	400
#define THRESHOLD		7

extern messagebus_t bus;

static int dist_values[PROXIMITY_NB_CHANNELS];


void ShowValue(int16_t sum){
	static int show = 0;
	show++;
	if(show>=8){
	show = 0;
	//chprintf((BaseSequentialStream *)&SD3, "speed=%d\n", sum + OFFSET_SPEED);
	//chprintf((BaseSequentialStream *)&SD3, "IR=%d\n", dist_values[0]);
	}
}

static int right_speed_to_set = 0;
static int left_speed_to_set = 0;

static THD_WORKING_AREA(waAvoidance, 256);
static THD_FUNCTION(Avoidance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	// messagebus variables
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t thd_prox_values;

    // calibrate proximity sensors
    calibrate_ir();

    // value array and display counter
    //static int dist_values[PROXIMITY_NB_CHANNELS];
    //static int8_t weights_l[PROXIMITY_NB_CHANNELS] = {-96,-64,-32,0,0,32,64,96};
    //static int16_t weights_l[PROXIMITY_NB_CHANNELS] = {-192,-128,-64,0,0,64,128,192};
    static int8_t weights_l[PROXIMITY_NB_CHANNELS] = {-96,-64,-64,0,0,64,64,96};
    //static int8_t weights_r[PROXIMITY_NB_CHANNELS] = {96,64,32,0,0,-32,-64,-96};

    int16_t tmp_r = 0;
    int16_t tmp_l = 0;

    uint8_t acc = 0;
    bool activated = false;

    uint8_t state = 0; //for state machine

    systime_t time;

	while (1) {
		messagebus_topic_wait(prox_topic, &thd_prox_values, sizeof(thd_prox_values));

		for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
		    // get prox_values and check positiveness
		    dist_values[i] = thd_prox_values.delta[i]-thd_prox_values.initValue[i];
		   	if(dist_values[i]<0){dist_values[i] = 0;}
		    // accumulate speed factor and divide by 128
		    tmp_l += ((int32_t)(weights_l[i]*dist_values[i]))>>7;
		    tmp_r -= ((int32_t)(weights_l[i]*dist_values[i]))>>7;
		    //chprintf((BaseSequentialStream *)&SDU1, "IR%d=%d\n",i, tmp);
		}

		// filter noise
		(abs(tmp_r) < THRESHOLD) && (tmp_r = 0);
		(abs(tmp_l) < THRESHOLD) && (tmp_l = 0);

		int selector = get_selector();
		state = get_state(); //added for the state machine control
		switch(selector){
		case 0:
			ShowValue(tmp_l);
			break;

		case 1:
			// activate the motors after n cycles (time to remove hand)
			if(!activated){
				(acc >= 50) ? (activated = true) : (acc++);
			}else{
				if(state == 1){
					acc = 0;
					//left_motor_set_speed(tmp_l + OFFSET_SPEED);
					left_speed_to_set = (tmp_l + OFFSET_SPEED);
					//right_motor_set_speed(tmp_r + OFFSET_SPEED);
					right_speed_to_set = (tmp_r + OFFSET_SPEED);
				}
				else{
					//left_motor_set_speed(0);
					left_speed_to_set = 0;
					//right_motor_set_speed(0);
					right_speed_to_set = 0;
				}
			}
			break;

		default:
			break;
		}

		// reset activation state and speed
		if(selector != 1){
			activated = false;
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		tmp_r = 0;
    	tmp_l = 0;

    	time = chVTGetSystemTime();
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

//Get right speed to set from avoidance
int avoidance_right_speed(void){
	return (right_speed_to_set);
}

//Get left speed to set from avoidance
int avoidance_left_speed(void){
	return (left_speed_to_set);
}

void avoidance_start(void){
	chThdCreateStatic(waAvoidance, sizeof(waAvoidance), NORMALPRIO+1, Avoidance, NULL);
}

