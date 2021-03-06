#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

int16_t pi_regulator(float angle, float goal){
	float error = 0;
		float speed = 0;

		static float sum_error = 0;

		error = angle - goal;

		//disables the PI regulator if the error is to small
		//this avoids to always move as we cannot exactly be where we want and
		//the camera is a bit noisy
		if(fabs(error) < ERROR_THRESHOLD){
			return 0;
		}

		sum_error += error;

		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR){
			sum_error = MAX_SUM_ERROR;
		}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
		}

		speed = KP * error + KI * sum_error;

	    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    //static float sum_error = 0;
    //float error = 0;

    while(1){
        time = chVTGetSystemTime();

        /*
		*	To complete
		*/
        /*error = get_distance_cm() - GOAL_DISTANCE;

        if(fabs(error) < ERROR_THRESHOLD){
        	return 0;
        }

        sum_error += error;

        if(sum_error > MAX_SUM_ERROR){
        	sum_error = MAX_SUM_ERROR;
        }
        else if(sum_error < -MAX_SUM_ERROR){
        	sum_error = -MAX_SUM_ERROR;
        }*/

        //speed = (int16_t)(KP * error + KI * sum_error);
        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        speed_correction = (get_line_position()-(IMAGE_BUFFER_SIZE/2));

        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator
		right_motor_set_speed((speed - ROTATION_COEFF * speed_correction));
		left_motor_set_speed((speed + ROTATION_COEFF*speed_correction));

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

