#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <motors.h>
#include <stdbool.h>

#include <odometry.h>
#include <dumb_speed.h>

//#define TEST_CIRCLE
//#define TEST_DIST
//#define TEST_BOX

static float ePuck_angle = 0;
//static float ePuck_x = 0;
//static float ePuck_y = 0;
static float ePuck_angle_p = 0;
static float ePuck_x_p = 0;
static float ePuck_y_p = 0;
static int32_t ePuck_val_right = 0;
static int32_t ePuck_val_left = 0;
static systime_t prevTime = 0;


//The thread that computes the odometry
static THD_WORKING_AREA(waCompOdometry, 256);
static THD_FUNCTION(CompOdometry, arg) {


    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Variables in the thread
    bool send_to_computer = true;
    int32_t right_motor_val = 0;
    int32_t left_motor_val = 0;
    int32_t prev_right_motor_val = 0;
    int32_t prev_left_motor_val = 0;
	systime_t current_time;
	systime_t time_elapsed;
	int16_t ePuck_speed_right = 0;
	int16_t ePuck_speed_left = 0;
	int8_t circle_counter = 0; //REMOVE FOR REAL CODE

    //To control the thread's frequency
    systime_t time;
    time = chVTGetSystemTime();

#ifdef TEST_BOX
		//REMOVE WHEN INTEGRATED
		go_straight();
#endif

	while(1){
#ifdef TEST_CIRCLE
		//REMOVE WHEN INTEGRATED
	    //Make the robot move a certain pattern: first straight, then turn, then straight, then stop
	    //if(time*MS2S < 10){go_straight();}
	    //else if(time*MS2S<12){go_circle();}
	    //else if(time*MS2S<17){go_straight();}
	    //else{stop_moving();}
		if(circle_counter < 3){
			go_circle_ccw();
		}
		else if(circle_counter < 6){
			go_circle_cw();
		}
		else{
			stop_moving();
			//circle_counter = 0;
		}
#endif

#ifdef TEST_DIST
		//REMOVE WHEN INTEGRATED
		go_straight();
		if(ePuck_x_p > 30){
			stop_moving();
		}
#endif

	    //Get the values from the encoder
	    //For now not useful but might use as a correction of the speed ? Looks good for now
	    right_motor_val = right_motor_get_pos();
	    left_motor_val = left_motor_get_pos();

	    //Get the motors speed values
	    //ePuck_speed_right = get_right_speed_mms();
	    //ePuck_speed_left = get_left_speed_mms();

	    //Compute elapsed time between two call of the thread
	    //current_time = chVTGetSystemTime();
	    //time_elapsed = current_time - prevTime;

	    //Update position and angle based on each motor's speed
	    //ePuck_x += time_elapsed*MS2S*cos(ePuck_angle)*(ePuck_speed_right + ePuck_speed_left)/2;
	    //ePuck_y += time_elapsed*MS2S*sin(ePuck_angle)*(ePuck_speed_right + ePuck_speed_left)/2;
	    //ePuck_angle += time_elapsed*MS2S*(ePuck_speed_right - ePuck_speed_left)/(2*EPUCK_RADIUS);

	    //Update position and angle based on each motor's measurements
	    ePuck_val_right = (right_motor_val - prev_right_motor_val)*STP2MM;
	    ePuck_val_left = (left_motor_val - prev_left_motor_val)*STP2MM;
	    ePuck_x_p += cos(ePuck_angle_p)*(ePuck_val_right + ePuck_val_left)/2;
	    ePuck_y_p += sin(ePuck_angle_p)*(ePuck_val_right + ePuck_val_left)/2;
	    ePuck_angle_p += (ePuck_val_right - ePuck_val_left)/(2*EPUCK_RADIUS);
	    prev_right_motor_val = right_motor_val;
	    prev_left_motor_val = left_motor_val;

	    //Reset the angles to 0 so we stay in +- 2*pi
	    if(ePuck_angle > 2*M_PI){
	    	ePuck_angle -= 2*M_PI;
	    }
	    else if(ePuck_angle_p > 2*M_PI){
	    	ePuck_angle_p -= 2*M_PI;
	    	circle_counter += 1;
	    }
	    else if(ePuck_angle < -2*M_PI){
	    	ePuck_angle += 2*M_PI;
	    }
	    else if(ePuck_angle_p < -2*M_PI){
	    	ePuck_angle_p += 2*M_PI;
	    	circle_counter += 1;
	    }

	    //REMOVE WHEN INTEGRATED
	    if(send_to_computer){
	    	chprintf((BaseSequentialStream *)&SD3, "\n rightmotorval=%-7d \r\n", right_motor_val);
	    	chprintf((BaseSequentialStream *)&SD3, "\n leftmotorval=%-7d \r\n", left_motor_val);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n rightmotorspeed=%-7d \r\n", ePuck_speed_right);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n leftmotorspeed=%-7d \r\n", ePuck_speed_left);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n Pos X=%.3f \r\n", ePuck_x);
	    	chprintf((BaseSequentialStream *)&SD3, "\n Pos_p X=%.3f \r\n", ePuck_x_p);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n Pos Y=%.3f \r\n", ePuck_y);
	    	chprintf((BaseSequentialStream *)&SD3, "\n Pos_p Y=%.3f \r\n", ePuck_y_p);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n angle=%.3f \r\n", ePuck_angle*RAD2DEG);
	    	chprintf((BaseSequentialStream *)&SD3, "\n angle_p=%.3f \r\n", ePuck_angle_p*RAD2DEG);
	    	//chprintf((BaseSequentialStream *)&SD3, "\n time elapsed=%.3f \r\n", (float) time_elapsed);
	    }
	    send_to_computer = !send_to_computer;
	    //prevTime = chVTGetSystemTime();

	    //To control the thread's frequency
	    time = chVTGetSystemTime();
	    //10Hz
	    chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}

//Get the ePuck's x position
float get_x_pos(void){
	return (ePuck_x_p);
}

//Get the ePuck's y position
float get_y_pos(void){
	return (ePuck_y_p);
}

//Get the ePuck's angle
float get_angle(void){
	return (ePuck_angle_p);
}

void odometry_start(void){
	chThdCreateStatic(waCompOdometry, sizeof(waCompOdometry), NORMALPRIO, CompOdometry, NULL);
}
