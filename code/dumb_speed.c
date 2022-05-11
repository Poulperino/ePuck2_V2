#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <dumb_speed.h>
#include <odometry.h>

static int16_t right_speed = 100;
static int16_t left_speed = 100;
static int16_t counter = 0;

//Speed is in step/s. There are 20 steps for one motor turn.
//Reductor of 50:1 --> 1000 steps for 1 wheel turn
//Wheels dia = 41mm --> one wheel turn is 128mm
//step/s to mm/s conversion: SPEED_CONV = 128/1000 = 0.128;

void go_straight(void){
	/*if(counter < 300){
		left_speed += 1;
		right_speed += 1;
	}
	else{
		left_speed = 400;
		right_speed = 400;
	}*/
	left_speed = 400;
	right_speed = 400;
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
	//counter++;
}

void go_circle_ccw(void){
	left_speed = 200;
	right_speed = 600;
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}

void go_circle_cw(void){
	left_speed = 600;
	right_speed = 200;
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}

void stop_moving(void){
	left_speed = 0;
	right_speed = 0;
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}

//Get the motor's speed in mm/s
int16_t get_right_speed_mms(void){
	return (right_speed*SPEED_CONV);
}

int16_t get_left_speed_mms(void){
	return (left_speed*SPEED_CONV);
}


