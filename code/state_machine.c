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
#include <process_image.h>
#include <leds.h>
#include <pi_regulator.h>
#include <avoidance.h>

static uint8_t state = 1;

//The thread that computes the odometry
static THD_WORKING_AREA(waStateMachine, 256);
static THD_FUNCTION(StateMachine, arg) {

	//For the odometry
	static float ePuck_x = 0;
	static float ePuck_y = 0;
	static float ePuck_angle = 0;

	//For the camera
	static uint16_t lineWidth = 0;

	//For the PID:
	int16_t speed = 0;
	int16_t speed_correction = 0;
	float angle_goal = 90;

    //To control the thread's frequency
    systime_t time;
    time = chVTGetSystemTime();

	while(1){
		//I'll do the computations in the switch case
		switch (state) {
		 case 1: //searching for a wall while local navigation
			 	 //Setting the speed of the motors from avoidance
			 	 left_motor_set_speed(avoidance_left_speed());
			 	 right_motor_set_speed(avoidance_right_speed());
			 	 //chprintf((BaseSequentialStream *)&SD3, "\n in state 1 \r\n");
			 	 //getting the position of the robot
			 	 ePuck_x = get_x_pos();
			 	 ePuck_y = get_y_pos();
			 	 //if that position is between certain thresholds (defined in main.h)
			 	 //Does it need to detect the wall actually ? Yes for the positioning
			 	 state = 1;
			 	 if(ePuck_x>EAST_WALL || ePuck_x<WEST_WALL || ePuck_y>NORTH_WALL || ePuck_y<SOUTH_WALL){
			 		set_body_led(1); //turns on body LEDs
			 		//stop_moving();
			 		state = 5; //Because there is nothing after but to be removed
			 	 }
		         break;
		 case 2: //Found a wall, adjust its position
			 	 //Lets say it adjusts its angle to go into a right angle direction
			 	 /*ePuck_angle = get_angle();
			 	 //if()
			         speed = pi_regulator(ePuck_angle, angle_goal);
			         //speed_correction = (get_line_position()-(IMAGE_BUFFER_SIZE/2));

			         if(abs(speed_correction) < ROTATION_THRESHOLD){
			         	speed_correction = 0;
			         }

			         //applies the speed from the PI regulator
			 		right_motor_set_speed((speed - ROTATION_COEFF * speed_correction));
			 		left_motor_set_speed((speed + ROTATION_COEFF*speed_correction));*/
		         break;
		 case 3: //Adjusted its position, follows the wall to the right to find a corner
		         break;
		 case 4: //Found a corner, adjust its position
		         break;
		 case 5: //Line detection and line action. For now just get line width and maybe count lines later
			 	 lineWidth = get_Linewidth();
			 	 chprintf((BaseSequentialStream *)&SD3, "\n lineWidth %d \r\n", lineWidth);
			 	 if(lineWidth < 250){
			 		set_led(LED1,0);
			 		set_led(LED3,0);
			 		set_led(LED5,0);
			 	 }
			 	 else if(lineWidth < 350){
			 		set_led(LED1, 1);
			 	 }
			 	 else if(lineWidth < 420){
			 		set_led(LED1, 1);
			 		set_led(LED3, 1);
			 	 }
			 	 else if (lineWidth > 420){
			 		 set_led(LED1,1);
			 		 set_led(LED3,1);
			 		 set_led(LED5,1);
			 	 }
			 	 else{
			 		 set_led(LED1,0);
			 		 set_led(LED3,0);
			 	 }
		         break;
		 case 6: //New target ?
		         break;
		 case 7: //New target ?
		         break;
		 case 0: //When the robot didn't start to move. It does nothing
			 	 break;
		 default: break;
		}

	    //To control the thread's frequency
	    time = chVTGetSystemTime();
	    //10Hz
	    chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}

uint8_t get_state(void){
	return (state);
}


void state_machine_start(void){
	chThdCreateStatic(waStateMachine, sizeof(waStateMachine), NORMALPRIO, StateMachine, NULL);
}
