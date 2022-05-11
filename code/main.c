#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include <i2c_bus.h>
#include <arm_math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h> //To control the motors
#include <camera/po8030.h> //To use the camera
#include <chprintf.h> //To communicate with the computer
#include <leds.h>

#include <process_image.h> //Process image from the camera
#include <odometry.h> //Odometry
#include <avoidance.h>
#include <sensors/proximity.h>
#include <state_machine.h>

//#define SEND_DIST

//messagebus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();  //General low level initialization
    chSysInit(); //ChibiOS initialization
    mpu_init(); //MPU initialization

    serial_start(); //Starts serial communication
    usb_start(); //Starts usb communication
    dcmi_start(); //Starts the camera
	po8030_start(); //Starts the camera
	i2c_start();
	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	motors_init(); //motors initialization

	//stars the threads for the pi regulator and the processing of the image
	odometry_start();
	state_machine_start();
	avoidance_start();
	bool corner = true;
	if (corner){
		process_image_start();
	}

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
