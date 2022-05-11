#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <motors.h>
#include <stdbool.h>
#include <math.h>
#include <leds.h>

#include <process_image.h>
#include <dumb_speed.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;

/* Extracts the line width
 * Input: buffer of the pixel's values
 * Output: width of the detected line
 */
uint16_t extract_line_width(uint8_t *buffer){
	uint16_t i = 0;
	uint16_t begin = 0;
	uint16_t end = 0;
	uint16_t width = 0;
	uint8_t stop = 0;
	uint8_t wrong_line = 0;
	uint8_t line_not_found = 0;
	uint32_t mean = 0;

	//PXTOCM is pixel to centimeter conversion
	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//Computing the mean of the pixels' values
	for(uint32_t i = 0; i< IMAGE_BUFFER_SIZE; i++){
		mean += buffer[i];
	}
	//Computing the average
	mean /= IMAGE_BUFFER_SIZE;

	//no need to reset i as it is done in the first loo of the function (TO DOUBLE CHECK)

	do{
		wrong_line = 0;

		//First find beginning of line
		//Need to have a rising pixel
		while(stop == 0 && i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
			if(buffer[i]>mean && buffer[i+WIDTH_SLOPE]<mean){ //why <mean ? it should be a rising slope ?
				begin = i;
				stop = i;
			}
			i++;
		}
		//Once beginning of line is found, find an end
		if(i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin){
			stop = 0;

			//search for the end of the line
			while(stop == 0 && i < IMAGE_BUFFER_SIZE){
				if(buffer[i]>mean && buffer[i-WIDTH_SLOPE]<mean){
					end = i;
					stop = i;
				}
				i++;
			}
			//If beginning found but no end
			if (i> IMAGE_BUFFER_SIZE || !end){
				line_not_found = 1;
			}
		}
		//if not ready to begin (i.e. first iteration)
		else{
			line_not_found =1;
		}

		//If found a line and its width is coherent
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	//No line --> send last found width
	if(line_not_found){
		begin =0;
		end = 0;
		width = last_width;
	}
	//The width of the line found
	else{
		last_width = width = (end-begin);
		line_position = (begin+end)/2;
	}

	//We consider there is a maximal distance to which the measurements are valid
	if((PXTOCM/width)>MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}
	else{
		return width;
	}
}

//The thread that captures the image
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static uint16_t lineWidth_red = 0;

//Thread that processes the image
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	//for now i'm reading the three colors but only one will be used, TBD
	static uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Reading the three colors
		for(int i=0; i<2*IMAGE_BUFFER_SIZE; i+=2){
			image_red[i/2] = img_buff_ptr[i]&0xF8;
			/*if(i+1<(2*IMAGE_BUFFER_SIZE)){
				image_blue[i/2] = img_buff_ptr[i+1]&0x1F;
				image_green[i/2] = ((img_buff_ptr[i]&0x07)<<5) | ((img_buff_ptr[i+1]&0xE0)>>5);
			}*/
		}

		//Computing line width for each color
		lineWidth_red = extract_line_width(image_red);

		//I'll first use the red line as it seems reliable
		//converts the width into a distance between the robot and the camera
		if(lineWidth_red){
			distance_cm = PXTOCM/lineWidth_red;
		}

		//Do something if we are close enough from the line
		/*if(distance_cm < DIST_THRESH){
			set_body_led(1); //turns on body LEDs
		}
		else{
			set_body_led(0); //turns off body LEDs
		}*/

		//sends the data buffer of the given size to the computer
		if(send_to_computer){
			//SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);

			//Sending the line width to the computer
			//chprintf((BaseSequentialStream *)&SD3, "\n lineWidth_red=%-7d \r\n", lineWidth_red);
		}
		send_to_computer = !send_to_computer;

    }
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_Linewidth(void){
	return lineWidth_red;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint16_t get_line_position(void){
	return line_position;
}
