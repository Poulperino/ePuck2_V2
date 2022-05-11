#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define GOAL_DISTANCE 10.0f
#define WIDTH_SLOPE 5
#define MIN_LINE_WIDTH 40
#define MAX_DISTANCE 25.0f
#define PXTOCM 1570.0f
#define ERROR_THRESHOLD 0.1f
#define KP 800.0f
#define KI 3.5f
#define ROTATION_COEFF 2
#define ROTATION_THRESHOLD 10
#define MAX_SUM_ERROR (MOTOR_SPEED_LIMIT/KI)
#define EPUCK_RADIUS 26.5 //[mm]
//#define WHEEL_DIA 41 //[mm]
//#define WHEEL_DIA 40.596 //[mm]
#define WHEEL_DIA 45 //[mm]
#define M_PI 3.14159265358979323846
#define STEPPERTURN 1000
#define SPEED_CONV ((M_PI*WHEEL_DIA)/STEPPERTURN) //from step/s to mm/s
#define STP2MM ((M_PI*WHEEL_DIA)/STEPPERTURN) //from step to mm
#define MS2S 0.001  //from ms to s
#define RAD2DEG 57.2958 //from rad to deg
#define DIST_THRESH 5 //distance threshold from line --> do something

//Thresholds for the walls of the box (remove width of ePuck + 1cm)
#define EAST_WALL 190 //[mm]
#define WEST_WALL (-190) //[mm]
#define NORTH_WALL 190 //[mm]
#define SOUTH_WALL (-190) //[mm]


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
