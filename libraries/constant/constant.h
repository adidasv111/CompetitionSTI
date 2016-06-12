#ifndef CONSTANT_H
#define CONSTANT_H

#include <Arduino.h>

#define M_2PI			2.0*M_PI
#define M_PI2			0.5*M_PI

#define ERROR 			-1

#define filterSamples   13              // filterSamples should  be an odd number, no smaller than 3

//--- States for machine state ---
#define GOING_TO_WAYPOINT 		0
#define GOING_TO_BOTTLE			1
#define GOING_HOME		2
#define DEPOSITION		3

//--- Mapping ---
#define HOME_X  500
#define HOME_Y  500

#define EMPTY 0
#define BOTTLE 1
#define TARGET 2

#define NB_WAYPOINTS 26
#define NB_BOTTLES_ARRAY	30

//--- Motors ---
#define MS_OFFSET	22	//Substract from left speed, Right motors turn slowers

#define Kmotors_plus	35.0
#define Kmotors_minus	10*Kmotors_plus
#define DIST_GOAL_THRESH	150
#define BIG_DIST_GOAL_THRESH	500
#define BEARING_GOAL_TRESH	0.15
#define SPEED_UNIT_RADS	0.0137

//--- Odometry ---
#define INIT_X			500
#define INIT_Y			500
#define INIT_THETA			0
#define COMPASS_WEIGHT 0.75		// Weight for compass in yaw estimation
#define WHEEL_BASE    0.45 		// Distance between wheels of robot (m)
#define WHEEL_RADIUS  0.062  	// Wheel radius (meters)
#define TICKS_PER_REV 3591.84/4 // Wheel radius (meters) (4 since using only rising edge on channel A)
//#define TICKS_PER_REV 8245.92/4 // Wheel radius (meters) (4 since using only rising edge on channel A)
#define TICKS_PER_M   TICKS_PER_REV/(M_2PI*WHEEL_RADIUS)  // Wheel radius (meters)

//--- Dynamixels ---
//Pusher
#define PUSHER_TIME     600
#define PUSHER_INIT_POS 800
#define PUSHER_SPEED	1000
#define PUSHER_HALF_PERIOD 10000				//in ms
#define PUSHER_RESET_PERIOD	25000
//Door
#define DOOR_INIT_POS	75 	
#define DOOR_END_POS	250
#define DOOR_CLOSE_POS	470
#define DOOR_SPEED		300
#define DOOR_HALF_PERIOD 750				//in ms

#define DOOR_OPEN		0
#define DOOR_CLOSE		1
#define DOOR_MOVE		2

//--- Sensors ---
#define N_SENSORS 8
#define OBS_THRESH	40
#define CRIT_OBS_THRESH	15
#define WALL_THRESH	12
#define KplusIR 57
#define KminusIR 150//4*110

#define WALL_CAL_THRESH	40
#define SENSOR_SEPERATION	21		//In centimeters, the distance between the right and left sensors
#define ROBOT_LEFT	0
#define ROBOT_RIGHT	1
#define ROBOT_BACk	2
#define	WALL_BOTTOM	0
#define	WALL_RIGHT 	1
#define	WALL_LEFT 	2
#define	WALL_TOP 	3

#define FULL_DELAY	60000
#define FULL_THRESHOLD	5			//number of times it is full to be considered full
#define FULL_N		5

#define EVASIVE_MANOEUVRE_DELAY	5
#define ITS_TOO_LATE_INT	570000
#define ITS_BEEN_TOO_LONG_INT	180000

#endif