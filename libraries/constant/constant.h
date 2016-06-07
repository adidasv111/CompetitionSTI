#ifndef CONSTANT_H
#define CONSTANT_H

#include <Arduino.h>
#define M_2PI 2.0*M_PI

#define RECYCLE_ZONE_X  0.5
#define RECYCLE_ZONE_Y  0.5

#define SPEED_UNIT_RADS	255

//--- Motors and odometry ---
#define COMPASS_WEIGHT 0.75		// Weight for compass in yaw estimation
#define WHEEL_BASE    0.41 		// Distance between wheels of robot (m)
#define WHEEL_RADIUS  0.065  	// Wheel radius (meters)
#define TICKS_PER_REV 3591.84/4 // Wheel radius (meters) (4 since using only rising edge on channel A)
#define TICKS_PER_M   TICKS_PER_REV/(M_2PI*WHEEL_RADIUS)  // Wheel radius (meters)

//--- Dynamixels ---
//Pusher
#define PUSHER_TIME     600                 //4.896 sec
#define PUSHER_INIT_POS 800
#define PUSHER_SPEED	1000
//Door
#define DOOR_INIT_POS	75
#define DOOR_END_POS	250
#define DOOR_CLOSE_POS	400
#define DOOR_SPEED		300
#define DOOR_HALF_PERIOD 500				//in ms


#define FULL_THRESHOLD	5			//number of times it is full to be considered full

#define EMPTY 0
#define ROBOT 1
#define PET 2
#define TARGET 3


#endif
