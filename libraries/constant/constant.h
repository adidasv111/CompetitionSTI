#ifndef CONSTANT_H
#define CONSTANT_H
 
#include <Arduino.h>

#define RECYCLE_ZONE_X  0
#define RECYCLE_ZONE_Y  0

#define SPEED_UNIT_RADS	255

#define WHEEL_BASE    300 // Distance between wheels of robot (mm)
#define WHEEL_RADIUS  60  // Wheel radius (mm)
#define TICKS_PER_REV 3591.84/4 //4 since using only rising edge on channel A
#define TICKS_PER_MM   TICKS_PER_REV/(M_2PI*WHEEL_RADIUS)  // Wheel radius (mm)

#define EMPTY 0
#define ROBOT 1
#define PET 2
#define TARGET 3

 
#endif
