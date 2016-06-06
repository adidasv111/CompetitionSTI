//*** X represents lateral motion with positive numbers to the right and negative numbers to the left, 
//*** Y represents horizontal motions with positive numbers forward and negative numbers back-wards
//*** theta represents rotations in radians with 0 straight ahead, positive rotations to the right, and negative rotations to the left.
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

#define M_2PI 2.0*M_PI

#define leftEncA  2   //channel A is interrupt
#define leftEncB  43
#define rightEncA  3
#define rightEncB  42

#define WHEEL_BASE    0.3 // Distance between wheels of robot (m)
#define WHEEL_RADIUS  0.06  // Wheel radius (meters)
#define TICKS_PER_REV 3591.84/4 // Wheel radius (meters) (4 since using only rising edge on channel A)
#define TICKS_PER_M   TICKS_PER_REV/(M_2PI*WHEEL_RADIUS)  // Wheel radius (meters)


    void initOdometry();
    void calcOdometry();
    void doEncoderLeft();
    void doEncoderRight();
    
    extern float robotPosition [3]; // x, y, theta
     
#endif
