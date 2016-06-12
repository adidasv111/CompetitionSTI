#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <mapping.h>
#include <Wire.h>
#include <constant.h>
#include <braitenberg.h>

#define motorControllerBrate 115200

void initMotors();
void setSpeeds(int speedLeft, int speedRight);
void initMotors_I2C();
void setSpeeds_I2C(int speedLeft, int speedRight);
void compute_wheel_speeds(float target_range, float target_bearing, int *msl, int *msr);
void compute_bottle_speeds_coord(float* position, coord target, int *msl, int *msr, char robotState);
void compute_waypoint_speeds_coord(float* position, coord target, int *msl, int *msr, char robotState);
void check_goal(float* position, coord target, char robotState, int* calibrationFlag);

extern char gotWaypoint;	//0 - not active, 1 - almost waypoint, 2 - got to waypoint
extern bool gotBottle;
extern bool gotHome;
#endif
