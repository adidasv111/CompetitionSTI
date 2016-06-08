#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <mapping.h>
#include <Wire.h>
#include <constant.h>

#define motorControllerBrate 115200

    void initMotors();
	void setSpeeds(int speedLeft, int speedRight);
	void initMotors_I2C();
    void setSpeeds_I2C(int speedLeft, int speedRight);
	void compute_wheel_speeds_coord(coord target, int *msl, int *msr);
	void compute_wheel_speeds(float target_range, float target_bearing, int *msl, int *msr);
 
#endif
