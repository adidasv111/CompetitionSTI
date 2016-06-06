#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Wire.h>

#define motorControllerBrate 115200

    void initMotors();
	  void setSpeeds(int speedLeft, int speedRight);
      void initMotors_I2C();
    void setSpeeds_I2C(int speedLeft, int speedRight);
 
#endif
