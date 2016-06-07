#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <Wire.h>

#define ADDRESS_COMPASS 0xE0

void initCompass_Serial2();
float readCompass_Serial2();
void calibrateCompass_Serial2();

float readCompass_I2C();
    
extern float yawCompass;		//in rad
     
#endif
