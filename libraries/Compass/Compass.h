#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <Wire.h>

#define ADDRESS 0xE0
#define M_2PI 2.0*M_PI

void initCompass_Serial2();
float readCompass_Serial2();
void calibrateCompass_Serial2();

float readCompass_I2C();
    
extern float yawCompass;		//in rad
     
#endif
