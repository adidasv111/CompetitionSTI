#ifndef MISC_H
#define MISC_H

#include <Arduino.h>
#include <constant.h>
#include <US_Sensor.h>
#include <Pusher.h>

void checkFull();			//check if container is full
void get_info_from_pi(float *bottle_x, float *bottle_y, bool *communicate);
void deposition(int* left_speed, int* right_speed);

extern bool isFull;

#endif