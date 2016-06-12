#ifndef MISC_H
#define MISC_H

#include <Arduino.h>
#include <constant.h>
#include <Ultrasonic.h>
#include <Pusher.h>

void checkFull();			//check if container is full

extern bool isFull;

#endif