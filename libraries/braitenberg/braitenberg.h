#ifndef BRAITENBERG_H
#define BRAITENBERG_H
 
#include <Arduino.h>
#include <Motors.h>

void updateIRSensors();
void obstacle_avoidance(int* left_speed, int* right_speed);
void calibration (float *robotPose, char side, char wall);
int checkObstacle();
void checkEvasiveManoeuvre(int *blockedFlag);

#endif
