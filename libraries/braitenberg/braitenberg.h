#ifndef BRAITENBERG_H
#define BRAITENBERG_H
 
#include <Arduino.h>

void updateIRSensors();
void obstacle_avoidance(int* left_speed, int* right_speed);
void calibration (float *robotPose, char side, char wall);
int checkObstacle(int *blockedFlag);

#endif
