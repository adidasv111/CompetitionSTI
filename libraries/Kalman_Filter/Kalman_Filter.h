#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
 
#include <Arduino.h>

typedef struct k_state kalman_state;

kalman_state kalman_init(double q, double r, double p, double intial_value);
void kalman_update(kalman_state* state, double measurement);
 
#endif
