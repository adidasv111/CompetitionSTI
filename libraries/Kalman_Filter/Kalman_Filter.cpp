#include "Kalman_Filter.h"

/*
x = x
p = p + q;

k = p / (p + r);
x = x + k * (measurement – x);
p = (1 – k) * p;

The first two formulas represent the prediction of the Kalman Filter. 
And since there is no information about the driving forces it is very simple. 
The second three formulas calculate the measurement update. The variables are x for the filtered value, 
q for the process noise, r for the sensor noise, p for the estimated error and k for the Kalman Gain. 
The state of the filter is defined by the values of these variables.

The filter is applied with each measurement and initialized with the process noise q, 
the sensor noise r, the initial estimated error p and the initial value x. 
The initial values for p is not very important since it is adjusted during the process. 
It must be just high enough to narrow down. The initial value for the readout is also not very important, 
since it is updated during the process.

*/
struct k_state{
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
};

kalman_state kalman_init(double q, double r, double p, double intial_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = intial_value;

  return result;
}

void kalman_update(kalman_state* state, double measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k) * state->p;
}


