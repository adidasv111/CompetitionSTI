#include "Misc.h"

char fullCounter = 0;
bool isFull = false;

//Ultrasound sensor to detect bottles that enter the robot
USSensor US_detection(52, 53);

void checkFull()
{
  int detector_dist = 0;
  for (int i = 0; i < FULL_N; i++)
  {
    detector_dist += US_detection.calc_distanceUS();    //****
  }
  detector_dist /= FULL_N;

  if (detector_dist < 15)
    fullCounter++;

  if (fullCounter >= FULL_THRESHOLD)
  {
    isFull = true;
    fullCounter = 0;
  }
  else
  {
    isFull = false;
  }

  isFull = false; //*******
}