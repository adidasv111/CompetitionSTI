#include "Misc.h"

char fullCounter = 0;
bool isFull = false;

//Ultrasound sensor to detect bottles that enter the robot
Ultrasonic US_detection(52);

void checkFull()
{
  int detector_dist = 0;
  int i;
  for (i = 0; i < FULL_N; i++)
  {
	  if(US_detection.MeasureInCentimeters() > 15)
		  break;
    detector_dist += US_detection.MeasureInCentimeters();    //****
  }
  
  detector_dist /= i;

  if (detector_dist < 15 && i == FULL_N)
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