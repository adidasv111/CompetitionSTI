#include "Misc.h"

char fullCounter = 0;
bool isFull = false;

//Ultrasound sensor to detect bottles that enter the robot
USSensor US_detection(52, 53);

void checkFull()
{
  int detector_dist = US_detection.calc_distanceUS();
  if (detector_dist < 15)
    fullCounter++;

  if (fullCounter >= FULL_THRESHOLD)
  {
    fullCounter = 0;
    isFull = true;
  }
  else
  {
    isFull = false;
  }
}


//Communicates with PI
void get_info_from_pi(float *bottle_x, float *bottle_y, bool *communicate)
{

}

void deposition(int* left_speed, int* right_speed)
{
  DymxPusher_EmptyBottles();
  *left_speed = 255;
  *right_speed = -255;
}