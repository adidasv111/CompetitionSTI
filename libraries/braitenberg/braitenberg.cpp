/**
    C++
    braitenberg.cpp
    Purpose: Perform obstacle avoidance manoeuvres

    @author Sean Thomas
    @version 1.0 3/6/16 
*/

#include "braitenberg.h"
#include "US_Sensor.h"

#define N_SENSORS 8

//double l_weight[N_SENSORS] = {0, 0, 0, 0, 20, 50, -50, -20};
//double r_weight[N_SENSORS] = {0, 0, 0, 0, -20, -50, 50, 20};

double l_weight[N_SENSORS] = {0, 0, 0, 0, 20, 50, -50, -20};
double r_weight[N_SENSORS] = {0, 0, 0, 0, -20, -50, 50, 20};

int us_value[N_SENSORS];

//US(Trig, Echo)
USSensor US1(44,45), US2(46,47), US3(48,49), US4(50,51), US5(40,41), US6(38,39), US7(36,37), US8(34,35);
//USSensor US8(34,35);
//Left Sensors: US5, US6, US4, US3
//Right Sensors: US8, US7, US1, US2


/** Changes the left and right wheel speeds to perform obstacle
	avoidance manoeuvres
	
	@param left_speed Speed of the left wheels
	@param right_speed Speed of the right wheels
	@return void
*/

void obstacle_avoidance(int* left_speed, int* right_speed)
{

  us_value[0] = US1.calc_distanceUS();
  us_value[1] = US2.calc_distanceUS();
  us_value[2] = US3.calc_distanceUS();
  us_value[3] = US4.calc_distanceUS();
  us_value[4] = US5.calc_distanceUS();
  us_value[5] = US6.calc_distanceUS();
  us_value[6] = US7.calc_distanceUS();
  us_value[7] = US8.calc_distanceUS();
	/*Serial.print(us_value[6]);
			Serial.print("US7 ");
	Serial.print(us_value[7]);
			Serial.print("US8 ");
			Serial.println(" ");*/
  int i;
  for (i = 0; i < N_SENSORS; i++)
  {
	  Serial.print(us_value[i]);
			Serial.print(" ");
	  if(us_value[i] < 50)
	  {
		*left_speed += (l_weight[i])*((50 - us_value[i])/50.0f);
		*right_speed += (r_weight[i])*((50 - us_value[i])/50.0f);
		/*Serial.print(*left_speed);
			Serial.print("   ");
				Serial.println(" ");*/
	  }
	  
  }
  Serial.println(" ");
  

  
}





