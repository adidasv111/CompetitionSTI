/**
    C++
    braitenberg.cpp
    Purpose: Perform obstacle avoidance manoeuvres

    @author Sean Thomas
    @version 1.0 3/6/16 
*/

#include "braitenberg.h"
#include "IR_Sensor.h"
#include "US_Sensor.h"
#include "constant.h"
#include "math.h"






#define KplusIR 57
#define KminusIR 110

//double l_weight[N_SENSORS] = {0, 0, 0, 0, 20, 50, -50, -20};
//double r_weight[N_SENSORS] = {0, 0, 0, 0, -20, -50, 50, 20};

//double l_weight[N_SENSORS] = {0, 0, 0, 0, 50, 100, -100, -50};
//double r_weight[N_SENSORS] = {0, 0, 0, 0, -50, -100, 100, 50};

int IRValue[N_SENSORS];

//US(Trig, Echo)
//USSensor US1(44,45), US2(46,47), US3(48,49), US4(50,51), US5(40,41), US6(38,39), US7(36,37), US8(34,35);
//USSensor US8(34,35);
//Left Sensors: US5, US6, US4, US3
//Right Sensors: US8, US7, US1, US2

IRSensor IR_FRR(A6), IR_FR(A1), IR_FLL(A2), IR_FL(A3), IR_L1(A4), IR_L2(A5), IR_R1(A0), IR_R2(A7);

/** Update all the distance sensors
	
	@return void
*/

void updateIRSensors()
{
	IRValue[0] = IR_FRR.calc_distanceIR();
	IRValue[1] = IR_FR.calc_distanceIR();
	IRValue[2] = IR_FLL.calc_distanceIR();
	IRValue[3] = IR_FL.calc_distanceIR();
	IRValue[4] = IR_L1.calc_distanceIR();
	IRValue[5] = IR_L2.calc_distanceIR();
	IRValue[6] = IR_R1.calc_distanceIR();
	IRValue[7] = IR_R2.calc_distanceIR();
	
	/*for(int i = 0; i < 8; i++)
	{
		Serial.print(IRValue[i]);
		Serial.print(" ");
		
	}
	Serial.println(" ");
	*/	/*Serial.print("IR Right");
			Serial.print(IRValue[0]);
			Serial.println(" ");
			Serial.print("IR Left");
			Serial.print(IRValue[2]);
			Serial.println(" ");*/
}


/** Changes the left and right wheel speeds to perform obstacle
	avoidance manoeuvres
	
	@param left_speed Speed of the left wheels
	@param right_speed Speed of the right wheels
	@return void
*/

void obstacle_avoidance(int* left_speed, int* right_speed)
{
	
	if(IRValue[0] < OBS_THRESH)
	{
		if(IRValue[0] < CRIT_OBS_THRESH)
		{
			*right_speed += (KplusIR/2.0f)*(80 - IRValue[0])/80.0f;
			*left_speed = -1*(*right_speed);
		}
		else
		{
		*right_speed += KplusIR*(80 - IRValue[0])/80.0f;
		*left_speed -= 5*KminusIR*(80 - IRValue[0])/80.0f;
		}
	}
	else if(IRValue[2] < OBS_THRESH)
	{
		if(IRValue[2] < CRIT_OBS_THRESH)
		{
			*left_speed += (KplusIR/2.2f)*(80 - IRValue[2])/80.0f;
			*right_speed = -1*(*left_speed);
		}
		else
		{
			*right_speed -= 5*KminusIR*(80 - IRValue[2])/80.0f;
			*left_speed += KplusIR*(80 - IRValue[2])/80.0f;
		}
	}
	else if(IRValue[1] < OBS_THRESH)
	{
		if(IRValue[1] < CRIT_OBS_THRESH)
		{
			*right_speed += (KplusIR/2.0f)*(80 - IRValue[1])/80.0f;
			*left_speed = -1*(*right_speed);
		}
		else
		{
		*right_speed += KplusIR*(80 - IRValue[1])/80.0f;
		*left_speed -= 5*KminusIR*(80 - IRValue[1])/80.0f;
		}
	}
	else if(IRValue[3] < OBS_THRESH)
	{
		if(IRValue[3] < CRIT_OBS_THRESH)
		{
			*left_speed += (KplusIR/2.2f)*(80 - IRValue[3])/80.0f;
			*right_speed = -1*(*left_speed);
		}
		else
		{
			*right_speed -= 5*KminusIR*(80 - IRValue[3])/80.0f;
			*left_speed += KplusIR*(80 - IRValue[3])/80.0f;
		}
	}
	
	/******** Wall avoidance ****************/
	if(IRValue[4] < WALL_THRESH || IRValue[5] < WALL_THRESH)
	{
		*right_speed -= 15;
	}
	
	if(IRValue[6] < WALL_THRESH || IRValue[7] < WALL_THRESH)
	{
		*left_speed -= 15;
	}
	
}


/*void obstacle_avoidance(int* left_speed, int* right_speed)
{

  us_value[0] = US1.calc_distanceUS();
  us_value[1] = US2.calc_distanceUS();
  us_value[2] = US3.calc_distanceUS();
  us_value[3] = US4.calc_distanceUS();
  us_value[4] = US5.calc_distanceUS();
  us_value[5] = US6.calc_distanceUS();
  us_value[6] = US7.calc_distanceUS();
  us_value[7] = US8.calc_distanceUS();
	Serial.print(us_value[6]);
			Serial.print("US7 ");
	Serial.print(us_value[7]);
			Serial.print("US8 ");
			Serial.println(" ");
  int i;
  for (i = 0; i < N_SENSORS; i++)
  {
	  Serial.print(us_value[i]);
			Serial.print(" ");
	  if(us_value[i] < 30)
	  {
		*left_speed += (l_weight[i])*((50 - us_value[i])/50.0f);
		*right_speed += (r_weight[i])*((50 - us_value[i])/50.0f);
		Serial.print(*left_speed);
			Serial.print("   ");
				Serial.println(" ");
	  }
	  
  }
  Serial.println(" ");
}*/


/** Calibrates the orientation of the robot using
	IR sensors 
	
	@param robotPose Pose of the robot
	@param side Location of the sensors on the robot
	@param wall Location of the wall
	@return void
*/
void calibration (float *robotPose, char side, char wall)
{
	float theta = 0;

	switch(side)
	{
		case ROBOT_LEFT:
			if(IRValue[4] < WALL_CAL_THRESH && IRValue[5] < WALL_CAL_THRESH)
			{
				theta = atan2((IRValue[5] - IRValue[4]),SENSOR_SEPERATION);
			}
			if(wall == WALL_BOTTOM)
				theta -= M_PI;
			else if(wall == WALL_RIGHT)
				theta += -M_PI;
			else if(wall == WALL_TOP)
				theta += -M_PI2;
			break;
		case ROBOT_RIGHT:
			if(IRValue[6] < WALL_CAL_THRESH && IRValue[7] < WALL_CAL_THRESH)
			{
				theta = atan2((IRValue[6] - IRValue[7]),SENSOR_SEPERATION);
			}
			if(wall == WALL_BOTTOM)
				theta += -M_PI2;
			else if(wall == WALL_TOP)
				theta += M_PI2;
			else if(wall == WALL_LEFT)
				theta += -M_PI;
			break;
	}
	robotPose[2] = 0.3*robotPose[2] + 0.7*theta;
	//robotPose[2] = theta;
}