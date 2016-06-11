/**
    C++
    Motors.cpp
    Purpose: Manage and initialze motors to follow targets

    @author Adi Vardi
    @version 1.0 3/6/16 
*/

#include "Motors.h"
// mode: 0=reverse, 1=brake, 2=forward
// PWM: PWM value for right motor speed / brake

    bool gotBottle = false;
    bool gotHome = false;
/** Initialize the motors and set wheel speeds to 0
	
	@return void
*/
	void initMotors()
	{
		Serial3.begin(motorControllerBrate);
		setSpeeds(0, 0);
	}

/** Set the values of the wheel speeds
	
	@param left_speed Speed of the left wheels
	@param right_speed Speed of the right wheels
	@return void
*/
	void setSpeeds(int speedLeft, int speedRight)
	{
		if(speedLeft > 255)
			speedLeft = 255;
		else if(speedLeft < -255)
			speedLeft = -255;
		
		if(speedRight > 255)
			speedRight = 255;
		else if(speedRight < -255)
			speedRight = -255;
  // Serial3.write("HB");
  unsigned int LeftMode = (speedLeft > 0) - (speedLeft < 0) + 1;	//returns positive->2, zero->1, negative->0
  unsigned int LeftPWM = abs(speedLeft);

  unsigned int RightMode = (speedRight > 0) - (speedRight < 0) + 1;	//returns positive->2, zero->1, negative->0
  unsigned int RightPWM = abs(speedRight);

  if (speedLeft == 0)
  	LeftPWM = 255;
  if (speedRight == 0)
  	RightPWM = 255;

  Serial3.write(LeftMode);
  Serial3.write(LeftPWM);
  Serial3.write(RightMode);
  Serial3.write(RightPWM);
}


void initMotors_I2C()
{
	Wire.begin();
	setSpeeds_I2C(0, 0);
}


void setSpeeds_I2C(int speedLeft, int speedRight)
{
	if (speedLeft != 0)
	{
		speedLeft -= MS_OFFSET;
	}
	
	if (speedLeft >= 255)
		speedLeft = 255;
	else if (speedLeft <= -255)
		speedLeft = -255;
	if (speedRight >= 255)
		speedRight = 255;
	else if (speedRight <= -255)
		speedRight = -255;

  unsigned int LeftMode = (speedLeft > 0) - (speedLeft < 0) + 1;    //returns positive->2, zero->1, negative->0
  unsigned int LeftPWM = abs(speedLeft);

  unsigned int RightMode = (speedRight > 0) - (speedRight < 0) + 1; //returns positive->2, zero->1, negative->0
  unsigned int RightPWM = abs(speedRight);

  if (speedLeft == 0)
  	LeftPWM = 255;
  if (speedRight == 0)
  	RightPWM = 255;

  Wire.beginTransmission(1);
  Wire.write(LeftMode);
  Wire.write(LeftPWM);
  Wire.write(RightMode);
  Wire.write(RightPWM);
  Wire.endTransmission();
}


void compute_bottle_speeds_coord(float* position, coord target, int *msl, int *msr, char robotState)
{
	if ((target.x >= 0 && target.x <= 8000) && (target.y >= 0 && target.y <= 8000))
	{
		float Ebearing = -(M_PI2 - atan2(target.y-position[1], (target.x-position[0])));
		if (Ebearing > M_PI)
			Ebearing -= M_2PI;
		if (Ebearing < -M_PI)
			Ebearing += M_2PI;

		Ebearing -= position[2];

		if (Ebearing > M_PI)
			Ebearing -= M_2PI;
		if (Ebearing < -M_PI)
			Ebearing += M_2PI;

		if (Ebearing > 0 && Ebearing < M_PI2)
		{
			*msl -= Kmotors_minus*abs(Ebearing);
			*msr += Kmotors_plus*abs(Ebearing);
		}
		else if (Ebearing < 0 && Ebearing > -M_PI2)
		{
			*msl += Kmotors_plus*abs(Ebearing);
			*msr -= Kmotors_minus*abs(Ebearing);
		}	
		else if (Ebearing > M_PI2)
		{
			*msl = -255;
			*msr = 255;
		}
		else if (Ebearing < -M_PI2)
		{
			*msl = 255;
			*msr = -255;
		}

		float Erange = sqrtf((target.x-position[0])*(target.x-position[0]) + (target.y-position[1])*(target.y-position[1]));
		if (Erange < DIST_GOAL_THRESH)
		{
			if(robotState == GOING_TO_BOTTLE)		//just captured a bottle
			{
				gotBottle = true;
				removeTarget();
				Serial.println("*****************************************");
				Serial.print("George picked bottle at: ");
				Serial.print(target.x);
				Serial.print(" , ");
				Serial.println(target.y);
				Serial.println("*****************************************");
			}
			else if (robotState == GOING_HOME)
			{
				gotHome = true;
				Serial.println("*****************************************");
				Serial.println("George is home, bitches!");
				Serial.println("*****************************************");
			}
			else if (robotState == GOING_TO_WAYPOINT)
			{
				Serial.println("*****************************************");
				Serial.print("I'm in waypoint number  ");
				Serial.print((int)currentWaypoint);
				Serial.println("		--George");
				Serial.println("*****************************************");
				switch(currentWaypoint)
				{
					case 0:
					calibration(position, ROBOT_LEFT, WALL_LEFT);
					break;
					case 1:
					calibration(position, ROBOT_LEFT, WALL_LEFT);
					break;
					case 8:
					calibration(position, ROBOT_LEFT, WALL_TOP);
					break;
					case 11:
					calibration(position, ROBOT_LEFT, WALL_RIGHT);
					break;
					case 15:
					calibration(position, ROBOT_LEFT, WALL_RIGHT);
					break;
					case 18:
					calibration(position, ROBOT_RIGHT, WALL_BOTTOM);
					break;
					case 20:
					calibration(position, ROBOT_RIGHT, WALL_BOTTOM);
					break;
					case 24:
					calibration(position, ROBOT_LEFT, WALL_BOTTOM);
					break;
					default:
					break;
				}
				currentWaypoint++;
			}
		}
	}
}


void compute_waypoint_speeds_coord(float* position, coord target, int *msl, int *msr, char robotState)
{
	float Ebearing = -(M_PI2 - atan2(target.y-position[1], (target.x-position[0])));
	if (Ebearing > M_PI)
		Ebearing -= M_2PI;
	if (Ebearing < -M_PI)
		Ebearing += M_2PI;

	Ebearing -= position[2];

	if (Ebearing > M_PI)
		Ebearing -= M_2PI;
	if (Ebearing < -M_PI)
		Ebearing += M_2PI;

	if (Ebearing > BEARING_GOAL_TRESH)
	{
		*msr += Kmotors_plus*abs(Ebearing);
		*msl = -1*(*msr);
	}
	else if (Ebearing < -BEARING_GOAL_TRESH)
	{
		*msl += Kmotors_plus*abs(Ebearing);
		*msr = -1*(*msl);
	}
	else
	{
		compute_bottle_speeds_coord(position, target, msl, msr, robotState);
	}
}

/** Adjust wheel speeds to follow target based on target 
	range and bearing
	
	@param taret_range Range between the target and the robot
	@param target_bearing Bearing between the target and the robot
	@param msl Speed of the left motors
	@param msr Speed of the right motors
	@return void
*/
/*
void compute_wheel_speeds(float target_range, float target_bearing, int *msl, int *msr)
{
	// Define constants
	float Ku = 2.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	float x = target_range * cosf(target_bearing);
	float y = target_range * sinf(target_bearing);
	float theta = 0;	//leader_orientation;
	//x += goal_range * cosf(- M_PI + goal_bearing + theta);
	//y += goal_range * sinf(- M_PI + goal_bearing + theta);
	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float w = Kw * range * sinf(bearing);// + Kb * leader_orientation;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)

	// Convert to wheel speeds!
	*msl = (int)((u - WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}
*/
/** Adjust wheel speeds to follow target based on target 
	coordinates
	
	@param taret Target coordinates: x and y
	@param msl Speed of the left motors
	@param msr Speed of the right motors
	@return void
*/
/*
void compute_wheel_speeds_coord(coord target, int *msl, int *msr)
{
	//target is coordinate relative to the current robot position
	// Define constants
	float Ku = 50.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	//float x = target_range * cosf(target_bearing);
	//float y = target_range * sinf(target_bearing);
	//float theta = 0;	//leader_orientation;
	//x += goal_range * cosf(- M_PI + goal_bearing + theta);
	//y += goal_range * sinf(- M_PI + goal_bearing + theta);
	float range = sqrtf(target.x*target.x + target.y*target.y); // This is the wanted position (range)
	float bearing = atan2(target.y, target.x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = (Ku * range * cosf(bearing))/1000;
	// Compute rotional control
	float w = (Kw * range * sinf(bearing))/1000;// + Kb * leader_orientation;

	// Convert to wheel speeds!
	*msl = (int)((u - WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}
*/