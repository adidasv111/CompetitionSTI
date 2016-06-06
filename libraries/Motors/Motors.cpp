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

/** Adjust wheel speeds to follow target based on target 
	range and bearing
	
	@param taret_range Range between the target and the robot
	@param target_bearing Bearing between the target and the robot
	@param msl Speed of the left motors
	@param msr Speed of the right motors
	@return void
*/
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

/** Adjust wheel speeds to follow target based on target 
	coordinates
	
	@param taret Target coordinates: x and y
	@param msl Speed of the left motors
	@param msr Speed of the right motors
	@return void
*/
void compute_wheel_speeds_coord(coord target, int *msl, int *msr)
{
	
	// Define constants
	float Ku = 2.0;
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
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float w = Kw * range * sinf(bearing);// + Kb * leader_orientation;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)

	// Convert to wheel speeds!
	*msl = (int)((u - WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + WHEEL_BASE*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}