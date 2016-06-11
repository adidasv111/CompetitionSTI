/**
    C++
    Odometry.cpp
    Purpose: Manage odometry of the robot

    @author Adi Vardi
    @version 1.0 3/6/16 
*/

#include "Odometry.h"

float robotPosition [3];  // x, y, theta
float theta = 0;			//Theta from the odometry calculation
float thetaCompass = 0;		//Theta from the compass
volatile int leftEncTicks = 0;
volatile int rightEncTicks = 0;
//unsigned int prevLeftTicks = 0;
//unsigned int prevRightTicks = 0;

/** Initialize odometry of the robot
	
	@return void
*/
  void initOdometry()
  {
    pinMode(leftEncA, INPUT); 
  //digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
    pinMode(leftEncB, INPUT); 
  //digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
    pinMode(rightEncA, INPUT); 
    pinMode(rightEncB, INPUT); 
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderLeft, RISING);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(digitalPinToInterrupt(3), doEncoderRight, RISING);   // encoder pin on interrupt 1 - pin 3
  
  robotPosition[0] = INIT_X;
  robotPosition[1] = INIT_Y;
  robotPosition[2] = INIT_THETA; 
  
  /* In case of reset
    robotPosition[0] = ;
  robotPosition[1] = ;
  robotPosition[2] = ;
  */
}

void doEncoderLeft()
{
  //Serial.println("left encoder");
	//If (channel A == channel B) => B lead => counter-clockwise => reverse => decrement counter.
	//If (channel A != channel B) => B late => clockwise => forward => increment counter.
	if (digitalRead(leftEncA) == digitalRead(leftEncB))
	{
		leftEncTicks++;
	}
	else
	{
		leftEncTicks--;
	}
}

void doEncoderRight()
{
  //Serial.println("right encoder");
  //If (channel A == channel B) => B lead => counter-clockwise => forward => increment counter.  ***Different from left motor! ***
  //If (channel A != channel B) => B late => clockwise => reverse => decrement counter.
	if (digitalRead(rightEncA) == digitalRead(rightEncB))
	{
		rightEncTicks--;
	}
	else
	{
		rightEncTicks++;
	}
}


void calcOdometry()
{
/*
  	//sample encoders at the beginning of this function
  	int currentLeftTicks = leftEncTicks;
  	int currentRightTicks = rightEncTicks;
  	
  	//calculate difference in ticks from last sampling
  	int leftTicks = currentLeftTicks - prevLeftTicks;
  	int rightTicks = currentRightTicks - prevRightTicks;
  	
  	//update ticks counts for next sampling
  	prevLeftTicks = currentLeftTicks;
  	prevRightTicks = currentRightTicks;
  	*/
  	//calculate difference in ticks from last sampling
  	int leftTicks = leftEncTicks;
  	int rightTicks = rightEncTicks;

  	//initalize encoders counters so they are differential
  	leftEncTicks = 0;
  	rightEncTicks = 0;
    /*
    Serial.print("left:   ");
    Serial.println(leftTicks);
    Serial.print("right:   ");
    Serial.println(rightTicks);
  */
    //calculate movement of each wheel in meters
    float dist_right = rightTicks/TICKS_PER_M;
    float dist_left = leftTicks/TICKS_PER_M;

    float du = (dist_right + dist_left)/2.0;
    
    theta += (dist_right - dist_left)/WHEEL_BASE;
    
    // Keep orientation within -pi, pi
    if (theta > M_PI)
      theta -= M_2PI;
    if (theta < -M_PI)
      theta += M_2PI;
    
	thetaCompass = readCompass_Serial2();	// Read orientation from compass
	robotPosition[2] = theta;
	//robotPosition[2] = COMPASS_WEIGHT*thetaCompass + (1-COMPASS_WEIGHT)*theta;	//Combine orientation from compass and odometry
	/*
	Serial.print("theta:   ");
    Serial.println(theta);
    Serial.print("thetaCompass:   ");
    Serial.println(thetaCompass);
	Serial.print("final theta:   ");
    Serial.println(robotPosition[2]);
	*/
    //update robotPosition in mm
    robotPosition[0] += -1000*du*sin(robotPosition[2]);
    robotPosition[1] += 1000*du*cos(robotPosition[2]);
/*
      Serial.print("x:   ");
    Serial.println(robotPosition[0]);
    Serial.print("y:   ");
    Serial.println(robotPosition[1]);
  Serial.print(" theta:   ");
    Serial.println(robotPosition[2]);
    */
  }