#include "Odometry.h"

float robotPosition [3];  // x, y, theta

volatile int leftEncTicks = 0;
volatile int rightEncTicks = 0;
//unsigned int prevLeftTicks = 0;
//unsigned int prevRightTicks = 0;

void initOdometry()
{
  pinMode(leftEncA, INPUT); 
  //digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(leftEncB, INPUT); 
  //digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  pinMode(rightEncA, INPUT); 
  pinMode(rightEncB, INPUT); 
  attachInterrupt(0, doEncoderLeft, RISING);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoderRight, RISING);   // encoder pin on interrupt 1 - pin 3
  
  robotPosition[0] = 0;
  robotPosition[1] = 0;
  robotPosition[2] = 0;
}

void doEncoderLeft()
{
	//If (channel A == channel B) => B lead => counter-clockwise => reverse => decrement counter.
	//If (channel A != channel B) => B late => clockwise => forward => increment counter.
	if (digitalRead(leftEncA) == digitalRead(leftEncB))
	{
		leftEncTicks--;
	}
	else
	{
		leftEncTicks++;
	}
}

void doEncoderRight()
{
  //If (channel A == channel B) => B lead => counter-clockwise => forward => increment counter.  ***Different from left motor! ***
  //If (channel A != channel B) => B late => clockwise => reverse => decrement counter.
	if (digitalRead(rightEncA) == digitalRead(rightEncB))
	{
		rightEncTicks++;
	}
	else
	{
		rightEncTicks--;
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
    
    Serial.print("left:   ");
    Serial.println(leftTicks);
    Serial.print("right:   ");
    Serial.println(rightTicks);
  
    //calculate movement of each wheel in meters
    float dist_right = rightTicks/TICKS_PER_M;
    float dist_left = leftTicks/TICKS_PER_M;
  
    float du = (dist_right + dist_left)/2.0;
    //float dtheta = (dist_right - dist_left)/WHEEL_BASE;
    robotPosition[2] += (dist_left - dist_right)/WHEEL_BASE;
  
    // Keep orientation within 0, 2pi
    if (robotPosition[2] > M_2PI) robotPosition[2] -= M_2PI;
    if (robotPosition[2] < 0) robotPosition[2] += M_2PI;
  
    //update robotPosition
    robotPosition[0] += 1000*du*sin(robotPosition[2]);
    robotPosition[1] += 1000*du*cos(robotPosition[2]);
}
