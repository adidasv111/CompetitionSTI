#include "Motors.h"

// mode: 0=reverse, 1=brake, 2=forward
// PWM: PWM value for right motor speed / brake

void initMotors()
{
  Serial3.begin(motorControllerBrate);
  setSpeeds(0, 0);
}

void setSpeeds(int speedLeft, int speedRight)
{
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
  setSpeeds(0, 0);
}

void setSpeeds_I2C(int speedLeft, int speedRight)
{
  // Serial3.write("HB");
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
