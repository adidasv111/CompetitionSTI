#include "Motors.h"
#include "Odometry.h"



void setup()
{
    Serial.begin(9600);
  initMotors();
  initOdometry();
  
  setSpeeds(-105,100);
}

void loop()
{
  calcOdometry();
  Serial.print(robotPosition[0]);
  Serial.print("  ");
  Serial.print(robotPosition[1]);
  Serial.print("  ");
  Serial.println(robotPosition[2]);
  delay(1000);
 /*
  setSpeeds(255,255);
    calcOdometry();
  Serial.print(robotPosition[0]);
  Serial.print("  ");
  Serial.print(robotPosition[1]);
  Serial.print("  ");
  Serial.println(robotPosition[2]);
  delay(2000);
  calcOdometry();
  Serial.print(robotPosition[0]);
  Serial.print("  ");
  Serial.print(robotPosition[1]);
  Serial.print("  ");
  Serial.println(robotPosition[2]);
  setSpeeds(0,0);
  delay(2000);

  setSpeeds(-255,-255);
 delay(1000);
 */
}
