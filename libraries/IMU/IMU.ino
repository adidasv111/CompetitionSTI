#include "IMU.h"




void setup()
{
    Serial.begin(9600);
    
    initIMU(0);    
}


void loop()
{
   calcIMU();
   printReadings();
//roll, pitch, yaw in IMUAngles[]

}


