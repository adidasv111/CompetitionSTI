#include "Compass.h"

float yawCompass;					//yaw measurement in rad
float yawCompass0 = 0;				//Initial yaw measurement in rad
char values[8];

//Initialize communication and initial yaw
void initCompass_Serial2()
{
  Serial.println("---- Initializing Compass ----");
  Serial2.begin(9600);

  readCompass_Serial2();
  delay(500);
  readCompass_Serial2();
  delay(200);
  readCompass_Serial2();
  yawCompass0 = yawCompass;		//Initilize yawCompass0 to the first measurement
  Serial.print("yawCompass0: ");
  Serial.println(yawCompass0);
}

//--------- Serial 2 ----------
//Read compass
float readCompass_Serial2()
{
  Serial2.write(0x31);

  if (Serial2.available())
  {
    for (int i = 0; i < 8; i++)
    {
      values[i] = Serial2.read();
    }
  }
  yawCompass = (values[2] & 0x0F) * 100 + (values[3] & 0x0F) * 10 + (values[4] & 0x0F) + (values[6] & 0x0F) * 0.1;
  yawCompass *= -1;
  yawCompass -= yawCompass0;
  
  // Keep orientation within -pi, pi
    if (yawCompass > 180)
		yawCompass -= 360;
    if (yawCompass <= -180)
		yawCompass += 360;  

  yawCompass = yawCompass * M_PI / 180;
  return yawCompass;
}


//Start calibration process
void calibrateCompass_Serial2()
{
  Serial.println("---- Calibrating Compass ----");
  Serial.println("Get ready to Rotate the compass two circles slowly, flatly and in a constant speed, a turn in around one minute!");
  delay(3000);
  Serial.println("Start roating");

  Serial2.write(0xC0);
  delay(125000);
  Serial2.write(0xC1);

  Serial.println("Compasse Calibration done!");
  Serial.println("Put robot down");
  delay(4000);
}
/* Other calibration method -----
  Method is just fix the compass module to whatever device you are using (in my case its my metal robot).
  Then ground and release the PIN 9 (CAL pin) of the module.
  now you will see the module is sending 000.00 and the green LED on the module is keep on.
  So this means your module is now on calibrating mode.
  Next step is just turn clockwise and anti-clockwise the device two three times
  (no need to turn the device so slowly as they have asked on the datasheet, and no need to turn complete rounds).
  Finally again ground the CAL pin and release it. That's it. Now your GY-26 compass module is fully calibrated.
*/


//--------- I2C ----------
float readCompass_I2C()
{
  Wire.beginTransmission(ADDRESS_COMPASS);
  // Wire.write(0x00);// first byte of "Get Data" command
  Wire.write(0x31);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(ADDRESS_COMPASS, 1);

  if (Wire.available())
  {
    values[0] = Wire.read(); // receive a byte as character
    Serial.print("val");
    Serial.print(values[0]);
  }

  yawCompass = (values[2] & 0x0F) * 100 + (values[3] & 0x0F) * 10 + (values[4] & 0x0F) + (values[6] & 0x0F) * 0.1;
  yawCompass -= yawCompass0;
  
  // Keep orientation within -pi, pi
    if (yawCompass > 180)
		yawCompass -= 360;
    if (yawCompass <= -180)
		yawCompass += 360;  

  yawCompass = yawCompass * M_PI / 180;
  return yawCompass;
}
