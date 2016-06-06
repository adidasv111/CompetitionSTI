#include "Compass.h"

void setup() {

    Serial.begin(9600);

      initCompass_Serial2();
//    Serial2.write(0xA0);
//    Serial2.write(0xAA);
//    Serial2.write(0xA5);
//    Serial2.write(0xE0);
}

void loop()
{
    //readCompass_I2C();
    readCompass_Serial2();
    Serial.println(yawCompass, 1);
  delay(300);
}

