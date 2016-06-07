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
    Serial.println(readCompass_Serial2(), 3);
  delay(300);
}

