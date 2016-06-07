#include <DynamixelSerial.h>
#include "Pusher.h"

void setup()
{
  Serial.begin(9600);
  initDynamixels();

  //DymxPusher_Reset();

  DymxPusher_EmptyBottles();
}


void loop()
{
    DymxPusher_checkReset();
    delay(1000);
}
