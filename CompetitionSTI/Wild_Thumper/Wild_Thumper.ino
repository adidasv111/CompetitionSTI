// UART is D0(RX) and D1(TX)
#include "IOpins.h"
#include "Constants.h"
#include <Wire.h>

//------- define global variables -------
unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int Leftspeed = 0;
int Rightspeed = 0;
int Speed;
int Steer;
byte Charged = 1;                                             // 0=Flat battery  1=Charged battery
int Leftmode = 1;                                             // 0=reverse, 1=brake, 2=forward
int Rightmode = 1;                                            // 0=reverse, 1=brake, 2=forward
byte Leftmodechange = 0;                                      // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange = 0;                                     // Right input must be 1500 before brake or reverse can occur
int LeftPWM;                                                  // PWM value for left  motor speed / brake
int RightPWM;                                                 // PWM value for right motor speed / brake
int data;

long last_update;

void setup()
{
  // Initialize I/O pins
  pinMode (Charger, OUTPUT);                              // change Charger pin to output
  pinMode (LED_Battery, OUTPUT);                          // change LED_Battery pin to output
  pinMode (LED_Com, OUTPUT);                              // change LED_Battery pin to output
  pinMode (LED_mode, OUTPUT);                            // change LED_Battery pin to output
  digitalWrite (Charger, 1);                              // disable current regulator to charge battery
  digitalWrite (LED_Battery, LOW);                        //turn off LED_Battery in the beginning
  digitalWrite (LED_Com, LOW);                            //turn off LED_Com in the beginning
  digitalWrite (LED_mode, LOW);                           //turn off LED_mode in the beginning

  if (Cmode == 1)   //Serial mode
  {
    Serial.begin(Brate);                                      // enable serial communications if Cmode=1
    Serial.flush();                                           // flush buffer
    digitalWrite (LED_mode, LOW);
  }
  else if (Cmode == 2) //I2C mode
  {
    // enable pull-ups
    digitalWrite(18, 1);
    digitalWrite(19, 1);

    Wire.begin(1);                // join i2c bus with address #1
    Wire.onReceive(receiveEvent); // register event
    digitalWrite (LED_mode, HIGH);
  }
  //Stop motors
  Leftmode = 1;
  LeftPWM = 255;
  Rightmode = 1;
  RightPWM = 255;
  last_update = millis();
}

void loop()
{
  //------- Check battery voltage and current draw of motors -------
  Volts = analogRead(Battery);                                // read the battery voltage
  LeftAmps = analogRead(LmotorC);                             // read left motor current draw
  RightAmps = analogRead(RmotorC);                            // read right motor current draw

  // left motor current draw exceeding safe limit
  if (LeftAmps > Leftmaxamps)
  {
    analogWrite (LmotorA, 0);                                 // turn off motors
    analogWrite (LmotorB, 0);                                 // turn off motors
    leftoverload = millis();                                  // record time of overload
  }

  // right motor current draw exceeding safe limit
  if (RightAmps > Rightmaxamps)
  {
    analogWrite (RmotorA, 0);                                 // turn off motors
    analogWrite (RmotorB, 0);                                 // turn off motors
    rightoverload = millis();                                 // record time of overload
  }

  // FLAT BATTERY (battery becoming flat)
  if ((Volts < lowvolt) && (Charged == 1))
  {
    //speed controller shuts down until battery is recharged (safety feature to prevent malfunction at low voltages!)
    Charged = 0;                                              // change battery status from charged to flat
    highVolts = Volts;                                        // record the voltage
    startVolts = Volts;
    chargeTimer = millis();                                   // record the time

    digitalWrite (Charger, 0);                                // enable current regulator to charge battery
    digitalWrite (LED_Battery, LOW);                          //turn off LED_Battery
  }

  // CHARGE BATTERY (battery is flat and charger has been connected (voltage has increased by at least 1V))
  if ((Charged == 0) && (Volts - startVolts > 67))
  {
    if (Volts > highVolts)                                    // has battery voltage increased?
    {
      highVolts = Volts;                                    // record the highest voltage. Used to detect peak charging.
      chargeTimer = millis();                               // when voltage increases record the time
    }

    if (Volts > batvolt)                                      // battery voltage must be higher than this before peak charging can occur.
    {
      if ((highVolts - Volts) > 5 || (millis() - chargeTimer) > chargetimeout) // has voltage begun to drop or levelled out?
      {
        Charged = 1;                                      // battery voltage has peaked
        digitalWrite (Charger, 1);                        // turn off current regulator
      }
    }
    digitalWrite (LED_Battery, LOW);                          //turn off LED_Battery
  }

  // GOOD BATTERY - normal behavior
  else
  {
    if (Cmode == 1)   // Serial mode via D0(RX) and D1(TX)
    {
      SCmode(); 
    }
    else if (Cmode == 2) //I2C mode
    {
      I2Cmode();
    }

    // Drive dual "H" bridges based on acquired motors modes and speeds
    if (Charged == 1)                                           // Only power motors if battery voltage is good
    {
      digitalWrite (LED_Battery, HIGH);                         //turn on LED_Battery
      if ((millis() - leftoverload) > overloadtime)             // if left motor has not overloaded recently
      {
        switch (Leftmode)
        {
          case 2:                                                 // left motor forward
            analogWrite(LmotorA, 0);
            analogWrite(LmotorB, LeftPWM);
            break;

          case 1:                                                 // left motor brake
            analogWrite(LmotorA, LeftPWM);
            analogWrite(LmotorB, LeftPWM);
            break;

          case 0:                                                 // left motor reverse
            analogWrite(LmotorA, LeftPWM);
            analogWrite(LmotorB, 0);
            break;
        }
      }
      if ((millis() - rightoverload) > overloadtime)            // if right motor has not overloaded recently
      {
        switch (Rightmode)
        {
          case 2:                                                 // right motor forward
            analogWrite(RmotorA, 0);
            analogWrite(RmotorB, RightPWM);
            break;

          case 1:                                                 // right motor brake
            analogWrite(RmotorA, RightPWM);
            analogWrite(RmotorB, RightPWM);
            break;

          case 0:                                                 // right motor reverse
            analogWrite(RmotorA, RightPWM);
            analogWrite(RmotorB, 0);
            break;
        }
      }
    }
    else                                                        // Battery is flat
    {
      analogWrite (LmotorA, 0);                                 // turn off motors
      analogWrite (LmotorB, 0);                                 // turn off motors
      analogWrite (RmotorA, 0);                                 // turn off motors
      analogWrite (RmotorB, 0);                                 // turn off motors
      digitalWrite (LED_Battery, LOW);                          //turn off LED_Battery
    }
  }
}

void SCmode()
{ // ------------------------------------------------------------ Code for Serial Communications --------------------------------------
  // "FL"(17996) = flush serial buffer
  // "AN"(16718) = report Analog inputs 1-5
  // "SV"(21334) = next 7 integers will be position information for servos 0-6 - NOT IMPLEMENTED
  // "HB" = "H" bridge data - next 4 bytes will be:
  //          left  motor mode 0-2 (direction)
  //          left  motor PWM  0-255
  //          right motor mode 0-2
  //          right motor PWM  0-255
  if (Serial.available() > 1)                                     // command available
  {
    /* int A=Serial.read();
      int B=Serial.read();
      int command=A*256+B;
      switch (command)
      {
       case 17996:                                             // FL
         Serial.flush();                                       // flush buffer
         break;

       case 16718:                                             // AN - return values of analog inputs 1-5
         for (int i=1;i<6;i++)                                 // index analog inputs 1-5
         {
           data=analogRead(i);                                 // read 10bit analog input
           Serial.write(highByte(data));                       // transmit high byte
           Serial.write(lowByte(data));                        // transmit low byte
         }
         break;

        case 18498:                                            // HB - mode and PWM data for left and right motors
    */    
    Serialread();
    Leftmode = data;
    Serialread();
    LeftPWM = data;
    Serialread();
    Rightmode = data;
    Serialread();
    RightPWM = data;

    /*     break;

       default:                                                // invalid command
         Serial.flush();                                       // flush buffer
      }*/
  }
}

// Read serial port until data has been received
void Serialread()
{
  do
  {
    data = Serial.read();
  } while (data < 0);
}

void I2Cmode()
{
/*
  if (millis()-last_update>1000) {
    Speed=0;
    Steer=0;
  }
  Set_Speed(Speed, Steer);
  */
}

void receiveEvent(int bytesReceived)
{
  if (Wire.available()==4)
  {
         Leftmode=Wire.read();
         LeftPWM=Wire.read();
         Rightmode=Wire.read();
         RightPWM=Wire.read();

         last_update=millis();
             if (Leftmode == 2)
             {
             }
     /*char cmd=Wire.read();
     if (cmd=='s') { //set speed
        Speed=(int)(((unsigned int)Wire.read())<<8 | (unsigned int)Wire.read());
        last_update=millis();
     } else
     if (cmd=='t') { //set turn speed
        Steer=(int)(((unsigned int)Wire.read())<<8 | (unsigned int)Wire.read());
        last_update=millis();
     }*/
   } 
   
   // empty buffer
   while (Wire.available()>0) Wire.read();
   /*
       digitalWrite (LED_Com,HIGH);
    delay(500);
        digitalWrite (LED_Com,LOW);
    delay(500);
    */
}
