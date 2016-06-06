#ifndef PUSHER_H
#define PUSHER_H

#include <Arduino.h>
#include <DynamixelSerial.h>

#define DYMX_PUSHER_ID  11
#define PUSHER_TIME     600                    //4.896 sec
#define BACKWARD        false
#define FORWARD         true
#define INIT_POS        800

    void initDynamixels();                  //Initialize Dynamixel connection
    void DymxPusher_Reset();                //Reset Pusher position and set to Endless
    void DymxPusher_checkReset();           //Check if Pusher must be reset
    void DymxPusher_EmptyBottles();         //Start emptying manoeuvre
    void startTimer2_Pusher();              //Initialize timer2 and start it
    ISR(TIMER2_OVF_vect);                   //Timer2 Overflow Interrupt Vector, 
           
#endif
