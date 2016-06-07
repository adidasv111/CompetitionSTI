#ifndef PUSHER_H
#define PUSHER_H

#include <Arduino.h>
#include <DynamixelSerial.h>
#include <constant.h>

#define DYMX_PUSHER_ID  11
#define DYMX_DOOR_ID	5

#define BACKWARD        false
#define FORWARD         true

/*
#define PUSHER_TIME     600                 //4.896 sec
#define PUSHER_INIT_POS 800
#define PUSHER_SPEED	1000

#define DOOR_INIT_POS	0
#define DOOR_END_POS	500
#define DOOR_SPEED		1000
#define DOOR_HALF_PERIOD 500				//in ms
*/

    void initDynamixels();                  //Initialize Dynamixel connection

    void DymxPusher_Reset();                //Reset Pusher position and set to Endless
    void DymxPusher_checkReset();           //Check if Pusher must be reset
    void DymxPusher_EmptyBottles();         //Start emptying manoeuvre
    void startTimer2_Pusher();              //Initialize timer2 and start it
    ISR(TIMER2_OVF_vect);                   //Timer2 Overflow Interrupt Vector, 

    void DymxDoor_Reset();					//Reset Door position and set to position mode
    void DymxDoor_moveToInit();				//move door to init position
    void DymxDoor_moveToEnd();				//move door to end position
	void DymxDoor_close();					//close door

    void tDoor();							//Task function for door
#endif
