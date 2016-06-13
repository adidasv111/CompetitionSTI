#ifndef PUSHER_H
#define PUSHER_H

#include <Arduino.h>
#include <constant.h>
#include <DynamixelSerial.h>

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
    void DymxPusher_startTask();
    //void DymxPusher_EmptyBottles_Timer();   //Start emptying manoeuvre using timer2
    //void DymxPusher_EmptyBottles_Task();    //Start emptying manoeuvre using task scheduler
    
    void DymxDoor_Reset();					//Reset Door position and set to position mode
    void DymxDoor_moveToInit();				//Move door to initial position
    void DymxDoor_moveToEnd();				//Move door to end position
	void DymxDoor_setPosition(int position);	//set door position

    void tDoor();							//Task function for door

    extern char doorState;                  //State for door task
    extern char depositionState;            //State for the deposition manoeuvre: 0 - not in deposition, 1 - during deposition, 2 - deposition almost ended, 3 - deposition ended
#endif
