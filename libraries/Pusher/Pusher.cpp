    #include "Pusher.h"

unsigned int Pusher_count = 0;                      //used to keep count of how many interrupts were fired
bool resetPusher = false;                           //if true the pusher needs to be reset
bool pusherDirection = false;                       //State for pusher task
bool doorDirection = false;                         //State for door task
char depositionState = 0;                           //State for the deposition manoeuvre: 0 - not in deposition, 1 - during deposition, 2 - deposition ended
char doorState = 0;                                 //0 - open, 1 - close, 2 - moving

//Initialize Dynamixel connection
void initDynamixels()
{
    Serial.println("---- Initializing Dynamixels communication ----");

    Dynamixel.begin(1000000,7);  // Initialize the servo at 1Mbps and Pin Control 2
    delay(100);

    DymxPusher_Reset();
    DymxDoor_Reset();
}

//----- Pusher functions -----
/*
//Initialize timer2 and start it
void startTimer2_Pusher()
{
    //Setup Timer2 to fire every 16.32ms
    TCCR2B = 0x00;        //Disbale Timer2 while we set it up
    TCNT2  = 0;           //Reset Timer Count to 0 out of 255
    TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
    TCCR2B = 0x07;        //Timer2 Control Reg B: Timer Prescaler set to 1024

    Pusher_count = 0;
}

//Timer2 Overflow Interrupt Vector, called every 16.32ms
ISR(TIMER2_OVF_vect)
{
    Pusher_count++;                                            //Increments the interrupt counter
    if(Pusher_count == 2*PUSHER_TIME+50)
    {
        TCCR2B = 0x00;                                          //Disbale Timer2
        resetPusher = true;
        Pusher_count = 0;                                      //Resets the interrupt counter
    }
    else if(Pusher_count == PUSHER_TIME)
    {
      Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, PUSHER_SPEED);    //Change direction
    }
    TCNT2 = 0;           //Reset Timer to 0 out of 255
    TIFR2 = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};
*/
//Reset Pusher position and set to Endless
void DymxPusher_Reset()
{
    Serial.println("---- Reset Pusher ----");
    Dynamixel.turn(DYMX_PUSHER_ID,0,0);                         //Stop
    delay(20);
    Dynamixel.setEndless(DYMX_PUSHER_ID, OFF);                  //Set to position mode
    delay(20);
    Dynamixel.move(DYMX_PUSHER_ID, PUSHER_INIT_POS);           //Move to initial position
    delay(1000);
    if (depositionState == 1)
        depositionState = 2;
    Serial.println("---- End Reset Pusher ----");
}

//Check if Pusher must be reset
void DymxPusher_checkReset()
{
    if (resetPusher == true)
    {
        DymxPusher_Reset();
        resetPusher = false;
    }
}
/*
//Start emptying manoeuvre using timer2
void DymxPusher_EmptyBottles_Timer()
{
    //Serial.println("---- Pusher timer start ----");
    Dynamixel.setEndless(DYMX_PUSHER_ID, ON);                   //Set to speed mode
    Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, PUSHER_SPEED);     //Move forward to empty bottles
    startTimer2_Pusher();
}
*/
//Start emptying manoeuvre using task scheduler
void DymxPusher_EmptyBottles_Task()
{
    //Serial.println("---- Pusher task start ----");
    Dynamixel.setEndless(DYMX_PUSHER_ID, ON);                   //Set to speed mode
    
    if (pusherDirection == false)
    {
        Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, PUSHER_SPEED);     //Move forward to empty bottles
        pusherDirection = true;
    }
    else if (pusherDirection == true)
    {
        Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, PUSHER_SPEED);    //Change direction
        pusherDirection = false;
        resetPusher = true;
    }
}

//----- Door functions -----
//Reset Door position and set to position mode
void DymxDoor_Reset()
{
    Serial.println("---- Reset Door ----");
    Dynamixel.setEndless(DYMX_PUSHER_ID, OFF);          //Set to position mode
    delay(20);
    Dynamixel.move(DYMX_DOOR_ID, DOOR_INIT_POS);        //Move to initial position
    delay(500);
    doorDirection = false;
    Serial.println("---- End Reset Door ----");
}

//Move door to initial position
void DymxDoor_moveToInit()                             //move door to init position
{
    Dynamixel.moveSpeed(DYMX_DOOR_ID, DOOR_INIT_POS, DOOR_SPEED);
    doorDirection = true;
}

//Move door to end position
void DymxDoor_moveToEnd()                              //move door to end position
{
    Dynamixel.moveSpeed(DYMX_DOOR_ID, DOOR_END_POS, DOOR_SPEED);
    doorDirection = false;
}

//Close door
void DymxDoor_close()                              //move door to end position
{
    Dynamixel.moveSpeed(DYMX_DOOR_ID, DOOR_CLOSE_POS, DOOR_SPEED);
}



//Task function for door
void tDoor()
{
    if (doorDirection == false)
        DymxDoor_moveToInit();
    else if (doorDirection == true)
        DymxDoor_moveToEnd();
}