#include "Pusher.h"

unsigned int Pusher_count = 0;                             //used to keep count of how many interrupts were fired
bool resetPusher = false;                           //if true the pusher needs to be reset

//Initialize Dynamixel connection
void initDynamixels()
{
    Serial.println("---- Initializing Dynamixels communication ----");
    Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
}

//Reset Pusher position and set to Endless
void DymxPusher_Reset()
{
    Serial.println("---- Reset Pusher ----");
    Dynamixel.turn(DYMX_PUSHER_ID,0,0);                 //Stop
    delay(20);
    Dynamixel.setEndless(DYMX_PUSHER_ID, OFF);          //Set to position mode
    delay(20);
    Dynamixel.move(DYMX_PUSHER_ID, INIT_POS);           //Move to initial position
    delay(1000);
    Dynamixel.setEndless(DYMX_PUSHER_ID, ON);           //Set to speed mode
    Serial.println("---- End Reset Pusher ----");
}

void DymxPusher_checkReset()           //Check if Pusher must be reset
{
    if (resetPusher == true)
    {
        DymxPusher_Reset();
        resetPusher = false;
    }

}
    
//Start emptying manoeuvre
void DymxPusher_EmptyBottles()
{
    Serial.println("---- Pusher start ----");
    Dynamixel.setEndless(DYMX_PUSHER_ID, ON);           //Set to speed mode
    Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, 1000);     //Move forward to empty bottles
    startTimer2_Pusher();
}

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
        TCCR2B = 0x00;                                  //Disbale Timer2
        resetPusher = true;
        Pusher_count = 0;                                      //Resets the interrupt counter
    }
    else if(Pusher_count == PUSHER_TIME)
    {
      Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, 1000);    //Change direction
    }
    
    TCNT2 = 0;           //Reset Timer to 0 out of 255
    TIFR2 = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};
