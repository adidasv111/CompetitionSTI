#include <Arduino.h>
#include <constant.h>
#include <Motors.h>
#include <IMU.h>
#include <Pusher.h>
#include <Odometry.h>
#include <mapping.h>
#include <braitenberg.h>
#include <US_Sensor.h>
#include <IR_Sensor.h>
//#include <gridSearch.h>
#include <Misc.h>
#include <TaskScheduler.h>


//----- Global Variables -----
char robotState = GOING_TO_WAYPOINT;    //state of state machine
coord destination;                      //Coordinates for the current destination of the robot
int left_speed = 0, right_speed = 0;    //motors speeds, changed every call of planning

unsigned int planningCounter = 0;
int blockedFlag = 0;
unsigned int blockedCounter = 0;            //counter for the evasive manoeuvre
unsigned int depositionTimeoutCounter = 0;  //counter for the deposition reverde
unsigned int captureBottleCounter = 0;      //counter for bottle capturing
int calibrationFlag = 0;
int pusherState = 0;
unsigned 
int depositionShake = 0;

//----- Headers for functions -----
void planning();
void deposition();
//void DepositionTimeout();
void timeoutWaypoint();
//void tCaptureBottle();
void blockedEvasiveManoeuvre();
void DymxDoor_setState(int stateDoor);
//void get_info_from_pi();
void goHomeItsTooLate();
void goHomeItsBeenTooLong();
void stopRobot();

void tprint()
{
  Serial.print("x:   ");
  Serial.println(robotPosition[0]);
  Serial.print("y:   ");
  Serial.println(robotPosition[1]);
  Serial.print(" theta:   ");
  Serial.println(robotPosition[2] * 180 / M_PI);
  Serial.print("left:   ");
  Serial.print(left_speed);
  Serial.print("      right:   ");
  Serial.println(right_speed);
  Serial.print("current target:   ");
  Serial.print(destination.x);
  Serial.print(" , ");
  Serial.println(destination.y);
  Serial.print("state:   ");
  Serial.println((int)robotState);
  //Serial.print("blocked?:   ");
  //Serial.println((int)blockedFlag);
  //Serial.println(blockedCounter);
}

//----- Tasks definitions -----
Task OdometryTask(100, TASK_FOREVER, &calcOdometry);                                //Create task that is called every 100ms and last forever to calculate odometry
Task PlanningTask(PLANNING_FREQ, TASK_FOREVER, &planning);
Task TimeoutWaypointTask(30 * TASK_SECOND, 1, &timeoutWaypoint);
//Task CaptureBottleTask(1000, 1, &tCaptureBottle);                                   //move forward over the bottle for 1sec when capturing
//Task DepositionTimeoutTask(3000, 1, &DepositionTimeout);
//Task EvasiveManoeuvreTask(EVASIVE_MANOEUVRE_DELAY, 2, &evasiveManoeuvre);

Task DoorMoveTask(DOOR_HALF_PERIOD, TASK_FOREVER, &tDoor);                          //Create task that moves the door back and forth
Task FullTask(2000, TASK_FOREVER, &checkFull);                                      //Create task that check if full
//Task DepositionTask(100, TASK_FOREVER, &deposition);                              //Deposition manoeuvre
Task PusherTask(PUSHER_HALF_PERIOD, 3, &DymxPusher_EmptyBottles_Task);              //Create task that moves the pusher back and forth once
//Task PusherResetTask(PUSHER_RESET_PERIOD, TASK_FOREVER, &DymxPusher_checkReset);    //Create task that checks if pusher is reseted

Task GeorgeGoHomeItsTooLateTask(ITS_TOO_LATE_INT, 1, &goHomeItsTooLate);                        //go home after 9.30 minutes
Task GeorgeGoHomeItsBeenTooLongTask(ITS_BEEN_TOO_LONG_INT, 1, &goHomeItsBeenTooLong);                  //go home if it's been 3 minutes since last deposition

//Task PiComTask(500, TASK_FOREVER, &get_info_from_pi);                               //Create task that communicate with the PI
//Task PrintTask(1000, TASK_FOREVER, &tprint);

Task DistSensorTask(20, TASK_FOREVER, &updateIRSensors);                           //Create task that updates IR Sensors
//Task StopTask(1000, 1, &stopRobot);

Scheduler runner;

//*****************************SETUP*************************/
void setup()
{
  Serial.begin(9600);

  initOdometry();
  init_waypoints();
  init_bottlesArray();
  initDynamixels();             //Already includes DymxPusher_Reset() and DymxDoor_Reset()
  initMotors_I2C();

  //initIMU(0);
  //initCompass_Serial2();
  //calibrateCompass_Serial2();

  destination.x = INIT_X;
  destination.y = INIT_Y;

  //Adding all tasks to runner
  runner.init();
  runner.addTask(OdometryTask);
  runner.addTask(PlanningTask);
  runner.addTask(TimeoutWaypointTask);
  //runner.addTask(CaptureBottleTask);
  //runner.addTask(EvasiveManoeuvreTask);
  //runner.addTask(DepositionTimeoutTask);
  runner.addTask(DoorMoveTask);
  runner.addTask(FullTask);
  //runner.addTask(DepositionTask);
  runner.addTask(PusherTask);
  //runner.addTask(PusherResetTask);
  runner.addTask(GeorgeGoHomeItsTooLateTask);
  runner.addTask(GeorgeGoHomeItsBeenTooLongTask);
  //runner.addTask(PiComTask);
  //runner.addTask(PrintTask);

  runner.addTask(DistSensorTask);
  DistSensorTask.enable();

  //enabling tasks that should start at beginnig of the programm
  OdometryTask.enable();
  PlanningTask.enable();
  //EvasiveManoeuvreTask.disable();
  FullTask.enableDelayed(FULL_DELAY);
  //PusherResetTask.enable();
  GeorgeGoHomeItsTooLateTask.enableDelayed(ITS_TOO_LATE_INT);
  GeorgeGoHomeItsBeenTooLongTask.enableDelayed(ITS_BEEN_TOO_LONG_INT);
  //PiComTask.enable();
  //PrintTask.enableDelayed(250);
}

//***************************** LOOP *************************
void loop()
{
  runner.execute();
}

//***************************** TASK FUNCTIONS *************************
//--- planning ---
void planning()
{
  left_speed = 200;
  right_speed = 200;

  checkEvasiveManoeuvre(&blockedFlag);

  if (isFull && (robotState != GOING_HOME) && (robotState != DEPOSITION)) //Container is full, start going home
  {
    robotState = GOING_HOME;            //state = GOING_HOME;
    FullTask.disable();                 //disable full check when going home
  }
  if (robotState == GOING_HOME)         //going home
  {
    //DymxDoor_setState(DOOR_CLOSE);      //close the door when going home
    DymxDoor_setState(DOOR_OPEN);
    destination.x = HOME_X;
    destination.y = HOME_Y;
  }
  else if (robotState == DEPOSITION)
  {
    destination.x = HOME_X;
    destination.y = HOME_Y;
  }
  else if (robotState == GOING_TO_WAYPOINT)
  {
    /*   destination = findClosestBottle(robotPosition);
       if (destination.x != -1)    //new target found
       {
         robotState == GOING_TO_BOTTLE;
       }
       else        // no new target found
       {*/
    //DymxDoor_setState(DOOR_CLOSE);          //close the door when going to waypoint
    DymxDoor_setState(DOOR_MOVE);
    destination.x = waypoints[currentWaypoint].x;
    destination.y = waypoints[currentWaypoint].y;
    //  }
  }
  /*    else if (robotState == GOING_TO_BOTTLE)    //Target already present, robot must continue tragectory towards target
      {
        DymxDoor_setState(DOOR_MOVE);   //start moving door
        //destination is already set to target
        compute_bottle_speeds_coord(robotPosition, destination, &left_speed, &right_speed, robotState);  //compute speeds to go to waypoint
      }
  */
  if (planningCounter >= checkObstacle())
  {
    compute_waypoint_speeds_coord(robotPosition, destination, &left_speed, &right_speed, robotState);  //compute speeds to go to bottle
    planningCounter = 0;
  }

  if (blockedFlag == 2)
  {
    left_speed = 150;
    right_speed = 255;

    blockedCounter++;
    if (blockedCounter >= EVASIVE_MANOEUVRE_DELAY)
    {
      blockedFlag = 0;
      blockedCounter = 0;
    }
    /* if (!EvasiveManoeuvreTask.isEnabled())  //if full task isn't already enabled
      {
       EvasiveManoeuvreTask.enableDelayed(EVASIVE_MANOEUVRE_DELAY);
      }*/
  }
  else if (blockedFlag == 4)
  {
    left_speed = 255;
    right_speed = 150;

    blockedCounter++;
    if (blockedCounter >= EVASIVE_MANOEUVRE_DELAY)
    {
      blockedFlag = 0;
      blockedCounter = 0;
    }
    /*EvasiveManoeuvreTask.enable();
      if (EvasiveManoeuvreTask.isEnabled() == false)  //if full task isn't already enabled
      {
            Serial.println("dkfieuhfiuehfbcdviu start");
      EvasiveManoeuvreTask.enableDelayed(EVASIVE_MANOEUVRE_DELAY);
      }*/
  }

  if (robotState == DEPOSITION)
    deposition();                 //DepositionTask is working. Wait for it to change robotState to GOING_TO_WAYPOINT when done.
  else
    obstacle_avoidance(&left_speed, &right_speed); //Turn on updateIRSensor function

  //Checking if current goal is achieved
  check_goal(robotPosition, destination, robotState);
  if (gotHome)                            //if got home
  {
    robotState = DEPOSITION;
    //DepositionTask.enable();
  }
  if (gotWaypoint == 1)
  {
    TimeoutWaypointTask.enableIfNot();
  }
  if (gotWaypoint == 2)
  {
    startCalibration(robotPosition, &left_speed, &right_speed, &calibrationFlag);
    if (calibrationFlag == 0)
    {
      gotWaypoint = 0;
      currentWaypoint++;
      TimeoutWaypointTask.disable();
      
      if (currentWaypoint == 3 || currentWaypoint == 9 || currentWaypoint == 13 || currentWaypoint == 17 || currentWaypoint == 22 || currentWaypoint == 25)
    {
      isFull = true;
    }
    }
    
  }
  /*  if (gotBottle)
    {
      //CaptureBottleTask.enableIfNot();
      left_speed = 255;
      right_speed = 255;

      captureBottleCounter++;
      if (captureBottleCounter >= CAPTURE_BOTTLE_DELAY)
      {
        gotBottle = false;
        captureBottleCounter = 0;
        robotState == GOING_TO_WAYPOINT;
      }
      obstacle_avoidance(&left_speed, &right_speed); //Turn on updateIRSensor function
    }*/

  /**********TESTING*****************/
  //left_speed = right_speed = 0;
  /**********************************/

  setSpeeds_I2C(left_speed, right_speed);
  planningCounter++;

  if (currentWaypoint > NB_WAYPOINTS)
  {
    setSpeeds_I2C(0, 0);
    runner.disableAll();
  }
}

//------ deposition -----
void deposition()                   //Deposition manoeuvre
{
  gotHome = false;
  DymxDoor_setState(DOOR_DEPOSITION);     //open the door

  if (depositionState == 0)         //1st iteration: stop, start pusher
  {
    left_speed = 0;
    right_speed = 0;
    depositionState = 1;
    PusherTask.enable();
  }
  else if (depositionState == 1)    //waiting for pusher to finish
  {    //Pusher is moving. wait for resetPusher to change depositionState to 2 when pusher is done
    if (depositionShake % 4 < 2)
    {
      left_speed = 230;
      right_speed = -240;
    }
    else
    {
      left_speed = -230;
      right_speed = 220;
    }
    depositionShake++;
  }
  else if (depositionState == 2)    //once pusher is done, go backwards for
  {
    pusherState = 0;
    left_speed = -240;
    right_speed = -240;
    setSpeeds_I2C(left_speed, right_speed);

    depositionTimeoutCounter++;
    if (depositionTimeoutCounter >= DEPOSITION_DELAY)
      depositionState = 3;
  }
  else if (depositionState == 3)    //once done going backwards, stop, and finish deposition manoeuvre
  {
    depositionTimeoutCounter = 0;
    left_speed = 0;
    right_speed = 0;
    isFull = false;
    depositionState = 0;
    robotState = GOING_TO_WAYPOINT;
    //currentWaypoint++;
    Serial.println("*****************************Deposition Done******************************");

    if (!FullTask.isEnabled())  //if full task isn't already enabled
    {
      FullTask.enableDelayed(FULL_DELAY);
    }
    if (GeorgeGoHomeItsBeenTooLongTask.isEnabled())                             //if goHomeItsBeenTooLongTask is already enabled, restart it in 3 minutes
      GeorgeGoHomeItsBeenTooLongTask.restartDelayed(ITS_BEEN_TOO_LONG_INT);
    else
      GeorgeGoHomeItsBeenTooLongTask.enableDelayed(ITS_BEEN_TOO_LONG_INT);      //if not already enabled, enable it

  }
}

/*
  void DepositionTimeout()
  {
  depositionState = 3;
  DepositionTimeoutTask.disable();
  }
*/

//Change door state according to wished state
void DymxDoor_setState(int stateDoor)
{
  switch (stateDoor)
  {
    case DOOR_OPEN:
      if (doorState != DOOR_OPEN)         //open the door
      {
        DoorMoveTask.disable(); //stop door from moving
        DymxDoor_moveToInit();  //open the door
        doorState = DOOR_OPEN;
      }
      break;
    case DOOR_CLOSE:
      if (doorState != DOOR_CLOSE)         //close the door
      {
        DoorMoveTask.disable(); //stop door from moving
        DymxDoor_setPosition(DOOR_CLOSE_POS);       //close the door
        doorState = DOOR_CLOSE;
      }
      break;
    case DOOR_MOVE:
      if (doorState != DOOR_MOVE)
      {
        DoorMoveTask.enableIfNot();  //start moving the door
        doorState = DOOR_MOVE;
      }
      break;
    case DOOR_DEPOSITION:
      if (doorState != DOOR_DEPOSITION)
      {
        DoorMoveTask.disable(); //stop door from moving
        DymxDoor_setPosition(DOOR_DEPOSITION_POS);       //close the door
        doorState = DOOR_DEPOSITION;
      }
      break;
  }
}

/*
  void tCaptureBottle()
  {
  gotBottle = false;
  CaptureBottleTask.disable();
  robotState == GOING_TO_WAYPOINT;
  }
*/
//if cannot get to waypoint too long, leave it
void timeoutWaypoint()
{
  if (TimeoutWaypointTask.isFirstIteration() == false)
  {
    Serial.println("*****************************************");
    Serial.print("I cannot get to waypoint number ");
    Serial.println((int)currentWaypoint);
    Serial.println("Fuck this shit, moving on!");
    Serial.println("*****************************************");

    gotWaypoint = 2;
    TimeoutWaypointTask.disable();
  }
}
/*
  void evasiveManoeuvre()
  {
  if (EvasiveManoeuvreTask.isFirstIteration() == false)
  {
    Serial.println("dkfieuhfiuehfbcdviu done");
    blockedFlag = 0;
    EvasiveManoeuvreTask.disable();
  }
  else
  Serial.println("hhtffgddtrvytf ytdftd");
  }
*/

/*//Communicates with PI
  void get_info_from_pi()
  {
  coord newBottle;
  //communicate
  //
  //if succesful com
  //newBottle.x = robotPosition[0] + bottle.x;
  //newBottle.y = robotPosition[1] + bottle.y;
  // insertBottle(newBottle)
  //*communicate = true;
  }
*/

void goHomeItsTooLate()
{
  Serial.println("*****************************************");
  Serial.println("GeorgeGoHomeItsTooLate");
  Serial.println("*****************************************");
  isFull = true;
}
void goHomeItsBeenTooLong()
{
  Serial.println("*****************************************");
  Serial.println("GeorgeGoHomeItsBeenTooLong");
  Serial.println("*****************************************");
  isFull = true;
}

void stopRobot()
{
  left_speed = right_speed = 0;
}


//Start emptying manoeuvre using task scheduler
void DymxPusher_EmptyBottles_Task()
{
    //Serial.println("---- Pusher task start ----");
  switch(pusherState)
  {
    case 0:
      Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, PUSHER_SPEED);
      //PusherTask.setInterval(9*TASK_SECOND);
      pusherState++;
      break;
    case 1:
      Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, PUSHER_SPEED);
      //PusherTask.setInterval(9*TASK_SECOND);
      pusherState++;
      depositionState = 2;  
      setSpeeds_I2C(-200,-200);
      break;
    /*case 2:
      Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, PUSHER_SPEED);
      PusherTask.setInterval(7500);
      pusherState++;
      break;
    case 3:
      Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, PUSHER_SPEED);
      PusherTask.setInterval(7500);
      pusherState++;
      break;
    case 4:
      Dynamixel.turn(DYMX_PUSHER_ID , FORWARD, PUSHER_SPEED);
      PusherTask.setInterval(10*TASK_SECOND);
      pusherState++;
      break;
    case 5:
      Dynamixel.turn(DYMX_PUSHER_ID , BACKWARD, PUSHER_SPEED);
      PusherTask.setInterval(10*TASK_SECOND);
      pusherState++;
      break;*/
    case 2:
      DymxPusher_Reset();
      //PusherTask.setInterval(10*TASK_SECOND);
      //resetPusher = false;
      //if (depositionState == 1)
      //  depositionState = 2;  
      break;
    default:
      DymxPusher_Reset();
      //PusherTask.setInterval(10*TASK_SECOND);
      //resetPusher = false;
      break;
  }
  /*

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
    }*/
}

