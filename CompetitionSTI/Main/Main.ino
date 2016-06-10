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

//----- Headers for functions -----
void planning();
void deposition();
void DymxDoor_setState(int stateDoor);
void get_info_from_pi();
void tCaptureBottle();
void goHomeItsTooLate();
void goHomeItsBeenTooLong();

void tprint()
{
  Serial.print("x:   ");
  Serial.println(robotPosition[0]);
  Serial.print("y:   ");
  Serial.println(robotPosition[1]);
  Serial.print(" theta:   ");
  Serial.println(robotPosition[2]);
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
  Serial.println(isFull);
}

//----- Tasks definitions -----
Task PlanningTask(100, TASK_FOREVER, &planning);
Task OdometryTask(100, TASK_FOREVER, &calcOdometry);                                //Create task that is called every 100ms and last forever to calculate odometry
Task CaptureBottleTask(1000, 1, &tCaptureBottle);                                   //move forward over the bottle for 1sec when capturing

Task DoorMoveTask(DOOR_HALF_PERIOD, TASK_FOREVER, &tDoor);                          //Create task that moves the door back and forth
Task FullTask(2000, TASK_FOREVER, &checkFull);                                      //Create task that check if full
//Task DepositionTask(100, TASK_FOREVER, &deposition);                              //Deposition manoeuvre
Task PusherTask(PUSHER_HALF_PERIOD, 2, &DymxPusher_EmptyBottles_Task);              //Create task that moves the pusher back and forth once
Task PusherResetTask(PUSHER_RESET_PERIOD, TASK_FOREVER, &DymxPusher_checkReset);    //Create task that checks if pusher is reseted

Task goHomeItsTooLateTask(9 * TASK_MINUTE + 30 * TASK_SECOND, 1, &goHomeItsTooLate);                        //go home after 9.30 minutes
Task goHomeItsBeenTooLongTask(3 * TASK_MINUTE, 1, &goHomeItsBeenTooLong);                  //go home if it's been 3 minutes since last deposition

Task PiComTask(500, TASK_FOREVER, &get_info_from_pi);                               //Create task that communicate with the PI
Task PrintTask(1000, TASK_FOREVER, &tprint);

Scheduler runner;

//*****************************SETUP*************************/
void setup()
{
  Serial.begin(9600);

  initOdometry();
  init_waypoints();
  init_bottlesArray();
  initMotors_I2C();
  initDynamixels();             //Already includes DymxPusher_Reset() and DymxDoor_Reset()
  //initIMU(0);
  //initCompass_Serial2();
  //calibrateCompass_Serial2();

  destination.x = INIT_X;
  destination.y = INIT_Y;

  //Adding all tasks to runner
  runner.init();
  runner.addTask(PlanningTask);
  runner.addTask(OdometryTask);
  runner.addTask(CaptureBottleTask);

  runner.addTask(DoorMoveTask);
  runner.addTask(FullTask);
  //runner.addTask(DepositionTask);
  runner.addTask(PusherTask);
  runner.addTask(PusherResetTask);

  runner.addTask(goHomeItsTooLateTask);
  runner.addTask(goHomeItsBeenTooLongTask);

  runner.addTask(PiComTask);
  runner.addTask(PrintTask);

  //enabling tasks that should start at beginnig of the programm
  PlanningTask.enable();
  OdometryTask.enable();
  FullTask.enable();
  PusherResetTask.enable();
  goHomeItsTooLateTask.enable();
  goHomeItsBeenTooLongTask.enable();
  //PiComTask.enable();
  PrintTask.enable();
}

//***************************** LOOP *************************
void loop()
{
  runner.execute();
}

//***************************** TASK FUNCTIONS *************************
void planning()
{
  /***********ADD TO TASK********/
  updateIRSensors();
  /******************************/

  left_speed = 200;
  right_speed = 200;
  if (isFull && (robotState != GOING_HOME) && (robotState != DEPOSITION)) //Container is full, start going home
  {
    robotState = GOING_HOME;              //state = GOING_HOME;
  }
  if (robotState == GOING_HOME)         //going home
  {
    DymxDoor_setState(DOOR_CLOSE);          //close the door when going home
    destination.x = HOME_X;
    destination.y = HOME_Y;
    compute_waypoint_speeds_coord(robotPosition, destination, &left_speed, &right_speed, robotState);  //compute speeds to go to bottle

    if (gotHome)                            //if got home
    {
      robotState = DEPOSITION;
      //DepositionTask.enable();
    }
  }
  else if (robotState == DEPOSITION)
  {
    destination.x = HOME_X;
    destination.y = HOME_Y;
    deposition();//DepositionTask is working. Wait for it to change robotState to GOING_TO_WAYPOINT when done.
  }
  else if (robotState == GOING_TO_WAYPOINT)
  {
    destination = findClosestBottle(robotPosition);
    if (destination.x != -1)    //new target found
    {
      robotState == GOING_TO_BOTTLE;
    }
    else        // no new target found
    {
      DymxDoor_setState(DOOR_CLOSE);          //close the door when going to waypoint
      destination.x = waypoints[currentWaypoint].x;
      destination.y = waypoints[currentWaypoint].y;
      compute_waypoint_speeds_coord(robotPosition, destination, &left_speed, &right_speed, robotState);  //compute speeds to go to waypoint
    }
  }
  else if (robotState == GOING_TO_BOTTLE)    //Target already present, robot must continue tragectory towards target
  {
    DymxDoor_setState(DOOR_MOVE);   //start moving door
    //destination is already set to target
    compute_bottle_speeds_coord(robotPosition, destination, &left_speed, &right_speed, robotState);  //compute speeds to go to waypoint

    if (gotBottle)
    {
      CaptureBottleTask.enableIfNot();
      left_speed = 255;
      right_speed = 255;
    }
  }
  if (currentWaypoint == 5)
  {
    isFull = true;
  }

  /*
    if (gotHome)
    {
      left_speed = 0;
      right_speed = 0;
    }
  */

  /**********TESTING*****************/
  //left_speed = right_speed = 0;
  /**********************************/
  obstacle_avoidance(&left_speed, &right_speed);
  setSpeeds_I2C(left_speed, right_speed);
}

void deposition()                   //Deposition manoeuvre
{
  gotHome = false;
  DymxDoor_setState(DOOR_OPEN);     //open the door

  if (depositionState == 0)         //1st iteration: stop, start pusher
  {
    left_speed = 0;
    right_speed = 0;
    PusherTask.enable();
    depositionState = 1;
  }
  else if (depositionState == 1)    //waiting for pusher to finish
  {
    left_speed = 0;
    right_speed = 0;
    //Pusher is moving. wait for resetPusher to change depositionState to 2 when pusher is done
  }
  else if (depositionState == 2)    //once pusher is done, go backwards for
  {
    left_speed = -240;
    right_speed = -240;
    depositionState = 3;
  }
  else if (depositionState == 3)    //once done going backwards, stop, and finish deposition manoeuvre
  {
    //DepositionTask.disable();
    left_speed = 0;
    right_speed = 0;
    isFull = false;
    depositionState = 0;
    robotState == GOING_TO_WAYPOINT;

    if (goHomeItsBeenTooLongTask.isEnabled())   //if goHomeItsBeenTooLongTask is enabled, restart it in 3 minutes
      goHomeItsBeenTooLongTask.restartDelayed(3 * TASK_MINUTE);
    else
      goHomeItsBeenTooLongTask.enable();
  }
}

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
        DymxDoor_close();       //close the door
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
  }
}


//Communicates with PI
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

void tCaptureBottle()
{
  gotBottle = false;
  CaptureBottleTask.disable();
  robotState == GOING_TO_WAYPOINT;
}

void goHomeItsTooLate()
{
  Serial.println("goHomeItsTooLate");
  isFull = true;
}
void goHomeItsBeenTooLong()
{
  Serial.println("goHomeItsBeenTooLong");
  isFull = true;
}
