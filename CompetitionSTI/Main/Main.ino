#include <constant.h>
#include <Motors.h>
#include <IMU.h>
#include <Pusher.h>
#include <Odometry.h>
#include <mapping.h>
#include <braitenberg.h>
#include <US_Sensor.h>
#include <IR_Sensor.h>
#include <gridSearch.h>
#include <Arduino.h>
#include <Misc.h>
#include <TaskScheduler.h>
//Global Varibles
int state = 0;

//USSensor US6(38, 39), US7(36,37);
//Coordinates for the current destination of the robot
coord destination;

//Current coordinates of the robot
coord robotPos;
int left_speed = 0, right_speed = 0;

coord pet_bottle;
bool pi_com;

char robotState = GOING_TO_WAYPOINT;
bool isCapuring = false;

bool goingHome = false;
bool isDeposition = false;

//----- Headers for functions -----
void pi_communication();
void deposition();
void DymxDoor_setState(int state);
void planning();
void tCaptureBottle();

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

//Tasks
Task PlanningTask(100, TASK_FOREVER, &planning);
Task OdometryTask(100, TASK_FOREVER, &calcOdometry);                                //Create task that is called every 100ms and last forever to calculate odometry
Task CaptureBottleTask(1000, 1, &tCaptureBottle);                                   //move forward over the bottle for 1sec when capturing

Task DoorMoveTask(DOOR_HALF_PERIOD, TASK_FOREVER, &tDoor);                          //Create task that moves the door back and forth
Task FullTask(2000, TASK_FOREVER, &checkFull);                                      //Create task that check if full
//Task DepositionTask(100, TASK_FOREVER, &deposition);                                //Deposition manoeuvre
Task PusherTask(PUSHER_HALF_PERIOD, 2, &DymxPusher_EmptyBottles_Task);              //Create task that moves the pusher back and forth once
Task PusherResetTask (PUSHER_RESET_PERIOD, TASK_FOREVER, &DymxPusher_checkReset);   //Create task that checks if pusher is reseted

Task PiComTask(500, TASK_FOREVER, &pi_communication);                               //Create task that communicate with the PI
Task PrintTask(1000, TASK_FOREVER, &tprint);

Scheduler runner;

//*****************************SETUP*************************/
void setup()
{
  Serial.begin(9600);
  //initCompass_Serial2();
  initOdometry();
  init_waypoints();
  init_bottlesArray();
  initMotors_I2C();
  initDynamixels();             //Already includes DymxPusher_Reset() and DymxDoor_Reset()
  //initIMU(0);
  //calibrateCompass_Serial2();

  destination.x = INIT_X;
  destination.y = INIT_Y;

  runner.init();
  runner.addTask(PlanningTask);
  runner.addTask(OdometryTask);
  runner.addTask(CaptureBottleTask);

  runner.addTask(DoorMoveTask);
  runner.addTask(FullTask);
  //runner.addTask(DepositionTask);
  runner.addTask(PusherTask);
  runner.addTask(PusherResetTask);

  runner.addTask(PiComTask);
  runner.addTask(PrintTask);

  PlanningTask.enable();
  OdometryTask.enable();
  FullTask.enable();
  PusherResetTask.enable();
  //PiComTask.enable();
  PrintTask.enable();
}

//***************************** LOOP *************************
void loop()
{
  runner.execute();
  robotPos.x = robotPosition[0];
  robotPos.y = robotPosition[1];
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
  left_speed = right_speed = 0;
  /**********************************/
  obstacle_avoidance(&left_speed, &right_speed);
  setSpeeds_I2C(left_speed, right_speed);
}

void deposition()                   //Deposition manoeuvre
{
  gotHome = false;
  DymxDoor_setState(DOOR_OPEN);     //open the door

  if (depositionState == 0)
  {
    left_speed = 0;
    right_speed = 0;
    //setSpeeds_I2C(left_speed, right_speed);    //************************
    PusherTask.enable();
    depositionState = 1;
  }
  else if (depositionState == 1)
  {
    left_speed = 0;
    right_speed = 0;
    //Pusher is moving. wait for resetPusher to change depositionState to 2 when pusher is done
  }
  else if (depositionState == 2)
  {
    left_speed = -240;
    right_speed = -240;
    //setSpeeds_I2C(left_speed, right_speed);    //************************
    depositionState = 3;
  }
  else if (depositionState == 3)
  {
    //DepositionTask.disable();
    left_speed = 0;
    right_speed = 0;
    //setSpeeds_I2C(left_speed, right_speed);     //************************
    isFull = false;
    depositionState = 0;
    robotState == GOING_TO_WAYPOINT;
  }
}

//Change door state according to wished state
void DymxDoor_setState(int state)
{
  switch (state)
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

void pi_communication()
{
  get_info_from_pi(&pet_bottle.x, &pet_bottle.y, &pi_com);
  if (pi_com)
  {
    //    set_map_value_from_pos(pet_bottle, PET);
  }
  pi_com = false;
}

void tCaptureBottle()
{
  gotBottle = false;
  CaptureBottleTask.disable();
  robotState == GOING_TO_WAYPOINT;
}
