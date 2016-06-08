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

bool goingHome = false;
bool isDeposition = false;

//Headers for functions
void pi_communication();
void deposition();
void planning();
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
}

//Tasks
Task PlanningTask(100, TASK_FOREVER, &planning);
Task OdometryTask(100, TASK_FOREVER, &calcOdometry);                //Create task that is called every 100ms and last forever to calculate odometry
Task PiComTask(1000, TASK_FOREVER, &pi_communication);              //Create task that communicate with the PI

Task DoorTask(750, TASK_FOREVER, &tDoor);                           //Create task that moves the door back and forth
Task PusherTask(PUSHER_HALF_PERIOD, 2, &DymxPusher_EmptyBottles_Task);                //Create task that moves the pusher back and forth once
Task PusherResetTask (PUSHER_RESET_PERIOD, TASK_FOREVER, &DymxPusher_checkReset); //Create task that checks if pusher is reseted

//Task MotorsTask(100, TASK_FOREVER, &tMotors);                     //Create task that  set the motors
Task FullTask(2000, TASK_FOREVER, &checkFull);                      //Create task that check if full

Task DepositionTask(500, TASK_FOREVER, &deposition);
Task PrintTask(1000, TASK_FOREVER, &tprint);

Scheduler runner;

/*
  void tMotors()
  {
    calcOdometry();
    //TESTING
    /*long dist = US6.calc_distanceUS();
      Serial.print(dist);
      Serial.print(" ");
      Serial.println(" ");*/
/*left_speed = 170;
  right_speed = 170;

  //obstacle_avoidance(&left_speed, &right_speed);
  setSpeeds_I2C(left_speed, right_speed);
  }
  /
  /*****************************SETUP*************************/
void setup() {
  Serial.begin(9600);
  //init_map();
  initCompass_Serial2();
  initOdometry();
  init_map();
  initMotors_I2C();
  //initIMU(0);
  initDynamixels();       //Already includes DymxPusher_Reset() and DymxDoor_Reset()

  //calibrateCompass_Serial2();

  //map_array[1][1] = 0;
  runner.init();
  runner.addTask(OdometryTask);
  runner.addTask(DoorTask);
  runner.addTask(PusherTask);
  runner.addTask(PusherResetTask);
  runner.addTask(DepositionTask);
  runner.addTask(PlanningTask);
  runner.addTask(PrintTask);

  //PlanningTask.enable();
  OdometryTask.enable();
  //DoorTask.enable();
  //PusherTask.enable();
  //PusherResetTask.enable();
  PrintTask.enable();
}

/*****************************LOOP*************************/
void loop() {
  // put your main code here, to run repeatedly:

  runner.execute();
  //calcOdometry(); //Update robot position
  robotPos.x = robotPosition[0];
  robotPos.y = robotPosition[1];

  //  compute_wheel_speeds_coord(destination, &left_speed, &right_speed);

  //obstacle_avoidance(&left_speed, &right_speed);
  /* Serial.print(left_speed);
       Serial.print("   ");
    Serial.print(right_speed);
       Serial.print("   ");
         Serial.println(" ");*/
  /*if(robotPos.y < 3000)
    {
     //left_speed = 200;
    //right_speed = 200;
    }
    else
    {
     left_speed = 0;
    right_speed = 0;
    }*/
  left_speed = 200;
  right_speed = 200;
  destination.x = -1500;
  destination.y = -1500;
  compute_waypoint_speeds_coord(robotPosition, destination, &left_speed, &right_speed);
  if (gotGoal)
  {
    left_speed = 0;
    right_speed = 0;
  }
  setSpeeds_I2C(left_speed, right_speed);
  //TESTING

  // obstacle_avoidance(&left_speed, &right_speed);
  //set_map_value_from_pos(robotPos, ROBOT);
}

void planning()
{
  if (isFull & !goingHome) //Container is full, begin deposition manoeuvre
  {
    //state= GO_HOME;
    destination.x = RECYCLE_ZONE_X;
    destination.y = RECYCLE_ZONE_Y;
    goingHome = true;
    if (doorState != 1)
    {
      DoorTask.disable();
      DymxDoor_close();
      doorState = 1;
    }
  }
  else if (goingHome)
  {
    if (abs(robotPos.x - RECYCLE_ZONE_X) < 0.5 && abs(robotPos.y - RECYCLE_ZONE_Y) < 0.5)
    {
      goingHome = false;
      isFull = false;
      isDeposition = true;
      DepositionTask.enable();
    }
  }
  else if (isDeposition)
  {

  }
  else
  {
    if (check_target() == false) //If target isn't present
    {
      if (find_number_bottles() != 0) //Target not present, does map contain bottles
      {
        //Bottle has been found, set as new target and destination
        destination = find_closest_bottle(robotPos);
        //state = GO_TO_BOTTLE;
        //target=true;
        compute_wheel_speeds_coord(destination, &left_speed, &right_speed);
        set_target(destination);
      }
      else
      {
        if (doorState != 1)
        {
          DoorTask.disable();
          DymxDoor_close();
          doorState = 1;
        }
        grid_search(&destination.x, &destination.y); //No target, No bottles, continue grid search
        set_target(destination);
      }
    }
    else //Target already present, robot must continue tragectory towards target
    {
      //state = GO_TO_BOTTLE;
      //target = true;
      if (doorState != 2)
      {
        DoorTask.enable();
        doorState = 2;
      }
    }
  }
}


void pi_communication()
{
  get_info_from_pi(&pet_bottle.x, &pet_bottle.y, &pi_com);
  if (pi_com)
  {
    set_map_value_from_pos(pet_bottle, PET);
  }
  pi_com = false;
}

void deposition()
{
  if (doorState != 0)
  {
    DoorTask.disable();
    DymxDoor_moveToInit();
    doorState = 0;
  }

  if (depositionState == 0)
  {
    left_speed = 0;
    right_speed = 0;
    setSpeeds_I2C(left_speed, right_speed);    //************************
    PusherTask.enable();
    depositionState = 1;
  }
  else if (depositionState == 2)
  {
    left_speed = -255;
    right_speed = -255;
    setSpeeds_I2C(left_speed, right_speed);    //************************
    depositionState = 3;
  }
  else if (depositionState == 3)
  {
    left_speed = 0;
    right_speed = 0;
    setSpeeds_I2C(left_speed, right_speed);     //************************
    depositionState = 0;
    isDeposition = false;
    isFull = false;
    DepositionTask.disable();
  }
}
