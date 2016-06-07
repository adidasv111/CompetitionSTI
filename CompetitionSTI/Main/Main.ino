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

void pi_communication()
{
  get_info_from_pi(&pet_bottle.x, &pet_bottle.y, &pi_com);
}

//Tasks
Task DoorTask(750, TASK_FOREVER, &tDoor);                           //Create task that moves the door back and forth
Task PusherResetTask (20000, TASK_FOREVER, &DymxPusher_checkReset); //Create task that checks if pusher is reseted
Task OdometryTask(100, TASK_FOREVER, &calcOdometry);                //Create task that is called every 100ms and last forever to calculate odometry
//Task MotorsTask(100, TASK_FOREVER, &tMotors);                     //Create task that  set the motors
Task FullTask(2000, TASK_FOREVER, &checkFull);                      //Create task that check if full
Task PiComTask(2000, TASK_FOREVER, &pi_communication);              //Create task that communicate with the PI
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

  //map_array[1][1] = 0;
  runner.init();
  runner.addTask(OdometryTask);
  OdometryTask.enable();
  runner.addTask(DoorTask);
  DoorTask.enable();
  runner.addTask(PusherResetTask);
  PusherResetTask.enable();
}

/*****************************LOOP*************************/
void loop() {
  // put your main code here, to run repeatedly:

  runner.execute();
  //calcOdometry(); //Update robot position
  robotPos.x = robotPosition[0];
  robotPos.y = robotPosition[1];

  //careful about the pi values that are returned (to review)
  get_info_from_pi(&pet_bottle.x, &pet_bottle.y, &pi_com);
  if (pi_com)
    set_map_value_from_pos(pet_bottle, PET);

  planning();
  //  compute_wheel_speeds_coord(destination, &left_speed, &right_speed);

  //obstacle_avoidance(&left_speed, &right_speed);
  /* Serial.print(left_speed);
       Serial.print("   ");
    Serial.print(right_speed);
       Serial.print("   ");
         Serial.println(" ");*/
  //setSpeeds(left_speed, right_speed);
  set_map_value_from_pos(robotPos, ROBOT);
}


void planning()
{
  if (isFull) //Container is full, begin deposition process
  {
    //state= GO_HOME;
    destination.x = RECYCLE_ZONE_X;
    destination.y = RECYCLE_ZONE_Y;
    goingHome = true;
  }
  if (goingHome)
  {
    if (abs(robotPos.x - RECYCLE_ZONE_X) < 0.5 && abs(robotPos.y - RECYCLE_ZONE_Y) < 0.5)
    {
      goingHome = false;
      isDeposition = true;
    }
  }
  else if (isDeposition)
  {
    deposition(&left_speed, &right_speed);
  }
  else
  {
    if (check_target() == false) //Is target already present
    {
      if (find_number_bottles() != 0) //Target not present, does map contain bottles
      {
        //Bottle has been found, set as new target and destination
        destination = find_closest_bottle(robotPos);
        //state = GO_TO_BOTTLE;
        //target=true;
        set_target(destination);
      }
      else
      {
        grid_search(&destination.x, &destination.y); //No target, No bottles, continue grid search
        set_target(destination);
      }
    }
    else //Target already present, robot must continue tragectory towards target
    {
      //state = GO_TO_BOTTLE;
      //target = true;
    }
  }
}
