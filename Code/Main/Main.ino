#include <Motors.h>
#include <Odometry.h>
#include <mapping.h>
#include <braitenberg.h>
#include <US_Sensor.h>
#include <IR_Sensor.h>
#include <gridSearch.h>
#include <Arduino.h>
#include <constant.h>
#include <TaskScheduler.h>

//Global Varibles
int state = 0;
IRSensor IR1(A1);
USSensor US_test(30, 31);

//Ultrasound sensor to detect bottles that enter the robot
USSensor US_detection(32, 33);

//Coordinates for the current destination of the robot
coord destination;
//Current coordinates of the robot
coord robotPos;
int left_speed = 0, right_speed = 0;
char capacity = 0;

Scheduler runner;
void tOdometry();
Task OdometryTsk(100, TASK_FOREVER, &tOdometry); //Create task that is called every 100ms and last forever to call function tOdometry

void setup() {
  //put your setup code here, to run once:
  // declare the ledPin as an OUTPUT:
  
  Serial.begin(9600);
  init_map();
  initOdometry();
  initMotors();
  //map_array[1][1] = 0;
  runner.init();
  runner.addTask(OdometryTsk);
  OdometryTsk.enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  long USdist;
  USdist = US_test.calc_distanceUS();
  /*Serial.print(USdist);
    Serial.print("   ");
      //Serial.println(" ");*/
  float dist;
  dist = IR1.calc_distanceIR();
  Serial.print(dist);
    Serial.print("   ");
      Serial.println(" ");

  runner.execute();
  //calcOdometry(); //Update robot position
  robotPos.x = robotPosition[0];
  robotPos.y = robotPosition[1];
  coord pet_bottle;

  //careful about the pi values that are returned (to review)
  if(get_info_from_pi(&pet_bottle.x, &pet_bottle.y))
    set_map_value_from_pos(pet_bottle, PET);
  
  planning();
  compute_wheel_speeds_coord(destination, &left_speed, &right_speed);
  obstacle_avoidance(&left_speed, &right_speed);
  setSpeeds(left_speed, right_speed);
  set_map_value_from_pos(robotPos, ROBOT);
}


void planning()
{
  if(Full()) //Container is full, begin deposition process
  {
    //state= GO_BACK;
    destination.x = RECYCLE_ZONE_X;
    destination.y = RECYCLE_ZONE_Y;
  }
  else
  {
    if(check_target() == false) //Is target already present
    {
      if(find_number_bottles() != 0) //Target not present, does map contain bottles
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


bool get_info_from_pi(float *bottle_x, float *bottle_y)
{
  return true;
}

bool Full()
{
  int detector_dist = US_detection.calc_distanceUS();
  if(detector_dist < 15)
    capacity++;

  if(capacity >= 3)
    return true;
  else
    return false;
}

void tOdometry()
{
  if(OdometryTsk.isFirstIteration())
  {
    initOdometry();
  }
  else
  {
    calcOdometry();
  }
}

