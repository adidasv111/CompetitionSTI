/**
    C++
    US_Sensor.cpp
    Purpose: Calculate and manage ultrasonic sensors

    @author Sean Thomas
    @version 1.0 3/6/16 
*/

#include "US_Sensor.h"

#define filterSamples 13

/**
    A UltraSound sensor constructor

    @param trigPin Pin value for the sensor trigger
	@param echoPin Pin value for the sensor echo
    @return UltraSound sensor structure
*/
USSensor::USSensor(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT); //Set PIN as Output;
  _trigPin = trigPin;
  _echoPin = echoPin;
}

/**
    A UltraSound sensor destructor
*/
USSensor::~USSensor(){
/*Nothing to destruct*/
}

/**
    Digital Smoothing filter for the ultrasound

    @param rawIn Raw input of the sensor value
	@param sensSmoothArray array to store the filtered values
    @return filtered value of the sensor
*/
int digitalSmoothUS(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

/*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}

/**
    Converts microseconds value of the sensor to centimeters

    @param microseconds Value of the sensor in microseconds
    @return value of the US sensor in centimeters
*/
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

/**
    Calculates the distance of the Ultrasonic Sensor

    @return distance calculated by the sensor
*/
long USSensor::calc_distanceUS()
{
	static int sensSmoothArrayUS [filterSamples];
	static float filt_duration = 0;
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;
  

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(_echoPin, INPUT);
  duration = pulseIn(_echoPin, HIGH);
  filt_duration += 0.1*(duration - filt_duration);

  // convert the time into a distance
  //inches = microsecondsToInches(duration);
  //return cm = microsecondsToCentimeters(digitalSmoothUS(filt_duration,sensSmoothArrayUS));
  return cm = microsecondsToCentimeters(duration);
}

