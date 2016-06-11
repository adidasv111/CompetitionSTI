#include "IR_Sensor.h"
#include "constant.h"

int sensSmoothArray [N_SENSORS][filterSamples];   // array for holding raw sensor values for sensor1
int nbIRSensors = 0;



IRSensor::IRSensor(int pin){
  //pinMode(pin, OUTPUT); //Set PIN as Output;
  _pin = pin;
  _num = nbIRSensors;
  nbIRSensors++;
}

IRSensor::~IRSensor(){
/*Nothing to destruct*/
	nbIRSensors--;
}


int digitalSmoothIR(int rawIn, int *sensSmooth)
{     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
	
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmooth[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmooth[j];
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


float millivoltToCentimeters(long IRvalue)
{
  float volt_IRvalue = IRvalue*49.0/10000.0; //1 Unit is equal to 4.9mV

  if(volt_IRvalue >= 3) //Sensor minimum value is 7cm
    return 7.0;
  else if(volt_IRvalue <= 0.4) //Sensor maximum value is 80cm
    return 80.0;
  else
    return 1/(0.0501*volt_IRvalue - 0.0113); //Calculate distance based on the linear relationship provided in the sensor datasheet
}

int IRSensor::calc_distanceIR()
{
  int IRValue;
  IRValue = analogRead(_pin);
  int smoothData;
  smoothData = digitalSmoothIR(IRValue, sensSmoothArray[_num]);
  //return millivoltToCentimeters(smoothData);
  return millivoltToCentimeters(IRValue);
}
