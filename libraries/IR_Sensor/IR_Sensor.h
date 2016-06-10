#ifndef IR_SENSOR_H
#define IR_SENSOR_H
 
#include <Arduino.h>
 
class IRSensor {
public:
        IRSensor(int pin);
        ~IRSensor();
        int calc_distanceIR();
private:
        int _pin;
		int _num;
};
 
#endif
