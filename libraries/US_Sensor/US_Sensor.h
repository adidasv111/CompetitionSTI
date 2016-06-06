#ifndef US_SENSOR_H
#define US_SENSOR_H
 
#include <Arduino.h>
 
class USSensor {
public:
        USSensor(int trigPin, int echoPin);
        ~USSensor();
        long calc_distanceUS();
private:
        int _trigPin;
        int _echoPin;
};
 
#endif
