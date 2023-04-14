#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
#include "rtos.h"

class Sensor {
public:
<<<<<<< Updated upstream
Sensor();
double calculate_distance_in_mm_front();
double calculate_distance_in_mm_back();



private:
/*
//Coefficents Sensor 1
double a_front= 1.211e04;
double b_front= 98.69;

//Coefficents Sensor 2
double a_back= 1.175e04;
double b_back= 25.98;

double mV_sensor=0.0;
*/

  Thread _run_thread;
  void run();
};

#endif