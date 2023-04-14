/*

#include "sensor.h"
#include <cmath>
#include <cstdio>
#include <math.h>


Sensor::Sensor(){

    _run_thread.start(callback(this, &Sensor::run));
}

//Coefficents Sensor 1
double a_front= 1.211e04;
double b_front= 98.69;

//Coefficents Sensor 2
double a_back= 1.175e04;
double b_back= 25.98;

double mV_sensor=0.0;


double calculate_distance_in_mm_front(mV_sensor){
return (a_front/(b_front+mV_sensor))
};

double calculate_distance_in_mm_back(mV_sensor){
return (a_back/(b_back+mV_sensor))
};




//if (mechanical_button.read()){
        //printf("IR sensor (mV): %3.3f\r\n",
               //ir_distance_mV);}


*/