#ifndef ROBOT_H
#define ROBOT_H

#include "ServoController.h"
#include "mbed.h"
#include "rtos.h"

class Robot {
public:
  Robot(ServoController *servo_joint_front, ServoController *servo_joint_back);

  void drive(double distance_in_mm);

  void standUp();
  void bendForward();
  void bendBackward();
  void flatOut();

private:
  ServoController *_servo_joint_front;
  ServoController *_servo_joint_back;
  Thread _run_thread;
  void run();
};

#endif