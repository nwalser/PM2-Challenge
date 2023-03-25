#ifndef ROBOT_H
#define ROBOT_H

#include "ServoController.h"
#include "mbed.h"
#include "rtos.h"

class Robot {
public:
  Robot(ServoController *servo_joint_front, ServoController *servo_joint_back);

  void MoveForward(double distance_in_mm);
  void MoveBackward(double distance_in_mm);
  void MoveJointFront(double angle_in_deg, bool move_tires = true);
  void MoveJointBack(double angle_in_deg, bool move_tires = true);

private:
  ServoController *_servo_joint_front;
  ServoController *_servo_joint_back;
  Thread _run_thread;
  void Run();
};

#endif