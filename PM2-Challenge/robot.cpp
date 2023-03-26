#include "robot.h"

Robot::Robot(ServoController *servo_joint_front,
             ServoController *servo_joint_back) {
  _servo_joint_front = servo_joint_front;
  _servo_joint_back = servo_joint_back;

  // start background thread
  _run_thread.start(callback(this, &Robot::run));
}



void Robot::run() {
  while (true) {
  }
}
