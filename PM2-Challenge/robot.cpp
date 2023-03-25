#include "robot.h"

Robot::Robot(ServoController *servo_joint_front,
             ServoController *servo_joint_back) {
  _servo_joint_front = servo_joint_front;
  _servo_joint_back = servo_joint_back;

  // start background thread
  _run_thread.start(callback(this, &Robot::Run));
}

void Robot::MoveJointFront(double angle_in_deg, bool move_tires) {
  _servo_joint_front->SetAngle(angle_in_deg);
}

void Robot::Run() {
  while (true) {
  }
}
