#ifndef ROBOT_H
#define ROBOT_H

#include "PositionController.h"
#include "ServoController.h"
#include "mbed.h"
#include "rtos.h"

class Robot {
public:
  Robot(ServoController *servo_joint_front, ServoController *servo_joint_back,
        PositionController *position_controller_front,
        PositionController *position_controller_back);

  void init();

  void drive(double distance_in_mm);

  bool isIdle();

  void standUp();
  void bowForward();
  void bowBackward();
  void sitDown();

private:
  enum States {
    NotReady,
    Initializing,
    Idle,
    StartChangingPose,
    ChangingPose,
    StartDriving,
    Driving,
  };

  States _state;
  bool _initialize;
  double _commanded_angle_front;
  double _commanded_angle_back;

  double _last_tire_distance;

  double _commanded_relative_movement;

  double LENGTH_ARM_FRONT = 70;
  double LENGTH_ARM_BACK = 70;
  double LENGTH_ARM_CENTER = 83;
  double TIRE_RADIUS = 40;

  ServoController *_servo_joint_front;
  ServoController *_servo_joint_back;
  PositionController *_position_controller_back;
  PositionController *_position_controller_front;

  double getCurrentTireDistance();
  double calculateTireDistance(double length_arm_center, double length_arm_back,
                               double length_arm_front, double angle_front_deg,
                               double angle_back_deg);

  double calculateTireRotation(double distance);

  bool onPose();
  bool onPosition();

  Thread _run_thread;
  void run();
};

#endif