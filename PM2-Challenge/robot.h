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
        PositionController *position_controller_back,
        DigitalOut *enable_motors);

  enum JointCorrectionMode {
    None,
    Front,
    Back,
    Both,
  };

  void init();

  void drive(double distance_in_mm);
  void driveWithBackTire(double distance_in_mm);

  bool isIdle();

  void standUp(JointCorrectionMode mode = JointCorrectionMode::Back);
  void bowForward(JointCorrectionMode mode = JointCorrectionMode::Back);
  void bowBackward(JointCorrectionMode mode = JointCorrectionMode::Back);
  void sitDown(JointCorrectionMode mode = JointCorrectionMode::Back);
  void setJointAngles(double back, double front,
                      JointCorrectionMode mode = JointCorrectionMode::Front);

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
  JointCorrectionMode _joint_correction_mode;

  bool _initialize;
  double _commanded_angle_front;
  double _commanded_angle_back;

  double _last_tire_distance;

  double _commanded_relative_movement;

  double LENGTH_ARM_FRONT = 110;
  double LENGTH_ARM_BACK = 110;
  double LENGTH_ARM_CENTER = 130;
  double TIRE_RADIUS = 27.5 * 1.15;

  ServoController *_servo_joint_front;
  ServoController *_servo_joint_back;
  PositionController *_position_controller_back;
  PositionController *_position_controller_front;
  DigitalOut *_enable_motors;

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