#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Map.h"
#include "Motion.h"
#include "Servo.h"
#include "TimeDelta.h"

class ServoController {
public:
  ServoController(Servo *servo, Motion *motion_planner, Map *angle_map);
  void init(double angle_in_deg = 0);
  void setAngle(double angle_in_deg);
  double getCurrentAngle();
  bool isIdle();

private:
  enum States {
    NotReady,
    Idle,
    StartMoving,
    Moving,
    StopMoving,
    Initializing,
  };

  States _state; 
  double _desired_angle;
  double _init_angle;
  bool _initialize;

  Servo *_servo;
  Motion *_motion_planner;
  Map *_angle_map;
  TimeDelta _time_delta;

  Thread _run_thread;

  bool onAngle();
  void run();
};

#endif