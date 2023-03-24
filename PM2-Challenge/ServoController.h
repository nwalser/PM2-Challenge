#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Map.h"
#include "Motion.h"
#include "Servo.h"
#include "TimeDelta.h"

class ServoController {
public:
  ServoController(Servo *servo, Motion *motion_planner, Map *angle_map);
  void init();
  void move_to(double angle_in_deg);
  bool is_on_position();
  bool is_initialized();

private:
  bool _initialized;
  bool _position_reached;
  double _desired_angle;
  const double ALLOWED_SERVO_OFFSET = 0.1;

  Servo *_servo;
  Motion *_motion_planner;
  Map *_angle_map;
  TimeDelta _time_delta;

  Thread _run_thread;

  void run();
};

#endif