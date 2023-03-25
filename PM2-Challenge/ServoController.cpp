#include "ServoController.h"
#include <cstdio>

ServoController::ServoController(Servo *servo, Motion *motion_planner,
                                 Map *angle_map) {
  _servo = servo;
  _motion_planner = motion_planner;
  _angle_map = angle_map;

  _initialized = false;
  _position_reached = false;

  _run_thread.start(callback(this, &ServoController::Run));
}

void ServoController::MoveTo(double angle_in_deg) {
  _desired_angle = angle_in_deg;
  _position_reached = false;
}

void ServoController::Init() {
  _motion_planner->set(0, 0);
  _desired_angle = 0;
  _initialized = true;
}

bool ServoController::IsOnPosition() { return _position_reached; }

bool ServoController::IsInitialized() { return _initialized; }

void ServoController::Run() {
  while (true) {
    if (!_position_reached) {
      // enable the servo if it is not enabled
      if (!_servo->isEnabled()) {
        _servo->enable();
      }

      // calculate new angle and apply
      double seconds_delta = _time_delta.GetSecondsDelta();
      _motion_planner->incrementToPosition(_desired_angle, seconds_delta);
      double angle = _motion_planner->getPosition();
      double normalised_angle = _angle_map->MapValue(angle);

      _servo->setNormalisedAngle(normalised_angle);

      // check if we reached destination position to disable servo
      _position_reached = fabs(_desired_angle - _motion_planner->position) <
                          ALLOWED_SERVO_OFFSET;
    } else {
      // disable the servo if it is not disabled
      if (_servo->isEnabled()) {
        _servo->disable();
      }
    }

    // run approximately every 5ms
    ThisThread::sleep_for(5ms);
  }
}