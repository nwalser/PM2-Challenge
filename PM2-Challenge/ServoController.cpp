#include "ServoController.h"
#include <cstdio>

ServoController::ServoController(Servo *servo, Motion *motion_planner,
                                 Map *angle_map) {
  _servo = servo;
  _motion_planner = motion_planner;
  _angle_map = angle_map;

  _run_thread.start(callback(this, &ServoController::run));
  _initialized = false;
}

void ServoController::move_to(double angle_in_deg) {
  _desired_angle = angle_in_deg;
  _position_reached = false;
}

void ServoController::init() {
  _motion_planner->set(0, 0);
  _desired_angle = 0;
  _initialized = true;
  _position_reached = false;
}

bool ServoController::is_on_position() { return _position_reached; }

bool ServoController::is_initialized() { return _initialized; }

void ServoController::run() {
  while (true) {
    if (!_position_reached) {
      // enable the servo if it is not enabled
      if (!_servo->isEnabled()) {
        _servo->enable();
      }

      // calculate new angle and apply
      double seconds_delta = _time_delta.get_second_delta();
      _motion_planner->incrementToPosition(_desired_angle, seconds_delta);
      double angle = _motion_planner->getPosition();
      double normalised_angle = _angle_map->map(angle);

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