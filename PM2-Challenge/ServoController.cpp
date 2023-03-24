#include "ServoController.h"
#include <cstdio>

const double min_angle = 0;
const double max_angle = 360;
const double max_angle_speed = 0.001;
const double allowed_servo_offset = 0.1;

ServoController::ServoController(Motion *motion_planner, Servo *servo) {
  _servo = servo;
  _motion_planner = motion_planner;
  _run_thread.start(callback(this, &ServoController::run));
  _last_micros = -1;
}

void ServoController::move_to(double angle_in_deg) {
  _desired_angle = angle_in_deg;
}

void ServoController::init() {
  _motion_planner->set(0, 0);
  _desired_angle = 10;
}

bool ServoController::is_on_desired_position() {
  bool is_on_desired_position =
      fabs(_desired_angle - _motion_planner->position) < allowed_servo_offset;
  return is_on_desired_position;
}

void ServoController::wait_until_idle() {
  while (!is_on_desired_position()) {
    ThisThread::sleep_for(100ms);
  }
}

double ServoController::calculate_new_angle() {
  using namespace std::chrono;

  auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
  long current_micros = now_ms.time_since_epoch().count();

  long micros_delta = current_micros - _last_micros;
  _last_micros = current_micros;

  float seconds_delta = (float)micros_delta / 1000000;

  _motion_planner->incrementToPosition(_desired_angle, seconds_delta);
  return _motion_planner->getPosition();
}

double ServoController::angle_to_normalised_angle(double angle) {
  if (angle > max_angle)
    return 1;
  if (angle < min_angle)
    return 0;

  double delta = max_angle - min_angle;
  double normalised = (angle - min_angle) / delta;

  return normalised;
}

void ServoController::run() {
  while (true) {
    if (!is_on_desired_position()) {
      // enable the servo if it is not enabled
      if (!_servo->isEnabled()) {
        _servo->enable();
      }

      double angle = calculate_new_angle();
      double normalised_angle = angle_to_normalised_angle(angle);

      printf("Angle n: %f \n", normalised_angle);

      _servo->setNormalisedAngle(normalised_angle);
    } else {
      // disable the servo if it is not disabled
      if (_servo->isEnabled()) {
        _servo->disable();
      }
    }

    // run approximately every 10ms
    ThisThread::sleep_for(50ms);
  }
}