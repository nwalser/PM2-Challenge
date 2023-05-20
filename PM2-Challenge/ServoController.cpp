#include "ServoController.h"
#include <cstdio>

#define EPS 0.001

ServoController::ServoController(Servo *servo, Motion *motion_planner,
                                 Map *angle_map) {
  _servo = servo;
  _motion_planner = motion_planner;
  _angle_map = angle_map;

  _initialize = false;
  _state = States::NotReady;

  _run_thread.start(callback(this, &ServoController::run));
}

void ServoController::setAngle(double angle_in_deg) {
  _desired_angle = angle_in_deg;

  // wait for 10ms to be sure that the state machine got the chance to flip to
  // the correct state, should be synced with thread to be 100% save, TODO
  ThisThread::sleep_for(10ms);
}

void ServoController::init(double angle_in_deg) {
  _init_angle = angle_in_deg;
  _initialize = true;

  // wait for 10ms to be sure that the state machine got the chance to flip to
  // the correct state, should be synced with thread to be 100% save, TODO
  ThisThread::sleep_for(10ms);
}

bool ServoController::isIdle() { return _state == States::Idle; }

bool ServoController::onAngle() {
  return fabs(_desired_angle - _motion_planner->position) < EPS;
}

double ServoController::getCurrentAngle() {
  return _motion_planner->getPosition();
}

void ServoController::run() {
  while (true) {
    switch (_state) {
    case States::NotReady: {
      if (_initialize) {
        _state = States::Initializing;
      }
      break;
    }

    case States::Initializing: {
      _initialize = false;

      _motion_planner->set(0, 0);
      _desired_angle = 0;
      double normalised_angle = _angle_map->mapValue(0);

      _servo->enable();
      _servo->setNormalisedAngle(normalised_angle);

      // wait because we don't know where the servo is
      ThisThread::sleep_for(500ms);
      //_servo->disable();

      _state = States::Idle;
      break;
    }

    case States::Idle: {
      if (!onAngle()) {
        _state = States::StartMoving;
      }
      break;
    }

    case States::StartMoving: {
      //_servo->enable();
      _time_delta.reset();

      _state = States::Moving;
      break;
    }

    case States::Moving: {
      // calculate new angle and apply
      double seconds_delta = _time_delta.getSecondsDelta();
      _motion_planner->incrementToPosition(_desired_angle, seconds_delta);
      double angle = _motion_planner->getPosition();
      double normalised_angle = _angle_map->mapValue(angle);
      _servo->setNormalisedAngle(normalised_angle);

      if (onAngle()) {
        _state = States::StopMoving;
      }
      break;
    }

    case States::StopMoving: {
      //_servo->disable();

      _state = States::Idle;
      break;
    }
    }

    // run approximately every 20ms
    ThisThread::sleep_for(100ms);
  }
}