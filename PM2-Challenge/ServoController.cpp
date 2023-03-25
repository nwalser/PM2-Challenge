#include "ServoController.h"
#include <cstdio>

ServoController::ServoController(Servo *servo, Motion *motion_planner,
                                 Map *angle_map) {
  _servo = servo;
  _motion_planner = motion_planner;
  _angle_map = angle_map;

  _initialize = false;
  _state = States::NotReady;

  _run_thread.start(callback(this, &ServoController::Run));
}

void ServoController::SetAngle(double angle_in_deg) {
  _desired_angle = angle_in_deg;

  // wait for 10ms to be sure that the state machine got the chance to flip to
  // the correct state
  ThisThread::sleep_for(10ms);
}

void ServoController::Init(double angle_in_deg) {
  _init_angle = angle_in_deg;
  _initialize = true;
  
  // wait for 10ms to be sure that the state machine got the chance to flip to
  // the correct state
  ThisThread::sleep_for(10ms);
}

bool ServoController::IsIdle() { return _state == States::Idle; }

bool ServoController::OnPosition() {
  return fabs(_desired_angle - _motion_planner->position) <
         ALLOWED_SERVO_OFFSET;
}

void ServoController::Run() {
  while (true) {
    switch (_state) {
    case States::NotReady: {
      if (_initialize) {
        _state = States::Initializing;
      }
      break;
    }

    case States::Initializing: {
      _motion_planner->set(0, 0);
      _desired_angle = 0;
      double normalised_angle = _angle_map->MapValue(0);

      _servo->enable();
      _servo->setNormalisedAngle(normalised_angle);

      // wait because we dont know where the servo is
      ThisThread::sleep_for(500ms);
      _servo->disable();

      _state = States::Idle;
      break;
    }

    case States::Idle: {
      if (!OnPosition()) {
        _state = States::StartMoving;
      }
      break;
    }

    case States::StartMoving: {
      _servo->enable();
      _time_delta.Reset();

      _state = States::Moving;
      break;
    }

    case States::Moving: {
      // calculate new angle and apply
      double seconds_delta = _time_delta.GetSecondsDelta();
      _motion_planner->incrementToPosition(_desired_angle, seconds_delta);
      double angle = _motion_planner->getPosition();
      double normalised_angle = _angle_map->MapValue(angle);
      _servo->setNormalisedAngle(normalised_angle);

      if (OnPosition()) {
        _state = States::StopMoving;
      }
      break;
    }

    case States::StopMoving: {
      _servo->disable();

      _state = States::Idle;
      break;
    }
    }

    // run approximately every 5ms
    ThisThread::sleep_for(5ms);
  }
}