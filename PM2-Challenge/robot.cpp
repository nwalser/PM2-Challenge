#include "robot.h"
#include <cmath>
#include <cstdio>
#include <math.h>

#define EPS 0.001
#define PI 3.14159265359

Robot::Robot(ServoController *servo_joint_front,
             ServoController *servo_joint_back,
             PositionController *position_controller_front,
             PositionController *position_controller_back,
             DigitalOut *enable_motors) {
  _servo_joint_front = servo_joint_front;
  _servo_joint_back = servo_joint_back;
  _position_controller_back = position_controller_back;
  _position_controller_front = position_controller_front;
  _enable_motors = enable_motors;

  _initialize = false;

  // start background thread
  _run_thread.start(callback(this, &Robot::run));
}

void Robot::init() {
  _initialize = true;

  ThisThread::sleep_for(10ms);
}

void Robot::standUp(JointCorrectionMode mode) {
  _joint_correction_mode = mode;
  _commanded_angle_back = 0;
  _commanded_angle_front = 0;

  ThisThread::sleep_for(10ms);
}

void Robot::sitDown(JointCorrectionMode mode) {
  _joint_correction_mode = mode;
  _commanded_angle_back = 90;
  _commanded_angle_front = 90;

  ThisThread::sleep_for(10ms);
}

void Robot::bowForward(JointCorrectionMode mode) {
  _joint_correction_mode = mode;
  _commanded_angle_back = 0;
  _commanded_angle_front = 135;

  ThisThread::sleep_for(10ms);
}

void Robot::bowBackward(JointCorrectionMode mode) {
  _joint_correction_mode = mode;
  _commanded_angle_back = 135;
  _commanded_angle_front = 0;

  ThisThread::sleep_for(10ms);
}

void Robot::setJointAngles(double back, double front,
                           JointCorrectionMode mode) {
  _joint_correction_mode = mode;
  _commanded_angle_back = back;
  _commanded_angle_front = front;

  ThisThread::sleep_for(10ms);
}

void Robot::drive(double distance_in_mm) {
  _commanded_relative_movement = distance_in_mm;

  ThisThread::sleep_for(10ms);
}

bool Robot::isIdle() { return _state == States::Idle; }

void Robot::run() {
  while (true) {
    switch (_state) {

    case States::NotReady: {
      if (_initialize) {
        _state = States::Initializing;
      }
      break;
    }

    case States::Initializing: {
      _servo_joint_front->init(0);
      _servo_joint_back->init(0);
      _position_controller_back->setDesiredRotation(0);
      _commanded_relative_movement = 0;
      _commanded_angle_back = 0;
      _commanded_angle_front = 0;

      _enable_motors->write(1);

      _state = States::Idle;
      break;
    }

    case States::Idle: {
      if (!onPosition()) {
        _state = States::StartDriving;
      }
      if (!onPose()) {
        _state = States::StartChangingPose;
      }
      break;
    }

    case States::StartChangingPose: {
      _last_tire_distance = getCurrentTireDistance();

      _position_controller_back->setMaxAccelerationRPS(1);
      _position_controller_front->setMaxAccelerationRPS(1);

      _state = States::ChangingPose;
      break;
    }

    case States::ChangingPose: {
      // feed positions into servo controllers, let them controll the angle
      _servo_joint_front->setAngle(_commanded_angle_front);
      _servo_joint_back->setAngle(_commanded_angle_back);

      // controll tires/tire as slaves of the position of the servos, yes the
      // tires will lag behind, but as long as the cycle time is low enough this
      // should not pose a problem.
      double current_tire_distance = getCurrentTireDistance();
      double tire_distance_delta = current_tire_distance - _last_tire_distance;
      _last_tire_distance = current_tire_distance;
      double delta_rotation = calculateTireRotation(tire_distance_delta);

      switch (_joint_correction_mode) {
      case JointCorrectionMode::None: {
        break;
      }
      case JointCorrectionMode::Back: {
        _position_controller_back->setDesiredRotationRelative(delta_rotation);
        break;
      }
      case JointCorrectionMode::Front: {
        _position_controller_front->setDesiredRotationRelative(-delta_rotation);
        break;
      }
      case JointCorrectionMode::Both: {
        _position_controller_back->setDesiredRotationRelative(delta_rotation /
                                                              2);
        _position_controller_front->setDesiredRotationRelative(-delta_rotation /
                                                               2);
        break;
      }
      }

      if (onPose()) {
        _state = States::Idle;
      }

      break;
    }

    case States::StartDriving: {
      double delta_rotation =
          calculateTireRotation(_commanded_relative_movement);
      _commanded_relative_movement = 0;

      _position_controller_back->setMaxAccelerationRPS(0.2);
      _position_controller_front->setMaxAccelerationRPS(0.2);

      _position_controller_back->setDesiredRotationRelative(delta_rotation);
      _position_controller_front->setDesiredRotationRelative(-delta_rotation);

      _state = States::Driving;
      break;
    }

    case States::Driving: {
      if (_position_controller_back->isIdle()) {
        _state = States::Idle;
      }
      break;
    }
    }

    ThisThread::sleep_for(5ms);
  }
}

bool Robot::onPose() {
  bool back_on_angle =
      fabs(_commanded_angle_back - _servo_joint_back->getCurrentAngle()) < EPS;
  bool front_on_angle = fabs(_commanded_angle_front -
                             _servo_joint_front->getCurrentAngle()) < EPS;

  return back_on_angle & front_on_angle;
}

bool Robot::onPosition() { return fabs(_commanded_relative_movement) < EPS; }

double Robot::calculateTireRotation(double distance) {
  double tire_circumference = TIRE_RADIUS * 2 * PI;
  double rotations = distance / tire_circumference;
  return rotations;
}

double Robot::getCurrentTireDistance() {
  return calculateTireDistance(LENGTH_ARM_CENTER, LENGTH_ARM_BACK,
                               LENGTH_ARM_FRONT,
                               _servo_joint_front->getCurrentAngle(),
                               _servo_joint_back->getCurrentAngle());
}

double Robot::calculateTireDistance(double length_arm_center,
                                    double length_arm_back,
                                    double length_arm_front,
                                    double angle_front_deg,
                                    double angle_back_deg) {
  // start on back tire with vector addition, take back arm as 0,0 on coordinate
  // system and fit coordinate unit vectors to back arm
  double xSum = 0;
  double ySum = 0;

  // first arm addition
  // take first arm as fixed
  double angle_first_arm_rad = 0;
  xSum += length_arm_back * cos(0);
  ySum += length_arm_back * sin(0);

  // second arm addition
  double angle_second_arm_rad =
      (angle_back_deg - 90) * PI / 180 + angle_first_arm_rad;
  xSum += length_arm_center * cos(angle_second_arm_rad);
  ySum += length_arm_center * sin(angle_second_arm_rad);

  // third arm addition
  double angle_third_arm_rad =
      (angle_front_deg - 90) * PI / 180 + angle_second_arm_rad;
  xSum += length_arm_front * cos(angle_third_arm_rad);
  ySum += length_arm_front * sin(angle_third_arm_rad);

  // distance from start (0,0) to xy is the tire distance
  double tire_distance = sqrt(pow(xSum, 2) + pow(ySum, 2));

  return tire_distance;
}
