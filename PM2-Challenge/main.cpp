#include "FastPWM.h"
#include "PositionController.h"
#include "Servo.h"
#include "ServoController.h"
#include "communication.h"
#include "mbed.h"
#include "robot.h"
#include <cmath>
#include <cstdio>

// setup communication stack
BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();
FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

#pragma region macros

// DANGEROUS, BUT USEFUL, MACRO! :D
#define WAIT_UNTIL_TRUE(func)                                                  \
  while (!(func)) {                                                            \
    ThisThread::sleep_for(10ms);                                               \
  }

#define WAIT_UNTIL_RANGE(func, value, range)                                   \
  WAIT_UNTIL_TRUE(fabs((func) - (value)) < (range))

#pragma endregion

int main() {
#pragma region dependency creation
  printf("Start \n");

  // create user button
  DigitalIn *user_button = new DigitalIn(BUTTON1);

  // SERVO PC_8
  // create front servo controller
  Servo *servo_joint_front = new Servo(PC_8);
  Motion *motion_planner_front = new Motion();
  motion_planner_front->setLimits(50, 30, 30);
  Map *angle_map_front = new Map(0, 160, 0.032, 0.115);
  ServoController *servo_controller_front = new ServoController(
      servo_joint_front, motion_planner_front, angle_map_front);

  // SERVO PB_2
  // create back servo controller
  Servo *servo_joint_back = new Servo(PB_2);
  Motion *motion_planner_back = new Motion();
  motion_planner_back->setLimits(50, 30, 30);
  //Map *angle_map_back = new Map(0, 120, 0.1, 0.04);
  Map *angle_map_back = new Map(0, 160, 0.032, 0.115);
  ServoController *servo_controller_back = new ServoController(
      servo_joint_back, motion_planner_back, angle_map_back);

  // create motor objects
  DigitalOut *enable_motors = new DigitalOut(PB_15);
  const float max_voltage = 12.0f;
  const float counts_per_turn = 20.0f * 78.125f;
  const float kn = 180.0f / 12.0f;
  // original k_gear value was 100, this was to low, don't know why :)
  const float k_gear = 120.0f / 78.125f;
  const float kp = 0.05f;
  float max_speed_rps = 0.5f;

  // m1
  // create front motor controller
  FastPWM pwm_front(PB_13);
  EncoderCounter encoder_front(PA_6, PC_7);
  PositionController *position_controller_front =
      new PositionController(counts_per_turn * k_gear, kn / k_gear, max_voltage,
                             pwm_front, encoder_front);
  position_controller_front->setSpeedCntrlGain(kp * k_gear);
  position_controller_front->setMaxVelocityRPS(max_speed_rps);

  // M2
  // create back motor controller
  FastPWM pwm_back(PA_9);
  EncoderCounter encoder_back(PB_6, PB_7);
  PositionController *position_controller_back =
      new PositionController(counts_per_turn * k_gear, kn / k_gear, max_voltage,
                             pwm_back, encoder_back);
  position_controller_back->setSpeedCntrlGain(kp * k_gear);
  position_controller_back->setMaxVelocityRPS(max_speed_rps);

  // create robot object
  Robot *robot = new Robot(servo_controller_front, servo_controller_back,
                           position_controller_front, position_controller_back,
                           enable_motors);
#pragma endregion

  // initialize all controlls
  printf("Init \n");

  robot->init();
  WAIT_UNTIL_TRUE(robot->isIdle());

  printf("Ready \n");
  // wait until blue user button is pressed
  WAIT_UNTIL_TRUE(!user_button->read());

  while (true) {
    printf("Min \n");
    robot->setJointAngles(0, 0);
    WAIT_UNTIL_TRUE(robot->isIdle());

    ThisThread::sleep_for(1s);

    printf("Max \n");
    robot->setJointAngles(160, 160);
    WAIT_UNTIL_TRUE(robot->isIdle());

    ThisThread::sleep_for(1s);
  }

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
