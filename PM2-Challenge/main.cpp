#include "Servo.h"
#include "ServoController.h"
#include "communication.h"
#include "mbed.h"
#include <cstdio>

#include "FastPWM.h"
#include "PositionController.h"

// setup communication stack
BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();
FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

// misc
DigitalIn *user_button = new DigitalIn(BUTTON1);

// create servo objects
Servo *servo_joint_front = new Servo(PC_8);
Motion *motion_planner_front = new Motion();
Map *angle_map_front = new Map(0, 180, 0.032, 0.11);
ServoController *servo_controller_front = new ServoController(
    servo_joint_front, motion_planner_front, angle_map_front);

Servo *servo_joint_back = new Servo(PB_2);
Motion *motion_planner_back = new Motion();
Map *angle_map_back = new Map(0, 180, 0.04, 0.1);
ServoController *servo_controller_back =
    new ServoController(servo_joint_back, motion_planner_back, angle_map_back);

// DANGEROUS, BUT USEFULL, MACRO! :D
#define WAIT_UNTIL_TRUE(func)                                                  \
  while (!(func)) {                                                            \
    ThisThread::sleep_for(10ms);                                               \
  }

#define WAIT_UNTIL_RANGE(func, value, range)                                   \
  WAIT_UNTIL_TRUE(fabs((func) - (value)) < (range))

int main() {
  // create motor objects
  DigitalOut enable_motors(PB_15);
  FastPWM pwm_M3(PB_13);

  EncoderCounter encoder_M3(PA_0, PA_1);
  const float max_voltage = 12.0f;
  const float counts_per_turn = 20.0f * 78.125f;
  const float kn = 180.0f / 12.0f;
  // original k_gear value was 100, this was to low, dont know why :)
  const float k_gear = 120.0f / 78.125f;
  const float kp = 0.05f;
  float max_speed_rps = 0.2f;

  PositionController *positionController_M3 = new PositionController(
      counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M3, encoder_M3);

  positionController_M3->setSpeedCntrlGain(kp * k_gear);
  positionController_M3->setMaxVelocityRPS(max_speed_rps);

  // initialize all controlls
  printf("Init \n");
  motion_planner_back->setLimits(50, 30, 30);
  motion_planner_front->setLimits(50, 30, 30);

  servo_controller_back->init(0);
  servo_controller_front->init();
  WAIT_UNTIL_TRUE(servo_controller_back->isIdle() &&
                  servo_controller_front->isIdle());

  enable_motors.write(0);

  printf("Ready \n");
  // wait until blue user button is pressed
  WAIT_UNTIL_TRUE(!user_button->read());

  while (true) {
    enable_motors.write(1);

    printf("Move 0 \n");
    positionController_M3->setDesiredRotation(0);
    WAIT_UNTIL_RANGE(positionController_M3->getRotation(), 0, 0.001);

    printf("Move 180 \n");
    positionController_M3->setDesiredRotation(0.5);
    WAIT_UNTIL_RANGE(positionController_M3->getRotation(), 0.5, 0.001);
  }

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
