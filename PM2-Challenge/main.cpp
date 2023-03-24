#include "Servo.h"
#include "ServoController.h"
#include "communication.h"
#include "mbed.h"
#include <cstdio>

// setup communication stack
BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();
FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

// misc
DigitalIn *user_button = new DigitalIn(BUTTON1);

// create servo objects
Servo *servo_joint_front = new Servo(PC_8);
Motion *motion_planner_front = new Motion();
Map *angle_map_front = new Map(0, 180, 0.032, 0.12);
ServoController *servo_controller_front = new ServoController(
    servo_joint_front, motion_planner_front, angle_map_front);

Servo *servo_joint_back = new Servo(PB_2);
Motion *motion_planner_back = new Motion();
Map *angle_map_back = new Map(0, 180, 0.04, 0.1);
ServoController *servo_controller_back =
    new ServoController(servo_joint_back, motion_planner_back, angle_map_back);

// DANGEROUS!
#define WAIT_UNTIL_TRUE(func)                                                  \
  while (!func) {                                                              \
    ThisThread::sleep_for(10ms);                                               \
  }

// mail loop. thread is managed by rtos
int main() {
  // initialize all controlls
  printf("Init \n");
  motion_planner_back->setLimits(50, 30, 30);
  motion_planner_front->setLimits(50, 30, 30);

  servo_controller_back->init();
  servo_controller_front->init();
  WAIT_UNTIL_TRUE(servo_controller_back->is_initialized());

  printf("Ready \n");
  // wait until blue user button is pressed
  WAIT_UNTIL_TRUE(!user_button->read());

  while (true) {
    printf("Move to 0 \n");
    servo_controller_back->move_to(0);
    servo_controller_front->move_to(0);
    WAIT_UNTIL_TRUE(servo_controller_back->is_on_position());

    printf("Move to 180 \n");
    servo_controller_back->move_to(180);
    servo_controller_front->move_to(180);
    WAIT_UNTIL_TRUE(servo_controller_back->is_on_position());
  }

  while (true) {
    ThisThread::sleep_for(1s);
  }
}