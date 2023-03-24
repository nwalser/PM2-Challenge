#include "Servo.h"
#include "ServoController.h"
#include "communication.h"
#include "mbed.h"
#include <cstdio>

BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();
FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

// create servo objects
Servo *servo_joint_front = new Servo(PB_2);
Motion *motion_planner = new Motion();
// ServoController *servo_controller =
//    new ServoController(motion_planner, servo_joint_front);

int main() {
  servo_joint_front->enable();

  double angle = 0;
  while (true) {
    servo_joint_front->setNormalisedAngle(angle);
    angle += 0.001;
    printf("%f \n", angle);
    ThisThread::sleep_for(100ms);
  }

  while (true) {
    ThisThread::sleep_for(1s);
  }
}

/*
int main() {
  motion_planner->setLimits(40, 20, 20);

  printf("Init \n");
  servo_controller->init();
  servo_controller->wait_until_idle();
  while (true) {
    printf("Move to 50 \n");
    servo_controller->move_to(50);
    servo_controller->wait_until_idle();

    printf("Move to 180 \n");
    servo_controller->move_to(180);
    servo_controller->wait_until_idle();
  }

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
*/