#include "ServoController.h"
#include "mbed.h"
#include "rtos.h"


class Robot {
public:
  Robot(ServoController *servo_joint_front, ServoController *servo_joint_back);

  void move_forward(double distance_in_mm);
  void move_backward(double distance_in_mm);
  void move_joint_front(double angle_in_deg, bool move_tires = true);
  void move_joint_back(double angle_in_deg, bool move_tires = true);
  void wait_until_finished();

private:
  ServoController *_servo_joint_front;
  ServoController *_servo_joint_back;
  Thread _run_thread;
  void run();
};