#include "mbed.h"
#include "rtos.h"

class Robot {
public:
  void move_forward(double distance_in_mm);
  void move_backward(double distance_in_mm);
  void move_joint_front(double angle_in_deg, bool move_tires=true);
  void move_joint_back(double angle_in_deg, bool move_tires=true);
  void wait_until_finished();





private:


};