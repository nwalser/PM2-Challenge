#include "Motion.h"
#include "Servo.h"

class ServoController {
public:
  ServoController(Motion *motion_planner, Servo *servo);
  void init();
  void move_to(double angle_in_deg);
  bool is_on_desired_position();
  void wait_until_idle();

private:
  double _desired_angle;
  long _last_micros;

  Servo *_servo;
  Motion *_motion_planner;

  Thread _run_thread;

  void run();
  double calculate_new_angle();
  double angle_to_normalised_angle(double angle);
};