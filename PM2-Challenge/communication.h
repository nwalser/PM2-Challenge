#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "mbed.h"
#include "rtos.h"

class Communicator {
public:
  Communicator();

  enum value_type { gyro_x, gyro_y, gyro_z };

  typedef struct {
    value_type type;
    double value;
  } value_message;

  void send_value(value_type type, double value);
  void send_value(value_type type, int value);

private:
  Mail<value_message, 16> _value_messages;
  Thread _dispatcher_thread;
  void run();
};

#endif