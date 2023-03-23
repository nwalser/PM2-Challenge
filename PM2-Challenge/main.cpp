#include "communication.h"
#include "mbed.h"
#include <cstdio>

BufferedSerial serial(USBTX, USBRX, 115200);
Communicator *communicator = new Communicator();

FileHandle *mbed::mbed_override_console(int fd) { return &serial; }

Thread t1;
Thread t2;
Thread t3;

void producer1() {
  while (true) {
    communicator->send_value(Communicator::value_type::gyro_x, 10.1);
    ThisThread::sleep_for(4s);
  }
}

void producer2() {
  while (true) {
    communicator->send_value(Communicator::value_type::gyro_y, 10.54);
    ThisThread::sleep_for(1s);
  }
}

void producer3() {
  while (true) {
    communicator->send_value(Communicator::value_type::gyro_z, 213);
    ThisThread::sleep_for(1100ms);
  }
}

// main() runs in its own thread in the OS
int main() {
  t1.start(producer1);
  t2.start(producer2);
  t2.start(producer3);

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
