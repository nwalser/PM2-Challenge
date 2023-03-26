#include "TimeDelta.h"
#include "mbed.h"
#include <cstdio>

void TimeDelta::reset() {
    _last_micros = getCurrentMicros();
}

long TimeDelta::getCurrentMicros() {
  using namespace std::chrono;

  auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
  long current_micros = now_ms.time_since_epoch().count();

  return current_micros;
}

long TimeDelta::getMicrosDelta() {
  long current_micros = getCurrentMicros();
  long micros_delta = current_micros - _last_micros;
  _last_micros = current_micros;

  return micros_delta;
}

double TimeDelta::getSecondsDelta() {
  return (double)getMicrosDelta() / 1000000;
}