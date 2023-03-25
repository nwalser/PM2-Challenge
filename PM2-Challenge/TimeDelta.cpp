#include "TimeDelta.h"
#include "mbed.h"
#include <cstdio>

long TimeDelta::GetMicrosDelta() {
  using namespace std::chrono;

  auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
  long current_micros = now_ms.time_since_epoch().count();

  long micros_delta = current_micros - _last_micros;
  _last_micros = current_micros;

  return micros_delta;
}

double TimeDelta::GetSecondsDelta() {
  return (double)GetMicrosDelta() / 1000000;
}