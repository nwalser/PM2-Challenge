#include "TimeDelta.h"
#include "mbed.h"
#include <cstdio>

long TimeDelta::get_micros_delta() {
  using namespace std::chrono;

  auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
  long current_micros = now_ms.time_since_epoch().count();

  long micros_delta = current_micros - _last_micros;
  _last_micros = current_micros;

  return micros_delta;
}

double TimeDelta::get_second_delta() {
  return (double)get_micros_delta() / 1000000;
}