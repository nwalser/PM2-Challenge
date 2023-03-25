#include "TimeDelta.h"
#include "mbed.h"
#include <cstdio>

void TimeDelta::Reset() {
    _last_micros = GetCurrentMicros();
}

long TimeDelta::GetCurrentMicros() {
  using namespace std::chrono;

  auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
  long current_micros = now_ms.time_since_epoch().count();

  return current_micros;
}

long TimeDelta::GetMicrosDelta() {
  long current_micros = GetCurrentMicros();
  long micros_delta = current_micros - _last_micros;
  _last_micros = current_micros;

  return micros_delta;
}

double TimeDelta::GetSecondsDelta() {
  return (double)GetMicrosDelta() / 1000000;
}