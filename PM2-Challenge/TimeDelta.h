#ifndef TIMEDELTA_H
#define TIMEDELTA_H

class TimeDelta {
public:
  void Reset();
  long GetMicrosDelta();
  double GetSecondsDelta();

private:
  long _last_micros;
  long GetCurrentMicros();
};

#endif