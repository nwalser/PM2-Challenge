#ifndef TIMEDELTA_H
#define TIMEDELTA_H

class TimeDelta {
public:
  long GetMicrosDelta();
  double GetSecondsDelta();

private:
  long _last_micros;
};

#endif