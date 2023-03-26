/*
TimeDelta measures the time that has passed between calls to its api.
*/

#ifndef TIMEDELTA_H
#define TIMEDELTA_H

class TimeDelta {
public:
  void reset();
  long getMicrosDelta();
  double getSecondsDelta();

private:
  long _last_micros;
  long getCurrentMicros();
};

#endif