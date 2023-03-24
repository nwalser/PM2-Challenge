#ifndef TIMEDELTA_H
#define TIMEDELTA_H

class TimeDelta {
public:
  long get_micros_delta();
  double get_second_delta();

private:
  long _last_micros;
};

#endif