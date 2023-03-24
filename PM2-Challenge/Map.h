#ifndef MAP_H
#define MAP_H

#include "Motion.h"
#include "Servo.h"

class Map {
public:
  Map(double input_start, double input_end, double output_start,
      double output_end);

  double map(double input);

private:
  double _input_start;
  double _input_end;
  double _output_start;
  double _output_end;
};

#endif