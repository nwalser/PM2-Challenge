#include "Map.h"

Map::Map(double input_start, double input_end, double output_start,
         double output_end) {
  _input_end = input_end;
  _input_start = input_start;
  _output_start = output_start;
  _output_end = output_end;
}

double Map::mapValue(double input) {
  if (input > _input_end) {
    return _output_end;
  }

  if (input < _input_start) {
    return _output_start;
  }

  double input_delta = _input_end - _input_start;
  double output_delta = _output_end - _output_start;

  double input_percentage = (input - _input_start) / input_delta;

  double output = _output_start + input_percentage * output_delta;

  return output;
}