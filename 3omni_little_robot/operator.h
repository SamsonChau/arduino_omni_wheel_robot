#ifndef OPERATOR_H
#define OPERATOR_H

float filter_low(float value, float filter){
  if (abs(value) < filter){
      value = 0;
  }
  return value;
}

float soften_value(float axis, float soften, float max_v){
  return axis * pow(abs(axis/max_v), soften);
}

#endif
