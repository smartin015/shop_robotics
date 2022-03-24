#ifndef CURVES_H
#define CURVES_H

// These arrays define motion curves where the X axis (array index) is time  and the Y axis (value) is in steps/second. 
#define CURVE_SZ 16
#define NCURVES 3
const int16_t CURVES[][CURVE_SZ] = {
  {},
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
  {0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0},
};

#endif // CURVES_H
