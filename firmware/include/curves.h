#ifndef CURVES_H
#define CURVES_H

#include <stdint.h>

// These arrays define motion curves where the X axis (array index) is time  and the Y axis (value) is in steps/second. 
#define CURVE_SZ 16
#define NCURVES 3

// One extra position to account for null `0` curve
extern const int16_t CURVES[NCURVES+1][CURVE_SZ];

#endif // CURVES_H
