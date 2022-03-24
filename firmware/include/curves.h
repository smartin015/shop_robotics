#ifndef CURVES_H
#define CURVES_H

#include <stdint.h>

// These arrays define motion curves where the X axis (array index) is time  and the Y axis (value) is in steps/second. 
#define CURVE_SZ 128
#define NCURVES 112

// One extra position to account for null `0` curve
extern const int16_t CURVES[NCURVES][CURVE_SZ];

#endif // CURVES_H
