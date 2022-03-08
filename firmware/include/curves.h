#ifndef CURVES_H
#define CURVES_H

#include <stdint.h>

// These arrays define motion curves where the X axis (array index) is time  and the Y axis (value) is in steps/second. 
#define CURVE_SZ 16
#define NCURVES 3
const int16_t CURVES[][CURVE_SZ];

#endif // CURVES_H
