#ifndef _WPROGRAM_H_
#define _WPROGRAM_H_

// basically just a dummy to get the TinyGPS lib to compile WITHOUT the Arduino bloatware

#include <stdint.h>
#include <math.h>
#include "custom_main.h" // for millis

#define byte uint8_t
#define millis(m) ESP_IDF_MILLIS(m)

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define degrees(rad) (rad * RAD_TO_DEG)
#define radians(deg) (deg * DEG_TO_RAD)
#define sq(val)      sqrt(val)

#endif // _WPROGRAM_H_
