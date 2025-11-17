#ifndef _TINY_GPS_WRAPPER_H
#define _TINY_GPS_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <time.h>

bool TinyGPS_wrapper_encode(char c);
int TinyGPS_wrapper_crack_datetime(struct tm* local, time_t* utc, uint32_t* age);

#ifdef __cplusplus
}
#endif

#endif // _TINY_GPS_WRAPPER_H
