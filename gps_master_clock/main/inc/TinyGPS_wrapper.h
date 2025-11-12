#ifndef _TINY_GPS_WRAPPER_H
#define _TINY_GPS_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <time.h>

bool TinyGPS_wrapper_encode(char c);
void TinyGPS_wrapper_crack_datetime(struct tm* localtime, uint32_t* age);
bool TinyGPS_wrapper_age_invalid(uint32_t age);

#ifdef __cplusplus
}
#endif

#endif // _TINY_GPS_WRAPPER_H
