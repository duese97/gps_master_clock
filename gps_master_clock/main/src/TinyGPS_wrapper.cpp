
#include "TinyGPS.h"

TinyGPS gps;

extern "C" {
    bool TinyGPS_wrapper_encode(char c)
    {
        return gps.encode(c);
    }
    void TinyGPS_wrapper_crack_datetime(int *year, uint8_t *month, uint8_t *day, 
    uint8_t *hour, uint8_t *minute, uint8_t *second, uint8_t *hundredths, unsigned long *fix_age)
    {
        gps.crack_datetime(year, month, day, hour, minute, second, hundredths, fix_age);
    }
}
