
#include "TinyGPS.h"
#include "TinyGPS_wrapper.h"

TinyGPS gps;


bool TinyGPS_wrapper_encode(char c)
{
    return gps.encode(c);
}
time_t TinyGPS_wrapper_crack_datetime(void)
{
    struct tm tim;
    uint8_t hundredths, month, day, hour, min, sec;
    int year;
    unsigned long age;

    gps.crack_datetime(&year, &month, &day, &hour, &min, &sec, &hundredths, &age);

    tim.tm_year = year - 1900;
    tim.tm_mon = month - 1;
    tim.tm_mday = day;
    tim.tm_hour = hour;
    tim.tm_min = min;
    tim.tm_sec = sec;
    tim.tm_isdst = 0;

    return mktime(&tim);
}
