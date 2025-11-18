
#include "TinyGPS.h"
#include "TinyGPS_wrapper.h"

extern "C" {
    #include "timekeep.h" // for TZ mutex
}

TinyGPS gps;


bool TinyGPS_wrapper_encode(char c)
{
    return gps.encode(c);
}
int TinyGPS_wrapper_crack_datetime(struct tm* local, time_t* utc, uint32_t* age)
{
    struct tm tim;
    uint8_t hundredths, month, day, hour, min, sec;
    int year;

    gps.crack_datetime(&year, &month, &day, &hour, &min, &sec, &hundredths, age);

    if (*age == TinyGPS::GPS_INVALID_AGE)
    {
        return -1;
    }

    // general sanity checks
    if (month > 12 || day > 31 || hour > 23 || min > 59 || sec > 59)
    {
        return -1;
    }

    // could not receive time older than project build date
    if (year < 2025)
    {
        return -1;
    }

    // Prepare time struct for mktime
    tim.tm_year = year - 1900;
    tim.tm_mon = month - 1;
    tim.tm_mday = day;
    tim.tm_hour = hour;
    tim.tm_min = min;
    tim.tm_sec = sec;
    tim.tm_isdst = -1;

    // For the case that the local time is calculated by another TASK we want to have the
    // proper timezone! Acquire lock
    take_tz_mutex();

    // Calculate UTC time
    *utc = mktime(&tim);

    // release lock again, other tasks can now calculate local time again
    give_tz_mutex();

    // Take timezone + daylight saving into account
    *local = *localtime(utc);

    return 0;
}
