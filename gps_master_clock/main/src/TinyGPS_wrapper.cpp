
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
void TinyGPS_wrapper_crack_datetime(struct tm* local, time_t* utc, uint32_t* age)
{
    struct tm tim;
    uint8_t hundredths, month, day, hour, min, sec;
    int year;

    gps.crack_datetime(&year, &month, &day, &hour, &min, &sec, &hundredths, age);

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

    // workaround: timegm POSIX function is not available, but we need to convert the
    // time struct to UTC timestamp first.
    setenv("TZ","GMT0",1);
    tzset();
    
    // Calculate UTC time
    *utc = mktime(&tim);
    
    // Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
    setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
    tzset();

    give_tz_mutex(); // release lock again, other tasks can now calculate local time again

    // Take timezone + daylight saving into account
    *local = *localtime(utc);
}

bool TinyGPS_wrapper_age_invalid(uint32_t age)
{
    return age == TinyGPS::GPS_INVALID_AGE;
}
