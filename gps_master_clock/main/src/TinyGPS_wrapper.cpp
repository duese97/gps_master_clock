
#include "TinyGPS.h"
#include "TinyGPS_wrapper.h"

TinyGPS gps;


bool TinyGPS_wrapper_encode(char c)
{
    return gps.encode(c);
}
void TinyGPS_wrapper_crack_datetime(struct tm* localtime, uint32_t* age)
{
    struct tm tim;
    uint8_t hundredths, month, day, hour, min, sec;
    int year;

    gps.crack_datetime(&year, &month, &day, &hour, &min, &sec, &hundredths, age);

    tim.tm_year = year - 1900;
    tim.tm_mon = month - 1;
    tim.tm_mday = day;
    tim.tm_hour = hour;
    tim.tm_min = min;
    tim.tm_sec = sec;
    tim.tm_isdst = -1;

    // workaround: timegm POSIX function is not available, but we need to convert the
    // time struct to UTC timestamp first.
    setenv("TZ","GMT0",1);
    tzset();
    
    // Calculate UTC time
    time_t gmt = mktime(&tim);
    
    // Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
    setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
    tzset();

    // Take timezone + daylight saving into account
    localtime_r(&gmt, localtime);
}

bool TinyGPS_wrapper_age_invalid(uint32_t age)
{
    return age == TinyGPS::GPS_INVALID_AGE;
}
