#include "timekeep.h"

#include "custom_main.h"



static SemaphoreHandle_t tz_mutex;

static struct tm mcu_local_time;

static int clock_hour_hand, clock_minute_hand;


void take_tz_mutex(void)
{
    bool res = false;

    if (tz_mutex == NULL) // already created?
    {
      tz_mutex = xSemaphoreCreateMutex(); // for access to LOGs
    }
    
    if (xSemaphoreTake(tz_mutex, portMAX_DELAY) != pdTRUE)
    {
        PRINT_LOG("Unable to take TZ lock");
    }
}

void give_tz_mutex(void)
{
    xSemaphoreGive(tz_mutex); 
}

void print_tm_time(char* additional_str, struct tm* tm)
{
    PRINT_LOG("%s%02d:%02d:%02d %d.%d.%d DST: %d",
        additional_str,
        tm->tm_hour, tm->tm_min, tm->tm_sec,
        tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900,
        tm->tm_isdst
    );
}

void timekeep_Task(void *parameter)
{
    task_msg_t msg;
    while(1)
    {
        if (receiveTaskMessage(TASK_TIMEKEEP, 100, &msg) == false)
        { // reception did not work
            continue;
        }

        take_tz_mutex();
        mcu_local_time = *localtime(&msg.utc_time);
        give_tz_mutex();

        if (mcu_local_time.tm_sec != 0) // only sync at full minutes
        {
            continue;
        }
        
        int hour_diff = mcu_local_time.tm_hour - clock_hour_hand;
        int min_diff = mcu_local_time.tm_min - clock_minute_hand;

        min_diff += hour_diff * 60;

        if (min_diff != 0)
        {
            if (min_diff < 0)
            {
                min_diff = 12 * 60 - min_diff;
            }
            PRINT_LOG("Needing %d minute ticks for compensation", min_diff);
        }

        clock_hour_hand = mcu_local_time.tm_hour;
        clock_minute_hand = mcu_local_time.tm_min;

        PRINT_LOG("Hours: %d Minutes: %d", clock_hour_hand, clock_minute_hand);
    }
}