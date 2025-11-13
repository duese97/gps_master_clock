#include "timekeep.h"

#include "custom_main.h"
#include "bsp.h"


static SemaphoreHandle_t tz_mutex;

static struct tm mcu_local_time;

static int clock_minutes_midnight = 0; // local minutes after midnight
static int clock_minutes_diff = 0; // difference to correct time

void take_tz_mutex(void)
{
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
    gpio_set_direction(GPIO_LED, GPIO_MODE_INPUT_OUTPUT);

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMEKEEP, 100, &msg) == false)
        { // reception did not work
            continue;
        }

        if(msg.cmd == SECOND_TRIGGER)
        {
            take_tz_mutex();
            mcu_local_time = *localtime(&msg.utc_time);
            give_tz_mutex();
    
            gpio_set_level(GPIO_LED, gpio_get_level(GPIO_LED) ? 0 : 1);
    
            if (mcu_local_time.tm_sec != 0) // only sync at full minutes
            {
                continue;
            }

            int mcu_minutes_midnight = mcu_local_time.tm_hour * 60 + mcu_local_time.tm_min;
            clock_minutes_diff = mcu_minutes_midnight - clock_minutes_midnight;

            // can not set counter clockwise difference, need to go clockwise -> wrap difference
            if (clock_minutes_diff < 0)
            {
                clock_minutes_diff = 12 * 60 - clock_minutes_diff;
            }

            PRINT_LOG("%d minutes time difference to target: %02d:%02d",
                clock_minutes_diff, mcu_local_time.tm_hour, mcu_local_time.tm_min);
        }
    }
}