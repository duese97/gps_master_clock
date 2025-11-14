#include "timekeep.h"

#include "custom_main.h"
#include "bsp.h"


static SemaphoreHandle_t tz_mutex;

static struct tm target_local_time;

static int clock_minutes_midnight = 0; // local minutes after midnight
static int clock_minutes_diff = 0; // difference to correct time

int pulse_len_ms, pulse_pause_ms;

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
        if (receiveTaskMessage(TASK_TIMEKEEP, 10, &msg) == false)
        { // reception did not work
            continue;
        }

        if(msg.cmd == SECOND_TRIGGER)
        {
            take_tz_mutex();
            // In case any process outside of this application (lwip,..?) changed the timezone.
            setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1); // For details see TinyGPS_wrapper_crack_datetime()
            target_local_time = *localtime(&msg.utc_time);
            give_tz_mutex();

            // Toggle LED to indicate activity
            gpio_set_level(GPIO_LED, gpio_get_level(GPIO_LED) ? 0 : 1);
    
            if (target_local_time.tm_sec != 0) // only sync at full minutes
            {
                continue;
            }

            int target_minutes_midnight = target_local_time.tm_hour * 60 + target_local_time.tm_min;
            clock_minutes_diff = target_minutes_midnight - clock_minutes_midnight;

            if (clock_minutes_diff == 0) // perfectly synced
            {
                continue;
            }

            if (clock_minutes_diff < 0) // can not set counter clockwise difference, need special handling
            {
                if (clock_minutes_diff < -MAX_LOCAL_CLOCK_LEAD_MINUTES) // if difference too large -> need to wrap around
                {
                    clock_minutes_diff = 12 * 60 - clock_minutes_diff;
                }
                else // difference is not too much, we can just wait
                {
                    PRINT_LOG("Time slightly leads, waiting %d minutes ...", clock_minutes_diff);
                    continue;
                }
            }

            PRINT_LOG("%d minutes time difference to target: %02d:%02d",
                clock_minutes_diff, target_local_time.tm_hour, target_local_time.tm_min);
        }

        if (clock_minutes_diff)
        { // if we come here: do clock pulses
            // set GPIO(s)
            vTaskDelay(pulse_len_ms / portTICK_PERIOD_MS);
            // set GPIO(s)
            vTaskDelay(pulse_pause_ms / portTICK_PERIOD_MS);

            clock_minutes_midnight++; // one step closer to the target time
        }
    }
}
