#include "timekeep.h"

#include "custom_main.h"
#include "bsp.h"


static SemaphoreHandle_t tz_mutex;

static int current_minutes_12o_clock = 0; // local minutes after 12 o clock position

int pulse_len_ms = 100, pulse_pause_ms = 100;


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
    static task_msg_t local_time_msg = {.dst = TASK_LCD, .cmd = LOCAL_TIME };
    int clock_minutes_diff = 0; // difference to correct time
    struct tm target_local_time;
    task_msg_t msg;
    bool locked_once = false;
    gpio_set_direction(GPIO_LED, GPIO_MODE_INPUT_OUTPUT);

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMEKEEP, 10, &msg) == true)
        {
            if(msg.cmd == SECOND_TRIGGER)
            {
                // Toggle LED to indicate activity
                gpio_set_level(GPIO_LED, gpio_get_level(GPIO_LED) ? 0 : 1);

                take_tz_mutex();
                // In case any process outside of this application (lwip,..?) changed the timezone.
                setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1); // For details see TinyGPS_wrapper_crack_datetime()
                target_local_time = *localtime(&msg.utc_time);
                give_tz_mutex();

                local_time_msg.local_time = target_local_time;
                sendTaskMessage(&local_time_msg);
        
                if (target_local_time.tm_sec != 0) // only sync at full minutes
                {
                    continue;
                }

                int target_minutes_12o_clock = target_local_time.tm_hour % 12; // we only have a 12 hour clock
                target_minutes_12o_clock *= 60; // 60 minutes per hour
                target_minutes_12o_clock += target_local_time.tm_min; // add minutes normally

                // determine the current difference
                clock_minutes_diff = target_minutes_12o_clock - current_minutes_12o_clock;

                // wrote time at least once
                locked_once = true;

                if (clock_minutes_diff == 0) // perfectly synced
                {
                    continue;
                }

                if (clock_minutes_diff < 0) // can not set counter clockwise difference, need special handling
                {
                    if (clock_minutes_diff < -MAX_LOCAL_CLOCK_LEAD_MINUTES) // if difference too large -> need to wrap around
                    {
                        clock_minutes_diff = 12 * 60 - clock_minutes_diff;
                        PRINT_LOG("Local time leads too much, wrapping around");
                    }
                    else // difference is not too much, we can just wait
                    {
                        PRINT_LOG("Local time slightly leads, waiting %d minutes ...", clock_minutes_diff);
                        continue;
                    }
                }

                PRINT_LOG("%02d:%02d -> %d minutes time difference to target -> %02d:%02d(%02d:%02d)",
                    current_minutes_12o_clock / 60, current_minutes_12o_clock % 60,
                    clock_minutes_diff,
                    target_local_time.tm_hour % 12, target_local_time.tm_min,
                    target_local_time.tm_hour, target_local_time.tm_min);
            }
        } // else: no new messages

        if (locked_once == false)
        {
            continue;
        }

        if (clock_minutes_diff > 0) // no backwards pulses possible
        { // if we come here: do clock pulses
            // set GPIO(s)
            gpio_set_level(GPIO_LED, 0);
            vTaskDelay(pulse_len_ms / portTICK_PERIOD_MS);
            // set GPIO(s)
            gpio_set_level(GPIO_LED, 1);
            vTaskDelay(pulse_pause_ms / portTICK_PERIOD_MS);

            // Set GPIO(s)
            gpio_set_level(GPIO_LED, 0);
            current_minutes_12o_clock++; // one step closer to the target time
            if (current_minutes_12o_clock >= 12 * 60) // keep within 12 hour bounds
            {
                current_minutes_12o_clock = 0;
            }
            clock_minutes_diff--;
        }
    }
}
