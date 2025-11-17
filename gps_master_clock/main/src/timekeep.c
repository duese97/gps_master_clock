#include "timekeep.h"

#include <string.h> // for string copy and other functions

#include "custom_main.h"
#include "bsp.h"


#define MINUTES_PER_12H  (12*60)

// Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
static const char* timezone_europe_berlin = "CET-1CEST,M3.5.0,M10.5.0/3";
static const char* timezone_gmt = "GMT0";

static SemaphoreHandle_t tz_mutex;

static int current_minutes_12o_clock = 0; // local minutes after 12 o clock position

// settings for the pulse waveform
int pulse_len_ms = 100, pulse_pause_ms = 100;

static time_t total_operating_seconds = 0;

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

void timekeep_Task(void *parameter)
{
    static task_msg_t local_time_msg = {.dst = TASK_LCD, .cmd = LOCAL_TIME };
    int clock_minutes_diff = 0; // difference to correct time
    struct tm target_local_time;
    task_msg_t msg;
    bool locked_once = false;
    char* timezone_env_ptr = NULL;
    gpio_set_direction(GPIO_LED, GPIO_MODE_INPUT_OUTPUT);

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMEKEEP, 10, &msg) == true)
        {
            if(msg.cmd == SECOND_TRIGGER)
            {
                total_operating_seconds++;

                // Toggle LED to indicate activity
                gpio_set_level(GPIO_LED, gpio_get_level(GPIO_LED) ? 0 : 1);

                take_tz_mutex(); // wait until we can manipulate the timezone

                /* Timezone/env handling in general is really messed up in newlib. Calling it over and over WILL
                 * RESULT IN A MEMORY LEAK! Suggested workaround: sentenv once with sufficiently long value, and
                 * then modify that value/change the timezone. See also:
                 * https://github.com/espressif/esp-idf/issues/3046#issuecomment-499168477 */
                if (timezone_env_ptr == NULL)
                {
                    setenv("TZ", timezone_europe_berlin, 1);
                    timezone_env_ptr = getenv("TZ");
                    PRINT_LOG("Initially allocating timezone: %s", timezone_env_ptr);
                }
                else // already allocated, can copy value
                {
                    strcpy(timezone_env_ptr, timezone_europe_berlin);
                }
                
                target_local_time = *localtime(&msg.utc_time); // determine the local time

                if (timezone_env_ptr)
                {
                    strcpy(timezone_env_ptr, timezone_gmt); // revert back to using GMT+0
                }

                give_tz_mutex(); // other processes can use the timezone again

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

                if (clock_minutes_diff > 0)
                {
                    int minutes_lead = (clock_minutes_diff + MAX_LOCAL_CLOCK_LEAD_MINUTES) % MINUTES_PER_12H;

                    // caution around 12'o clock position: if our local time is 00:00 or after, and the
                    // received GPS time is before 00:00 -> large difference, where it would make sense to wait!
                    if (minutes_lead < MAX_LOCAL_CLOCK_LEAD_MINUTES)
                    {
                        clock_minutes_diff = -minutes_lead; // this way is 'shorter'
                        PRINT_LOG("Local time leads slightly, with the GPS time about to wrap");
                        continue;
                    }
                }
                else if (clock_minutes_diff < 0) // can not set counter clockwise difference, need special handling
                {
                    if (clock_minutes_diff < -MAX_LOCAL_CLOCK_LEAD_MINUTES) // if difference too large -> need to wrap around
                    {
                        clock_minutes_diff = MINUTES_PER_12H - clock_minutes_diff;
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

                PRINT_LOG("Free heap: %lu, minimum free heap: %lu", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());

#if 0
                        // once every minute: print the delta unconditionally (or immediately if the sync was more than a minute ago)
        if ((gps_local_time.tm_min != last_gps_time.tm_min) || (gps_local_time.tm_hour != last_gps_time.tm_hour))
        {
            print_tm_time("GPS local time: ", &gps_local_time);
            PRINT_LOG("MCU <-> GPS delta: %ds, total corrected: pos:%ds neg:%ds",
                clock_diff, total_pos_time_corrected, total_neg_time_corrected);
        }
#endif

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
            if (current_minutes_12o_clock >= MINUTES_PER_12H) // keep within 12 hour bounds
            {
                current_minutes_12o_clock = 0;
            }
            clock_minutes_diff--;
        }
    }
}
