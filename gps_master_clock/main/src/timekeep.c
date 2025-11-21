#include "timekeep.h"

#include <string.h> // for string copy and other functions

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  // for vTaskGetRunTimeStats

#include "custom_main.h"
#include "bsp.h"


// Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
static const char* timezone_europe_berlin = "CET-1CEST,M3.5.0,M10.5.0/3";
static const char* timezone_gmt = "GMT0";

static SemaphoreHandle_t tz_mutex;


static void print_stats(void)
{
    static uint8_t last_num_tasks = 0;
    static char* runtime_stat_buffer_ptr; // ~40B per task, 

    // some general stats:
    PRINT_LOG(
        "General:\n"
        "\tFree heap: %lu, minimum free heap: %lu\n"
        "\tTotal corrected: pos:%lus neg:%lus\n"
        "\tUptime: %lus = %luh = %lud",
        esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
        rm.total_pos_time_corrected, rm.total_neg_time_corrected,
        rm.total_uptime_seconds, rm.total_uptime_seconds / 3600, rm.total_uptime_seconds / (3600 * 24)
    );
    uint8_t curr_num_tasks = uxTaskGetNumberOfTasks();
    if (last_num_tasks != curr_num_tasks)
    { // number of tasks changed, re-allocate
        if (runtime_stat_buffer_ptr)
        { // free old buffer
            vPortFree(runtime_stat_buffer_ptr);
            runtime_stat_buffer_ptr = NULL;
        }

        PRINT_LOG("Re-allocating, task num changed from %u to %u", last_num_tasks, curr_num_tasks);

        // see :https://www.freertos.org/Documentation/02-Kernel/04-API-references/03-Task-utilities/00-Task-utilities#vtaskgetruntimestats
        // around 40B per task -> double it for safety
        runtime_stat_buffer_ptr = pvPortMalloc(80 * curr_num_tasks);
        if (runtime_stat_buffer_ptr)
        { // if allocation worked: remember new amount of tasks
            last_num_tasks = curr_num_tasks;
        }
    }
    if (runtime_stat_buffer_ptr != NULL) // make sure allocation worked
    {
        vTaskGetRunTimeStats(runtime_stat_buffer_ptr);
        PRINT_LOG("Runtime stats:\n%s", runtime_stat_buffer_ptr);
    }
}

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

void TIMEKEEP_Task(void *parameter)
{
    static task_msg_t local_time_msg = {.dst = TASK_LCD, .cmd = TASK_CMD_LOCAL_TIME };
    int clock_minutes_diff = 0; // difference to correct time
    struct tm target_local_time; // from conversion from received UTC to localtime
    task_msg_t msg; // scratch buffer for receiving task messages
    char* timezone_env_ptr = NULL; // points to heap, where timezone string will be buffered
    bool commissioning = false;

    gpio_set_direction(GPIO_LED, GPIO_MODE_INPUT_OUTPUT);

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMEKEEP, 10, &msg) == true)
        {
            switch(msg.cmd)
            {
                case TASK_CMD_SHUTDOWN:
                {
                    gpio_set_level(GPIO_LED, 0); // disable LED to save a bit power
                    vTaskSuspend(NULL);
                    break;
                }

                case TASK_CMD_START_COMMISSIONING:
                case TASK_CMD_STOP_COMMISSIONING:
                {
                    commissioning = (msg.cmd == TASK_CMD_START_COMMISSIONING);
                    clock_minutes_diff = 0;
                    break;
                }
                case TASK_CMD_SLAVE_ADVANCE_MINUTE:
                case TASK_CMD_SLAVE_ADVANCE_HOUR:
                {
                    if (commissioning)
                    { // force one tick
                        clock_minutes_diff = (msg.cmd == TASK_CMD_SLAVE_ADVANCE_MINUTE) ? 1 : 60;
                    }
                    
                    break;
                }
                case TASK_CMD_SECOND_TICK:
                {
                    rm.total_uptime_seconds++;
                    if (rm.total_uptime_seconds % 60 == 0)
                    {
                        print_stats();
                    }

                    if (commissioning == true) // if commissioning right now -> skip all of the handling
                    {
                        continue;
                    }

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

                    int target_minutes_12o_clock = target_local_time.tm_hour * 60 + target_local_time.tm_min;

                    // determine the current difference
                    clock_minutes_diff = target_minutes_12o_clock - rm.current_minutes_12o_clock;
                    clock_minutes_diff = clock_minutes_diff % MINUTES_PER_12H;

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
                        rm.current_minutes_12o_clock / 60, rm.current_minutes_12o_clock % 60,
                        clock_minutes_diff,
                        target_local_time.tm_hour % 12, target_local_time.tm_min,
                        target_local_time.tm_hour, target_local_time.tm_min);
                    break;
                }
                default:
                {
                    break;
                }
            }
        } // else: no new messages

        if (clock_minutes_diff > 0) // no backwards pulses possible
        { // if we come here: do clock pulses
            // set GPIO(s)
            gpio_set_level(GPIO_LED, 0);
            vTaskDelay(rm.pulse_len_ms / portTICK_PERIOD_MS);
            // set GPIO(s)
            gpio_set_level(GPIO_LED, 1);
            vTaskDelay(rm.pulse_pause_ms / portTICK_PERIOD_MS);

            // Set GPIO(s)
            gpio_set_level(GPIO_LED, 0);
            rm.current_minutes_12o_clock++; // one step closer to the target time
            rm.current_minutes_12o_clock %= MINUTES_PER_12H; // keep within 12 hour bounds
            clock_minutes_diff--;
        }
    }
}
