#include "slave_clk.h"

#include <string.h> // for string copy and other functions

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  // for vTaskGetRunTimeStats

#include "esp_adc/adc_oneshot.h"

#include "custom_main.h"
#include "bsp.h"


// Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
static const char* timezone_europe_berlin = "CET-1CEST,M3.5.0,M10.5.0/3";
static const char* timezone_gmt = "GMT0";

static SemaphoreHandle_t tz_mutex;

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t calib_handle;
static int slave_rel_min_voltage_mV;

static void print_stats(void)
{
    static uint8_t last_num_tasks = 0;
    static char* runtime_stat_buffer_ptr; // ~40B per task, 

    // some general stats:
    LOG(
        "General:\n"
        "\tFree heap: %lu, minimum free heap: %lu\n"
        "\tTotal corrected: pos:%lus neg:%lus\n"
        "\tTotal operating time: %lus = %luh = %lud\n"
        "\tOperating time since boot: %lus = %luh = %lud\n",
        esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
        ram_mirror.total_pos_time_corrected_ms, ram_mirror.total_neg_time_corrected_ms,
        ram_mirror.total_operating_seconds, SEC_TO_H(ram_mirror.total_operating_seconds), SEC_TO_DAY(ram_mirror.total_operating_seconds),
        ram_shared.operating_seconds, SEC_TO_H(ram_shared.operating_seconds), SEC_TO_DAY(ram_shared.operating_seconds)
    );
    uint8_t curr_num_tasks = uxTaskGetNumberOfTasks();
    if (last_num_tasks != curr_num_tasks)
    { // number of tasks changed, re-allocate
        if (runtime_stat_buffer_ptr)
        { // free old buffer
            vPortFree(runtime_stat_buffer_ptr);
            runtime_stat_buffer_ptr = NULL;
        }

        LOG("Re-allocating, task num changed from %u to %u", last_num_tasks, curr_num_tasks);

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
        LOG("Runtime stats:\n%s", runtime_stat_buffer_ptr);
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
        LOG("Unable to take TZ lock");
    }
}

void give_tz_mutex(void)
{
    xSemaphoreGive(tz_mutex); 
}

static int calc_slave_voltage_mv(void)
{
    int raw = 0, voltage = 0, samples, result;

    for (samples = 0; samples < 4; ++samples)
    {
        int tmp;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &tmp));

        raw += tmp;
    }
    raw = raw / samples;  

    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calib_handle, raw, &voltage));

    // we have a 61k <-> 4.7k resistive divider
    result = ((61000.0 + 4700.0) / 4700.0) * voltage;
    LOG("Samples: %d Raw reading: %d ADC Voltage: %d Slave voltage: %d", samples, raw, voltage, result);

    return result;
}

static bool is_slave_voltage_ok(void)
{
    bool result;
    int slave_voltage_mv = calc_slave_voltage_mv();

    // sanity check: Absolute value should not be too low (in case inital measurement is already
    // under fault measured)
    if (slave_voltage_mv < MIN_SLAVE_VOLTAGE_MV)
        result = false;
    // Voltage should not get too low, relative to the inital measurement
    else if (slave_voltage_mv < slave_rel_min_voltage_mV * MAX_SLAVE_VOLTAGE_DEVIATION)
        result = false;
    else
        result = true;

    return result;    
}

void SLAVE_CLK_Task(void *parameter)
{
    static task_msg_t local_time_msg = {.dst = TASK_LCD, .cmd = TASK_CMD_LOCAL_TIME };
    int clock_minutes_diff = 0; // difference to correct time
    struct tm target_local_time; // from conversion from received UTC to localtime
    task_msg_t msg; // scratch buffer for receiving task messages
    char* timezone_env_ptr = NULL; // points to heap, where timezone string will be buffered

    bool comm_line_1 = false;
    bool comm_line_2 = false;

    bool ignore_sec_tick = false;

    gpio_set_direction(GPIO_LED, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(H_BRIDGE1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE2_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE2_D, GPIO_MODE_OUTPUT);

    // Make sure everything is off by default
    gpio_set_level(H_BRIDGE1_A, 0);
    gpio_set_level(H_BRIDGE1_B, 0);
    gpio_set_level(H_BRIDGE2_C, 0);
    gpio_set_level(H_BRIDGE2_D, 0);

    // configure the one-shot ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config)); // ADC_CHANNEL_4 = GPIO_NUM_32

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &calib_handle));

    // Determine what the inital driving voltage for the clock slaves is (assumes
    // there is no issue upon boot). Then calculate what the relative threshold is, below which
    // no operation is possible.
    slave_rel_min_voltage_mV = calc_slave_voltage_mv() * MAX_SLAVE_VOLTAGE_DEVIATION;

    LOG("Initial slave voltage: %dmV", slave_rel_min_voltage_mV);

    while(1)
    {
        uint32_t wait_time = clock_minutes_diff != 0 ? 0 : portMAX_DELAY; // if there are pulses to be done -> skip the wait for new messages
        if (receiveTaskMessage(TASK_SLAVE_CLK, wait_time, &msg) == true)
        {
            switch(msg.cmd)
            {
                case TASK_CMD_SHUTDOWN:
                {
                    gpio_set_level(GPIO_LED, 0); // disable LED to save a bit power
                    vTaskSuspend(NULL);
                    break;
                }

                case TASK_CMD_COMMISSIONING:
                {
                    comm_line_1 = msg.comm_line_1;
                    comm_line_2 = msg.comm_line_2;
                    clock_minutes_diff = 0;
                    break;
                }
                case TASK_CMD_LINE_ADVANCE_MINUTES:
                {
                    if (comm_line_1 || comm_line_2)
                    {
                        clock_minutes_diff += msg.line_advance_minutes;
                        LOG("Minutes left to step: %d", clock_minutes_diff);
                    }
                    
                    break;
                }
                case TASK_CMD_SIMULATE_SECOND_TICK:
                    ignore_sec_tick = true;
                // fall through
                case TASK_CMD_SECOND_TICK:
                {
#if USE_TESTCODE == 1
                    if (ignore_sec_tick == true && msg.cmd == TASK_CMD_SECOND_TICK)
                    { // special case: we are testing the seconds tick. skip the "normal" message from the IRQ
                        continue;
                    }
#endif // USE_TESTCODE == 1
                    ram_mirror.total_operating_seconds++;
                    ram_shared.operating_seconds++;
                    if (ram_mirror.total_operating_seconds % 60 == 0)
                    {
                        print_stats();
                    }

                    if (comm_line_1 || comm_line_2) // if commissioning right now -> skip all of the handling
                    {
                        continue;
                    }

                    take_tz_mutex(); // wait until we can manipulate the timezone

                    /* Timezone/env handling in general is really messed up in newlib. Calling it over and over WILL
                    * RESULT IN A MEMORY LEAK! Suggested workaround: sentenv once with sufficiently long value, and
                    * then modify that value/change the timezone. See also:
                    * https://github.com/espressif/esp-idf/issues/3046#issuecomment-499168477 */
                    if (timezone_env_ptr == NULL)
                    {
                        setenv("TZ", timezone_europe_berlin, 1);
                        timezone_env_ptr = getenv("TZ");
                        LOG("Initially allocating timezone: %s", timezone_env_ptr);
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
                    clock_minutes_diff = target_minutes_12o_clock - ram_mirror.current_slave_minutes_12o_clock;
                    clock_minutes_diff = clock_minutes_diff % MINUTES_PER_12H;

                    if (clock_minutes_diff > 0)
                    {
                        int minutes_lead = (clock_minutes_diff + MAX_LOCAL_CLOCK_LEAD_MINUTES) % MINUTES_PER_12H;

                        // caution around 12'o clock position: if our local time is 00:00 or after, and the
                        // received GPS time is before 00:00 -> large difference, where it would make sense to wait!
                        if (minutes_lead < MAX_LOCAL_CLOCK_LEAD_MINUTES)
                        {
                            clock_minutes_diff = -minutes_lead; // this way is 'shorter'
                            LOG("Local time leads slightly, with the GPS time about to wrap"); // TODO: correct print/handling?
                            continue;
                        }
                    }
                    else if (clock_minutes_diff < 0) // can not set counter clockwise difference, need special handling
                    {
                        if (clock_minutes_diff < -MAX_LOCAL_CLOCK_LEAD_MINUTES) // if difference too large -> need to wrap around
                        {
                            clock_minutes_diff = MINUTES_PER_12H - clock_minutes_diff;
                            LOG("Local time leads too much, wrapping around");
                        }
                        else // difference is not too much, we can just wait
                        {
                            LOG("Local time slightly leads, waiting %d minutes ...", clock_minutes_diff);
                            continue;
                        }
                    }

                    LOG("%02ld:%02ld -> %d minutes time difference to target -> %02d:%02d(%02d:%02d)",
                        ram_mirror.current_slave_minutes_12o_clock / 60, ram_mirror.current_slave_minutes_12o_clock % 60,
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
            bool line_1_en = comm_line_1 || !comm_line_2;
            bool line_2_en = comm_line_2 || !comm_line_1;

            // set polarity of the h bridges
            if (line_1_en)
            {
                // In case many pulses have to be generated and a short circuit is present:
                // Do not short out the line over and over. Either wait until the error is gone
                // or we are back to regular, minute triggered pulses.
                if (clock_minutes_diff == 1 || !ram_shared.short_circuit_line_1)
                {
                    // enable bridge, let current flow
                    gpio_set_level(H_BRIDGE1_A, ram_mirror.line1_last_pol);
                    gpio_set_level(H_BRIDGE1_B, !ram_mirror.line1_last_pol);

                    ram_shared.short_circuit_line_1 = !is_slave_voltage_ok();
                }

                if (ram_shared.short_circuit_line_1)
                { // something is wrong, disable the bridge again
                    LOG("Short circuit when driving line 1");
                }
                else
                { // if voltage is within bounds: Wait for pulse to finish
                    vTaskDelay(ram_mirror.period_ms / portTICK_PERIOD_MS);

                    // Toggle polarity of H bridge(s) for next time
                    ram_mirror.line1_last_pol = !ram_mirror.line1_last_pol;
                }
                // disable bridges again
                gpio_set_level(H_BRIDGE1_A, 0);
                gpio_set_level(H_BRIDGE1_B, 0);
            }

            // Same as above, but "level out" the current draw a bit between the H bridges
            if (line_2_en)
            {
                if (clock_minutes_diff == 1 || !ram_shared.short_circuit_line_2)
                {
                    gpio_set_level(H_BRIDGE2_C, ram_mirror.line2_last_pol);
                    gpio_set_level(H_BRIDGE2_D, !ram_mirror.line2_last_pol);

                    ram_shared.short_circuit_line_2 = !is_slave_voltage_ok();
                }

                if (ram_shared.short_circuit_line_2)
                {
                    LOG("Short circuit when driving line 2");
                }
                else
                {
                    vTaskDelay(ram_mirror.period_ms / portTICK_PERIOD_MS);
                    ram_mirror.line2_last_pol = !ram_mirror.line2_last_pol;
                }

                gpio_set_level(H_BRIDGE2_C, 0);
                gpio_set_level(H_BRIDGE2_D, 0);
            }
            
            // do not increment when any slave is commissioned with pulses (keep master time as is)
            if (!comm_line_2 && !comm_line_1)
            {
                ram_mirror.current_slave_minutes_12o_clock++; // one step closer to the target time
                ram_mirror.current_slave_minutes_12o_clock %= MINUTES_PER_12H; // keep within 12 hour bounds
            }
            clock_minutes_diff--;
        }
    }
}
