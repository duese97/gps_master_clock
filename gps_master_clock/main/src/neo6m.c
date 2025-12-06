#include "neo6m.h"

#include "custom_main.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gptimer.h"
#include "bsp.h"

#include "slave_clk.h"
#include "TinyGPS_wrapper.h"


#define SECOND_TIMER_PERIOD_US 1000000ULL
#define UART_BLOCK_TICKS 2000

#define NUM_DRIFT_EVALUATIONS   7


/* Configure parameters of an UART driver, communication pins and install the driver */
const uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

const int intr_alloc_flags =
#if CONFIG_UART_ISR_IN_IRAM
    ESP_INTR_FLAG_IRAM;
#else
    0;
#endif // CONFIG_UART_ISR_IN_IRAM


static volatile time_t mcu_utc; // current, locally tracked UTC time of the micro controller


static void periodic_timer_callback(void* arg)
{
    static task_msg_t msg = {.dst = TASK_TIMER, .cmd = TASK_CMD_SECOND_TICK }; // prepare message for timer
    
    mcu_utc++;
    msg.utc_time = mcu_utc;
    msg.us_timestamp = esp_timer_get_time();

    sendTaskMessageISR(&msg);
}

static const esp_timer_create_args_t periodic_timer_args =
{
    .callback = &periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "secTimer"
};

static void calc_linear_regression(int64_t* src_arr, int src_len, float* slope, float* y_intercept)
{
    float x_average = (src_len - 1) / 2; // assumes equally spaced x values
    float y_average = 0;

    for (int idx = 0; idx < src_len; idx++)
    {
        y_average += src_arr[idx];
    }
    y_average /= src_len;

    float std_deviation_x = 0;
    float std_deviation_y = 0;
    for (int idx = 0; idx < src_len; idx++)
    {
        float x_square = (idx - x_average); x_square *= x_square;
        std_deviation_x += x_square;

        float y_square = (src_arr[idx] - y_average); y_square *= y_square;
        std_deviation_y += y_square;
    }
    // why (src_len - 1) -> we take it from a sample and not a population (just scr_len)
    std_deviation_x = sqrt(std_deviation_x / (src_len - 1));
    std_deviation_y = sqrt(std_deviation_y / (src_len - 1));

    float covariance_xy = 0;
    for (int idx = 0; idx < src_len; idx++)
    {
        covariance_xy += ((idx - x_average) * (src_arr[idx] - y_average));
    }
    covariance_xy *= (1.0 / (src_len - 1));

    float correlation_xy = covariance_xy / (std_deviation_x * std_deviation_y);

    *slope = correlation_xy * std_deviation_x / std_deviation_y;
    *y_intercept = -1.0f * (*slope) * x_average + y_average;
}

void TIMER_Task(void *parameter)
{
    task_msg_t msg_slave_clk = {.dst = TASK_SLAVE_CLK, .cmd = TASK_CMD_SECOND_TICK }; // prepare message for slave_clk

    task_msg_t msg;
    // setup periodic timer for local slave_clking
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    int64_t last_gps_connection_us = 0;

    // timestamp in microseconds when minute wraparound happened..
    int64_t minute_wraparound_ISR = 0; // .. in interrupt
    int64_t minute_wraparound_GPS = 0; // .. from GPS
    time_t gps_utc = 0, isr_utc = 0;

    uint64_t corrected_period_us = 0;

    int num_drift_evals = 0;
    int64_t drift_per_min[NUM_DRIFT_EVALUATIONS];

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMER, portMAX_DELAY, &msg) != true)
            continue;

        if (msg.cmd == TASK_CMD_GPS_TIME)
        {
            if (esp_timer_is_active(periodic_timer) == false)
            { // inital startup
                mcu_utc = msg.utc_time;
                // start cyclic timer
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US));
            }

            last_gps_connection_us = msg.us_timestamp;
            ram_mirror.last_connected_utc = msg.utc_time;

            // check if time is actually different, in case more than one message comes per second
            if (msg.utc_time % 60 == 0 && gps_utc != msg.utc_time )
            {
                minute_wraparound_GPS = msg.us_timestamp;
            }
            gps_utc = msg.utc_time;

            if (isr_utc == 0) // check if ISR already came
                continue; // wait until first message arrived

            // determine time difference between local clock and received time
            int32_t clock_diff_utc_sec = difftime(isr_utc, gps_utc);
    
            // Reason to adjust the timer: the UTC time is simply wrong (transmission error, etc.)
            // or the local timer leads/lags too much
            bool adjust_utc_diff = abs(clock_diff_utc_sec) >= MAX_ALLOWED_ABS_DIFF_SECONDS;
            if (adjust_utc_diff)
            { // too great, adjust
                ESP_ERROR_CHECK(esp_timer_stop(periodic_timer)); // halt timer, it does read-modify-write of the variable (not atomic)!
                mcu_utc = gps_utc; // set new UTC timestamp
                minute_wraparound_ISR = 0; minute_wraparound_GPS = 0; // reset the timestamps
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US)); // restart timer
    
                PRINT_LOG("Local clock drifted by: %ld, halting and re-adjusting to %lld", clock_diff_utc_sec, mcu_utc);
    
                // Accumulate the total drifted time into separate counters
                if (clock_diff_utc_sec > 0)
                {
                    ram_mirror.total_pos_time_corrected_ms += SEC_TO_MS(clock_diff_utc_sec);
                }
                else
                {
                    ram_mirror.total_neg_time_corrected_ms += -SEC_TO_MS(clock_diff_utc_sec);
                }
            }
        }
        else if (msg.cmd == TASK_CMD_SECOND_TICK)
        {
            if (last_gps_connection_us)
            {
                ram_shared.gps_last_connected_us = esp_timer_get_time() - last_gps_connection_us;
            }

            if (corrected_period_us != 0)
            {
                if (esp_timer_is_active(periodic_timer))
                { // sanity check: In case of weird race conditions make sure to stop the timer beforehand
                    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
                } // else: No need to stop the timer, was a one shot one

                // now use the period as previously determined
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, corrected_period_us));

                PRINT_LOG("Compensation done, using new period of %llu", corrected_period_us);
                corrected_period_us = 0; // finalize the correction
            }

            if (msg.utc_time % 60 == 0 && isr_utc != msg.utc_time)
            {
                minute_wraparound_ISR = msg.us_timestamp;
            }
            isr_utc = msg.utc_time;
        }
        else
        {
            PRINT_LOG("Unknown command: %u", msg.cmd);
            continue;
        }

        // check if both have been set
        if (minute_wraparound_ISR != 0 && minute_wraparound_GPS != 0)
        {
            bool perform_correction = true;

            // determine difference
            ram_shared.drift_total_us = minute_wraparound_ISR - minute_wraparound_GPS;

            // First, check if difference is plausible. Everything greater than the max allowed drift does not make sense
            if (llabs(ram_shared.drift_total_us) > MAX_PLAUSIBLE_DRIFT)
            {
                perform_correction = false;
                PRINT_LOG("Drift = %lld seconds: not plausible, resetting evaluation. Minute wraparound timestamp: local @ %lld gps @ %lld.",
                    ram_shared.drift_total_us,
                    minute_wraparound_ISR,
                    minute_wraparound_GPS);

                num_drift_evals = 0;
            }

            if (perform_correction)
            {
                PRINT_LOG("Current total deviation: %lldus", ram_shared.drift_total_us);

                drift_per_min[num_drift_evals] = ram_shared.drift_total_us;
                num_drift_evals++;

                // check if we are done
                perform_correction = (num_drift_evals >= NUM_DRIFT_EVALUATIONS);
            }

            float us_drift_per_min = 0, total_drift_us = 0;
            if (perform_correction)
            {
                num_drift_evals = 0; // reset counter in any case
                calc_linear_regression(drift_per_min, NUM_DRIFT_EVALUATIONS, &us_drift_per_min, &total_drift_us);

                PRINT_LOG("Drift %fus/min(%fms/h), total: %fus",
                    us_drift_per_min, us_drift_per_min * 60.0 * 24.0 / 1000.0, total_drift_us);

                // too small drifts do not need to be corrected
                if (fabs(total_drift_us) < DRIFT_CORR_THRESHOLD_US)
                {
                    PRINT_LOG("Drift too small, local clock accurate enough (for now)");
                    perform_correction = false;
                }
            }

            if (perform_correction)
            {
                uint64_t align_time_us = SECOND_TIMER_PERIOD_US - total_drift_us;
                corrected_period_us = (us_drift_per_min / 60.0) * SECOND_TIMER_PERIOD_US + SECOND_TIMER_PERIOD_US;

                // first stop the timer
                ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
                // align again with GPS clock, compensate the total drift once by modifying the period
                ESP_ERROR_CHECK(esp_timer_start_once(periodic_timer, align_time_us));
                PRINT_LOG("Threshold surpassed, starting drift correction in %lldus", align_time_us);
            }
            minute_wraparound_ISR = 0;
            minute_wraparound_GPS = 0;
        }


        if (msg.cmd == TASK_CMD_SECOND_TICK) // forward tick to slave_clk
        {
            msg_slave_clk.utc_time = mcu_utc;
            sendTaskMessage(&msg_slave_clk);
        }
    }
}


void NEO6M_Task(void *parameter)
{
     // prepare message
    task_msg_t msg_locked = {.dst = TASK_LCD, .cmd = TASK_CMD_GPS_LOCK_STATE };
    task_msg_t msg_gps_time = {.dst = TASK_TIMER, .cmd = TASK_CMD_GPS_TIME };

    char buf;
    struct tm gps_local_time = {0}; 
    uint32_t age;

    GPS_LOCK_STATE_t lock_state = GPS_LOCK_UNINITIALIZED;

    // setup the UART for the neo6M module
    ESP_ERROR_CHECK(uart_driver_install(NEO6M_UART, 256 /*must be at least this big(?)*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NEO6M_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO6M_UART, NEO6M_TX_PIN, NEO6M_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));

    while(1)
    {
        int res = uart_read_bytes(NEO6M_UART, &buf, sizeof(buf), UART_BLOCK_TICKS); // normally data should frequently come in
        if (res <= 0) 
        { // timed out or any other error
            if (lock_state != GPS_LOCK_LOST) // only need to set state / send message once
            {
                PRINT_LOG("No GPS signal");
                lock_state = GPS_LOCK_LOST;
                msg_locked.lock_state = GPS_LOCK_LOST;
                sendTaskMessage(&msg_locked);
            }
            continue;
        }

        if (TinyGPS_wrapper_encode(buf) == false)
        { // not yet done parsing
            continue;
        }
        
        // interpret received data
        res = TinyGPS_wrapper_crack_datetime(&gps_local_time, &msg_gps_time.utc_time, &age);
        if (res != 0)
        {
            PRINT_LOG("Unable to crack datetime, result: %d", res);
            continue;
        }
        msg_gps_time.us_timestamp = esp_timer_get_time();
        sendTaskMessage(&msg_gps_time);

        
        if (lock_state == GPS_LOCK_UNINITIALIZED)
        {
            PRINT_LOG("Inital lock, age: %lu GPS UTC: %lld", age, msg_gps_time.utc_time);
        }

        if (lock_state != GPS_LOCKED) // avoid sending same message over and over, if lock did not change
        {
            lock_state = GPS_LOCKED;
            msg_locked.lock_state = GPS_LOCKED;
            sendTaskMessage(&msg_locked);
        }
    }
}