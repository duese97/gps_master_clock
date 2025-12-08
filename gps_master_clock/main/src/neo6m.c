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


enum
{
    CLK_CORRECT_NONE,
    CLK_CORRECT_PHASE,
    CLK_CORRECT_DRIFT
};


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

    int num_drift_evals = 0;
    int64_t drift_per_min[NUM_DRIFT_EVALUATIONS];
    ram_shared.current_period_us = SECOND_TIMER_PERIOD_US;

    int clk_correct_state = CLK_CORRECT_NONE;

    while(1)
    {
        if (receiveTaskMessage(TASK_TIMER, portMAX_DELAY, &msg) != true)
            continue;

        if (msg.cmd == TASK_CMD_GPS_TIME)
        {
            if (esp_timer_is_active(periodic_timer) == false)
            { // inital startup
                mcu_utc = msg.utc_time;
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ram_shared.current_period_us));
            }

            bool eval_drift = false;

            // check if time is actually different, in case more than one message comes per second
            if (msg.utc_time % 60 == 0 && gps_utc != msg.utc_time )
            {
                if (last_gps_connection_us != 0) // must have had at least one minute wraparound before
                {
                    drift_per_min[num_drift_evals] = SECOND_TIMER_PERIOD_US - (msg.us_timestamp - last_gps_connection_us);
                    PRINT_LOG("Drift = %lldus eval %d/%d",
                        drift_per_min[num_drift_evals],
                        num_drift_evals+1,
                        NUM_DRIFT_EVALUATIONS
                    );
                    eval_drift = true;
                }
                minute_wraparound_GPS = msg.us_timestamp;
            }
            last_gps_connection_us = msg.us_timestamp;
            ram_mirror.last_connected_utc = msg.utc_time;
            gps_utc = msg.utc_time;

            if (eval_drift)
            {
                bool perform_correction = true;

                // First, check if difference is plausible. Everything greater than the max allowed drift does not make sense
                if (llabs(drift_per_min[num_drift_evals]) > MAX_PLAUSIBLE_DRIFT)
                {
                    PRINT_LOG("Drift = %lldus: not plausible, resetting evaluation.",
                        drift_per_min[num_drift_evals]
                    );

                    perform_correction = false;
                    num_drift_evals = 0;
                }

                if (perform_correction)
                {
                    num_drift_evals++;
                    // check if we are done
                    perform_correction = (num_drift_evals >= NUM_DRIFT_EVALUATIONS);
                }

                float us_drift_per_min = 0, total_drift_us = 0;
                if (perform_correction)
                {
                    num_drift_evals = 0; // reset counter in any case
                    calc_linear_regression(drift_per_min, NUM_DRIFT_EVALUATIONS, &us_drift_per_min, &total_drift_us);

                    PRINT_LOG("Drift %fus/min(%fus/d), total: %fus",
                        us_drift_per_min, us_drift_per_min * 60.0 * 24.0, total_drift_us);
                }
            }

            // determine time difference between local clock and received time
            int64_t clock_diff_usec;
            bool adjust_utc_diff = false;

            if (clk_correct_state == CLK_CORRECT_PHASE)
            {
                clock_diff_usec = ram_shared.drift_total_us;
                adjust_utc_diff = true;
            }
            else if (isr_utc != 0) // check if ISR already came
            {
              clock_diff_usec = difftime(isr_utc, gps_utc);
              clock_diff_usec = SEC_TO_US(clock_diff_usec);

              // Reason to adjust the timer: the UTC time is simply wrong (transmission error, etc.)
              // or the local timer leads/lags too much
              adjust_utc_diff = llabs(clock_diff_usec) >= MAX_ALLOWED_ABS_DIFF_USEC;
            }
    
            if (adjust_utc_diff)
            { // too great, adjust
                ESP_ERROR_CHECK(esp_timer_stop(periodic_timer)); // halt timer, it does read-modify-write of the variable (not atomic)!
                mcu_utc = gps_utc; // set new UTC timestamp
                minute_wraparound_ISR = 0; minute_wraparound_GPS = 0; // reset the timestamps
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ram_shared.current_period_us)); // restart timer
    
                char* reason;
                if (clk_correct_state == CLK_CORRECT_PHASE)
                {
                    clk_correct_state = CLK_CORRECT_NONE;
                    reason = "Phase drifted too much";
                }
                else
                {
                    reason = "Absolute times differed";
                }
                PRINT_LOG("%s. Local clock drifted by: %lldus, halting and re-adjusting to %lld",
                    reason,
                    clock_diff_usec,
                    mcu_utc
                );
    
                // Accumulate the total drifted time into separate counters
                if (clock_diff_usec > 0)
                {
                    ram_mirror.total_pos_time_corrected_ms += clock_diff_usec;
                }
                else
                {
                    ram_mirror.total_neg_time_corrected_ms += -clock_diff_usec;
                }
            }
        }
        else if (msg.cmd == TASK_CMD_SECOND_TICK)
        {
            if (last_gps_connection_us)
            {
                ram_shared.gps_last_connected_us = esp_timer_get_time() - last_gps_connection_us;
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
            // determine difference
            ram_shared.drift_total_us = minute_wraparound_ISR - minute_wraparound_GPS;

            // check if the difference is too much
            bool drift_corr_needed = (llabs(ram_shared.drift_total_us) < MAX_PLAUSIBLE_DRIFT);
            drift_corr_needed &= (llabs(ram_shared.drift_total_us) > DRIFT_CORR_THRESHOLD_US);
            if (drift_corr_needed)
            {
                clk_correct_state = CLK_CORRECT_PHASE; // request alignment
                PRINT_LOG("Aligning local clock to GPS...");
            }
            else // Not plausible, resetting
            {
                clk_correct_state = CLK_CORRECT_NONE;
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
    time_t last_utc = 0;

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
        
        // only send message if the seconds actually differ
        if (last_utc == msg_gps_time.utc_time)
        {
            continue;
        }

        last_utc = msg_gps_time.utc_time;
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