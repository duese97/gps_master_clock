#include "neo6m.h"

#include "custom_main.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gptimer.h"
#include "bsp.h"

#include "timekeep.h"
#include "TinyGPS_wrapper.h"


#define SECOND_TIMER_PERIOD_US 1000000ULL
#define UART_BLOCK_TICKS 2000

#define NUM_CLOCKDIFF_EVALUATIONS   5

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

// timestamp in microseconds when minute wraparound happened..
static volatile int64_t minute_wraparound_ISR = 0; // .. in interrupt
static volatile int64_t minute_wraparound_Task = 0; // .. in task
static volatile int64_t phase_difference_us = 0; // current difference


static int64_t diffs[NUM_CLOCKDIFF_EVALUATIONS];

static void periodic_timer_callback(void* arg)
{
    static task_msg_t msg = {.dst = TASK_TIMEKEEP, .cmd = TASK_CMD_SECOND_TICK }; // prepare message for timekeep
    
    mcu_utc++;
    msg.utc_time = mcu_utc;
    ram_shared.gps_time_age = mcu_utc - ram_mirror.last_connected_utc; // determine when the last connection happened
    if (mcu_utc % 60 == 0)
    {
        minute_wraparound_ISR = esp_timer_get_time(); // remember time
        if (minute_wraparound_Task != 0) // we were slower than the task
        {
            phase_difference_us = minute_wraparound_ISR - minute_wraparound_Task;
            minute_wraparound_Task = 0; // 'ack' the read
        }
    }

    sendTaskMessageISR(&msg);
}

static const esp_timer_create_args_t periodic_timer_args =
{
    .callback = &periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "secTimer"
};



void NEO6M_Task(void *parameter)
{
     // prepare message
    static task_msg_t msg_locked = {.dst = TASK_LCD, .cmd = TASK_CMD_GPS_LOCK_STATE };

    char buf;
    struct tm gps_local_time = {0}; 
    uint32_t age;
    int minute_old; // for determining when minute changed

    GPS_LOCK_STATE_t lock_state = GPS_LOCK_UNINITIALIZED;

    // setup the UART for the neo6M module
    ESP_ERROR_CHECK(uart_driver_install(NEO6M_UART, 256 /*must be at least this big(?)*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NEO6M_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO6M_UART, NEO6M_TX_PIN, NEO6M_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));

    // setup periodic timer for local timekeeping
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

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
        res = TinyGPS_wrapper_crack_datetime(&gps_local_time, &ram_mirror.last_connected_utc, &age);
        if (res != 0)
        {
            PRINT_LOG("Unable to crack datetime, result: %d", res);
            continue;
        }
        
        if (lock_state == GPS_LOCK_UNINITIALIZED)
        {
            mcu_utc = ram_mirror.last_connected_utc;
            minute_old = gps_local_time.tm_min;

            // start cyclic timer
            ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US));

            PRINT_LOG("Inital lock, age: %lu mcu utc: %lld last connected utc: %lld", age, mcu_utc, ram_mirror.last_connected_utc);
        }
        else if (lock_state == GPS_LOCKED && minute_old != gps_local_time.tm_min)
        {
            // If locked, we can compare the phase difference between the GPS signal and
            // the local clock. Ideally it should remain relatively constant.
            minute_old = gps_local_time.tm_min;

            // Critical section to determine the difference
            portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
            taskENTER_CRITICAL(&mux);
            minute_wraparound_Task = esp_timer_get_time(); // make snapshot in any case
            if (minute_wraparound_ISR != 0) // if the ISR was faster than this task: calculate the difference
            {
                // Since the phase difference is not accumulated over a long time it should be fine to
                // base the difference off the internal ESP timer. It will be subjected to the same drift
                // or inaccuracies as the seconds timer, but within a minute it should be negligible.
                phase_difference_us = minute_wraparound_ISR - minute_wraparound_Task;
                minute_wraparound_ISR = 0; // 'ack' the read
            }
            // Calculate the difference regardless, worst case is that this is the difference from the last minute
            ram_shared.phase_difference_ms = USEC_TO_MS(phase_difference_us);
            taskEXIT_CRITICAL(&mux);
            
            PRINT_LOG("Phase difference (local clock - GPS clock): %ldms", ram_shared.phase_difference_ms);
        }

        if (lock_state != GPS_LOCKED) // avoid sending same message over and over, if lock did not change
        {
            lock_state = GPS_LOCKED;
            msg_locked.lock_state = GPS_LOCKED;
            sendTaskMessage(&msg_locked);
        }
        
        // determine time difference between local clock and received time
        int32_t clock_diff_utc_sec = difftime(mcu_utc, ram_mirror.last_connected_utc);

        // Reason to adjust the timer: the UTC time is simply wrong (transmission error, etc.)
        // or the local timer leads/lags too much
        bool adjust_utc_diff = abs(clock_diff_utc_sec) >= MAX_ALLOWED_LOCAL_CLOCK_DRIFT_SECONDS;
        if (adjust_utc_diff)
        { // too great, adjust
            ESP_ERROR_CHECK(esp_timer_stop(periodic_timer)); // halt timer, it does read-modify-write of the variable (not atomic)!
            mcu_utc = ram_mirror.last_connected_utc; // set new UTC timestamp
            minute_wraparound_Task = 0; minute_wraparound_ISR = 0; // reset the timestamps
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
}