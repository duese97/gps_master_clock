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

void TIMER_Task(void *parameter)
{
    task_msg_t msg_timekeep = {.dst = TASK_TIMEKEEP, .cmd = TASK_CMD_SECOND_TICK }; // prepare message for timekeep

    task_msg_t msg;
    // setup periodic timer for local timekeeping
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    uint64_t last_gps_connection = 0;
    // timestamp in microseconds when minute wraparound happened..
    int64_t minute_wraparound_ISR = 0; // .. in interrupt
    int64_t minute_wraparound_GPS = 0; // .. from GPS
    time_t gps_utc = 0, isr_utc = 0;

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

            last_gps_connection = msg.us_timestamp;
            ram_mirror.last_connected_utc = msg.utc_time;
            if (msg.utc_time % 60 == 0 && gps_utc != msg.utc_time )
            {
                minute_wraparound_GPS = msg.us_timestamp;
            }
            gps_utc = msg.utc_time;
        }
        else if (msg.cmd == TASK_CMD_SECOND_TICK)
        {
            if (last_gps_connection)
            {
                ram_shared.gps_time_age = USEC_TO_S(esp_timer_get_time() - last_gps_connection);
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
            ram_shared.phase_difference_us = minute_wraparound_ISR - minute_wraparound_GPS;
            PRINT_LOG("Phase difference (local clock - GPS clock): %lldms", USEC_TO_MS(ram_shared.phase_difference_us));
            minute_wraparound_ISR = 0;
            minute_wraparound_GPS = 0;
        }

        if (isr_utc != 0 && gps_utc != 0)
        {
            // determine time difference between local clock and received time
            int32_t clock_diff_utc_sec = difftime(isr_utc, gps_utc);
    
            // Reason to adjust the timer: the UTC time is simply wrong (transmission error, etc.)
            // or the local timer leads/lags too much
            bool adjust_utc_diff = abs(clock_diff_utc_sec) >= MAX_ALLOWED_LOCAL_CLOCK_DRIFT_SECONDS;
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

        if (msg.cmd == TASK_CMD_SECOND_TICK) // forward tick to timekeep
        {
            msg_timekeep.utc_time = isr_utc;
            sendTaskMessage(&msg_timekeep);
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