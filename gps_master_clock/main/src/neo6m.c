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


static volatile time_t mcu_utc;



static void periodic_timer_callback(void* arg)
{
    static task_msg_t msg = {.dst = TASK_TIMEKEEP, .cmd = TASK_CMD_SECOND_TICK }; // prepare message
    mcu_utc++;
    msg.utc_time = mcu_utc;

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
        res = TinyGPS_wrapper_crack_datetime(&gps_local_time, &rm.last_connected_utc, &age);
        if (res != 0)
        {
            PRINT_LOG("Unable to crack datetime, result: %d", res);
            continue;
        }
        
        if (lock_state == GPS_LOCK_UNINITIALIZED)
        {
            mcu_utc = rm.last_connected_utc;

            // start cyclic timer
            ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US));

            PRINT_LOG("Inital lock, age: %lu mcu utc: %lld last connected utc: %lld", age, mcu_utc, rm.last_connected_utc);
        }

        if (lock_state != GPS_LOCKED) // avoid sending same message over and over, if lock did not change
        {
            lock_state = GPS_LOCKED;
            msg_locked.lock_state = GPS_LOCKED;
            sendTaskMessage(&msg_locked);
        }

        // determine time difference between local clock and received time
        double clock_diff = difftime(mcu_utc, rm.last_connected_utc);
        if (fabs(clock_diff) > MAX_ALLOWED_LOCAL_CLOCK_DRIFT_SECONDS)
        { // too great, adjust
            ESP_ERROR_CHECK(esp_timer_stop(periodic_timer)); // halt timer, it does read-modify-write of the variable (not atomic)!
            mcu_utc = rm.last_connected_utc; // set new UTC timestamp
            ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US)); // restart timer

            PRINT_LOG("Local clock drifted by: %lf, halting and re-adjusting to %lld", clock_diff, mcu_utc);

            // Accumulate the total drifted time into separate counters
            if (clock_diff > 0)
            {
                rm.total_pos_time_corrected += clock_diff;
            }
            else
            {
                rm.total_neg_time_corrected += -clock_diff;
            }
        }
    }
}