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
    static task_msg_t msg = {.dst = TASK_TIMEKEEP, .cmd = SECOND_TRIGGER }; // prepare message
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



void neo6M_Task(void *parameter)
{
    char buf;
    struct tm last_gps_time = {0};
    struct tm gps_local_time = {0}; 
    time_t last_connected_utc = 0;
    uint32_t age;
    bool synced_once = false;
    double total_time_corrected = 0;

    // setup the UART for the neo6M module
    ESP_ERROR_CHECK(uart_driver_install(NEO6M_UART, 256 /*must be at least this big(?)*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NEO6M_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO6M_UART, NEO6M_TX_PIN, NEO6M_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));

    // setup periodic timer for local timekeeping
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    while(1)
    {
        int res = uart_read_bytes(NEO6M_UART, &buf, sizeof(buf), 2000); // normally data should frequently come in
        if (res <= 0) 
        { // timed out or any other error
            PRINT_LOG("No GPS signal");
            continue;
        }

        if (TinyGPS_wrapper_encode(buf) == false)
        { // not yet done parsing
            continue;
        }
        
        // interpret received data
        TinyGPS_wrapper_crack_datetime(&gps_local_time, &last_connected_utc, &age);

        if (TinyGPS_wrapper_age_invalid(age) == true)
        { // age should have proper value
            continue;
        }
        
        if (synced_once == false)
        {
            PRINT_LOG("Inital sync");
            synced_once = true;
            last_gps_time = gps_local_time;

            mcu_utc = last_connected_utc;

            // start cyclic timer
            ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECOND_TIMER_PERIOD_US));
            continue;
        }

        // determine time difference between local clock and received time
        double clock_diff = difftime(mcu_utc, last_connected_utc);
        if (fabs(clock_diff) > MAX_ALLOWED_LOCAL_CLOCK_DRIFT_SECONDS)
        { // too great, adjust
            esp_timer_stop(periodic_timer); // halt timer, it does read-modify-write of the variable (not atomic)!
            mcu_utc = last_connected_utc; // set new UTC timestamp
            esp_timer_restart(periodic_timer, SECOND_TIMER_PERIOD_US); // restart timer

            total_time_corrected += fabs(clock_diff);
        }

        // once every minute: print the delta unconditionally (or immediately if the sync was more than a minute ago)
        if ((gps_local_time.tm_min != last_gps_time.tm_min) || (gps_local_time.tm_hour != last_gps_time.tm_hour))
        {
            print_tm_time("GPS time: ", &gps_local_time);
            PRINT_LOG("MCU <-> GPS delta: %.0fs, total corrected: %.0lf", clock_diff, total_time_corrected);
        }

        last_gps_time = gps_local_time; // remember last time
    }
}