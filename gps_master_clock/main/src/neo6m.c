#include "neo6m.h"

#include "custom_main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "TinyGPS_wrapper.h"

#include "bsp.h"

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


void neo6M_Task(void *parameter)
{
    char buf;
    struct tm time_local;
    uint32_t age;

    ESP_ERROR_CHECK(uart_driver_install(NEO6M_UART, 256 /*must be at least this big(?)*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NEO6M_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO6M_UART, NEO6M_TX_PIN, NEO6M_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));

    while(1)
    {
        if (uart_read_bytes(NEO6M_UART, &buf, sizeof(buf), 1) != 1)
        { // timed out or any other error
            continue;
        }

        if (TinyGPS_wrapper_encode(buf) == false)
        { // not yet done parsing
            continue;
        }
        
        TinyGPS_wrapper_crack_datetime(&time_local, &age);

        if (TinyGPS_wrapper_age_invalid(age) == true)
        { // age should have proper value
            continue;
        }

        // calculate required offsets for 'actual' date values
        time_local.tm_year += 1900;
        time_local.tm_mon += 1;

        PRINT_LOG("%02d:%02d:%02d %d.%d.%d daylight saving: %d age: %lu",
            time_local.tm_hour, time_local.tm_min, time_local.tm_sec,
            time_local.tm_mday, time_local.tm_mon, time_local.tm_year, time_local.tm_isdst, age
        );
    }
}