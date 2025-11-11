#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp.h"
#include "custom_main.h"

#include <sys/time.h>
#include "TinyGPS_wrapper.h"



// for logging
SemaphoreHandle_t xUartSemaphore;
char print_buf[MAX_LOG_LEN];

static void init_NEO_6M_uart(void)
{
    uart_port_t portNum = UART_NUM_2;
    int tx_pin = GPIO_NUM_17, rx_pin = GPIO_NUM_16;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(portNum, MAX_LOG_LEN /*not needed?*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(portNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(portNum, tx_pin, rx_pin, GPIO_NUM_NC, GPIO_NUM_NC));
}

static void init_serial_print(void)
{
    xUartSemaphore = xSemaphoreCreateMutex(); // for access to LOGs

    uart_port_t portNum = UART_NUM_0;
    int tx_pin = GPIO_NUM_1, rx_pin = GPIO_NUM_3;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(portNum, MAX_LOG_LEN /*not needed?*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(portNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(portNum, tx_pin, rx_pin, GPIO_NUM_NC, GPIO_NUM_NC));
}

//---------------------------------------------------------------------------
// Exported
//---------------------------------------------------------------------------

void serial_print_custom(void)
{
    size_t print_len = strlen(print_buf);
    if (print_len <= 0)
        return;

    if (print_len >= sizeof(print_buf))
        print_len = sizeof(print_buf);

    uart_write_bytes(UART_NUM_0, print_buf, print_len);
}


void app_main(void)
{
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    init_serial_print();
    init_NEO_6M_uart();

    // Set timezone for Europe/Berlin (https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv)
    setenv("TZ","CET-1CEST,M3.5.0,M10.5.0/3",1);
    tzset();

    char buf;
    struct tm tm;

    gpio_set_level(GPIO_LED, !gpio_get_level(GPIO_LED));
    while(1)
    {
        if (uart_read_bytes(UART_NUM_2, &buf, sizeof(buf), 1) != 1)
        { // timed out or any other error
            continue;
        }

        if (TinyGPS_wrapper_encode(buf) == false)
        { // not yet done parsing
            continue;
        }
        
        time_t now_utc = TinyGPS_wrapper_crack_datetime();
        localtime_r(&now_utc, &tm);

        PRINT_LOG("%02d:%02d:%02d %d.%d.%d daylight saving: %d",
            tm.tm_hour, tm.tm_min, tm.tm_sec,
            tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900, tm.tm_isdst
        );
    }
}