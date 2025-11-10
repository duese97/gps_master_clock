#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "custom_main.h"

#define GPIO_LED GPIO_NUM_2


// for logging
SemaphoreHandle_t xUartSemaphore;
char print_buf[MAX_LOG_LEN];



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

    while(1)
    {
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(100);
        gpio_set_level(GPIO_LED, 0);
        vTaskDelay(100);
        PRINT_LOG("Toggle");
    }
}