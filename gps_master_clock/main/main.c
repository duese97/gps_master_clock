#include <stdio.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_LED GPIO_NUM_2

void app_main(void)
{
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(100);
        gpio_set_level(GPIO_LED, 0);
        vTaskDelay(100);
    }
}