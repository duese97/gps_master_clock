#include "LCD.h"
#include "LCM1602.h"
#include "custom_main.h"

void LCD_Task(void *parameter)
{
    lcm1602_init();

    lcm1602_display();

    lcm1602_clear();

    lcm1602_print(" test ");

    while(1)
    {
        vTaskDelay(100);
    }
}
