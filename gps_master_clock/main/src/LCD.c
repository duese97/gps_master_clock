#include "LCD.h"
#include "LCM1602.h"

#include "custom_main.h"

#define NUM_COLUMNS 16
#define NUM_ROWS 2

//static const char wait_animation[] = {'|', '/', '-', '\\'};

// contains the current time, is of fixed length, example:
// '13:08:00 15.11.2025 DST: 0    ' = 26 chars + 4 spaces + 1 null
#define MAX_TIME_PRINT_LEN 30
static char time_print_buff[MAX_TIME_PRINT_LEN + 1 + 6 /* compiler needs this to remove annoying warning*/];
int curr_dst_start;
static char scratch_buff[NUM_COLUMNS + 1];

void LCD_Task(void *parameter)
{
    task_msg_t msg;
    LOCK_STATE_t lock_state_local = LOCK_UNINITIALIZED;
    if (LCD_I2C_begin(NUM_COLUMNS, NUM_ROWS) != ESP_OK)
    {
        
    }
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_print("GPS Master Clock");
    LCD_I2C_setCursor(0, 1);
    LCD_I2C_print("  2025 D.Weber  ");
    vTaskDelay(1000);

    LCD_I2C_setCursor(0, 1);
    LCD_I2C_print("Await GPS lock");

    while(1)
    {
        if (receiveTaskMessage(TASK_LCD, 500, &msg) == true)
        {
            if (msg.cmd == GPS_LOCK_STATE)
            {
                lock_state_local = msg.lock_state;

                LCD_I2C_setCursor(0, 1);
                if (msg.lock_state == LOCKED_FIST)
                {
                    LCD_I2C_print("GPS first locked");
                }
                else if (msg.lock_state == LOCK_LOST)
                {
                    LCD_I2C_print("GPS lost lock   ");
                }
                else if (msg.lock_state == LOCKED_AGAIN)
                {
                    LCD_I2C_print("GPS re-locked   ");
                }
            }
            else if (msg.cmd == LOCAL_TIME)
            {
                struct tm* tm = &msg.local_time;
                snprintf(time_print_buff, sizeof(time_print_buff), "%02u:%02u:%02u %02u.%02u.%04u DST: %1u    ",
                    (uint8_t)tm->tm_hour, (uint8_t)tm->tm_min, (uint8_t)tm->tm_sec,
                    (uint8_t)tm->tm_mday, (uint8_t)(tm->tm_mon + 1), (uint16_t)(tm->tm_year + 1900),
                    (bool)tm->tm_isdst
                );
            }
        }
        // all actions which need to be polled
        else if (lock_state_local == LOCKED_FIST || lock_state_local == LOCKED_AGAIN)
        {
            int overhang = MAX_TIME_PRINT_LEN - (curr_dst_start + NUM_COLUMNS);
            if (overhang >= 0) // buffer is completely within
            {
                memcpy(scratch_buff, time_print_buff + curr_dst_start, NUM_COLUMNS);
            }
            else
            {
                memcpy(scratch_buff, time_print_buff + curr_dst_start, NUM_COLUMNS + overhang);
                memcpy(scratch_buff + NUM_COLUMNS + overhang, time_print_buff, -overhang);
            }
            PRINT_LOG("Original: '%s', '%s', idx: %d overhang: %d", time_print_buff, scratch_buff, curr_dst_start, overhang);
            curr_dst_start++;
            if (curr_dst_start >= MAX_TIME_PRINT_LEN)
            {
                curr_dst_start = 0;
            }
            scratch_buff[NUM_COLUMNS] = 0;
            LCD_I2C_setCursor(0, 0);
            LCD_I2C_print(scratch_buff);
        }
    }
}
