#include "LCD.h"
#include "LCM1602.h"

#include "custom_main.h"

#define NUM_COLUMNS 16
#define NUM_ROWS 2

#define GRAM_BACKSLASH_INDEX 1

// '13:08:00 15.11.2025 DST: 0    ' = 26 chars + 4 spaces + 1 null
#define MAX_TIME_PRINT_LEN 30

static const char wait_animation[] = {'|', '/', '-', GRAM_BACKSLASH_INDEX};
static uint8_t backslash_charmap[] =
{
 // these bits are usable:
 //      <--->
    0b00000000,
    0b00010000,
    0b00001000,
    0b00000100,
    0b00000010,
    0b00000001,
    0b00000000,
    0b00000000
};


void LCD_Task(void *parameter)
{
    // contains the current time, is of fixed length (why +6 -> compiler needs this to remove annoying warning)
    char time_print_buff[MAX_TIME_PRINT_LEN + 1 + 6] = "??:??:?? ??.??.???? DST: ?    ";
    int curr_src_start = 0; // index where to start copying from the time buffer
    char scratch_buff[NUM_COLUMNS + 1]; // to temporarily format the time
    int wait_animation_idx = 0;

    task_msg_t msg;
    GPS_LOCK_STATE_t lock_state_local = GPS_LOCK_UNINITIALIZED;

    if (LCD_I2C_begin(NUM_COLUMNS, NUM_ROWS) != ESP_OK)
    { // in case no display was found just poll/consume the task messages to avoid overflowing the queue
        PRINT_LOG("Unable to setup LCD I2C!");
        while(1)
        {
            receiveTaskMessage(TASK_LCD, 500, &msg);
        }
    }

    PRINT_LOG("LCD init done");

    LCD_I2C_setCursor(0, 0);
    LCD_I2C_print("GPS Master Clock");
    LCD_I2C_setCursor(0, 1);
    LCD_I2C_print("  2025 D.Weber  ");

    // for some reason the driver has no backslash, create one ourself
    // for some other reason writing to GRAM only works after the prints above
    LCD_I2C_createChar(GRAM_BACKSLASH_INDEX, backslash_charmap);

    vTaskDelay(1000);

    while(1)
    {
        if (receiveTaskMessage(TASK_LCD, 500, &msg) == true)
        {
            if (msg.cmd == GPS_LOCK_STATE)
            {
                lock_state_local = msg.lock_state;
                LCD_I2C_setCursor(0, 1);
                
                if (msg.lock_state == GPS_LOCKED)
                {
                    LCD_I2C_print("GPS locked      ");
                    continue; // nothing to be done, printing once is sufficient
                }
            }
            else if (msg.cmd == LOCAL_TIME)
            {
                // format the new time into local buffer
                struct tm* tm = &msg.local_time;
                snprintf(time_print_buff, sizeof(time_print_buff), "%02u:%02u:%02u %02u.%02u.%04u DST: %1u    ",
                    (uint8_t)tm->tm_hour, (uint8_t)tm->tm_min, (uint8_t)tm->tm_sec,
                    (uint8_t)tm->tm_mday, (uint8_t)(tm->tm_mon + 1), (uint16_t)(tm->tm_year + 1900),
                    (bool)tm->tm_isdst
                );
            }
        }
        
        /* all actions which need to be polled */

        // scrolling time value handling
        int overhang = MAX_TIME_PRINT_LEN - (curr_src_start + NUM_COLUMNS);
        if (overhang >= 0)
        { // buffer is completely within
            memcpy(scratch_buff, time_print_buff + curr_src_start, NUM_COLUMNS);
        }
        else
        { // almost at the end, only some chunks left to copy, start with filling up from the start again
            memcpy(scratch_buff, time_print_buff + curr_src_start, NUM_COLUMNS + overhang);
            memcpy(scratch_buff + NUM_COLUMNS + overhang, time_print_buff, -overhang);
        }

        // increment src index, wrap around if needed
        curr_src_start++;
        if (curr_src_start >= MAX_TIME_PRINT_LEN)
        {
            curr_src_start = 0;
        }

        // print the result
        scratch_buff[NUM_COLUMNS] = 0;
        LCD_I2C_setCursor(0, 0);
        if (LCD_I2C_print(scratch_buff) != ESP_OK)
        {
            PRINT_LOG("Unable to print time to LCD");
            while(1)
            {
                receiveTaskMessage(TASK_LCD, 500, &msg);
            }
        }

        if (lock_state_local != GPS_LOCKED)
        {
            LCD_I2C_setCursor(0, 1);
            snprintf(scratch_buff, sizeof(scratch_buff), "Await GPS lock %c", wait_animation[wait_animation_idx]);
            LCD_I2C_print(scratch_buff);

            wait_animation_idx++;
            if (wait_animation_idx >= ARRAY_LEN(wait_animation))
            {
                wait_animation_idx = 0;
            }
        }
    }
}
