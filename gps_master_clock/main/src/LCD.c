#include "LCD.h"
#include "LCM1602.h"

#include "custom_main.h"
#include "menu_handler.h"
#include "bsp.h"


//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

#define NUM_COLUMNS 16
#define NUM_ROWS 2

#define GRAM_BACKSLASH_INDEX 1

#define REFRESH_INTERVAL_MS     ( 500 / portTICK_PERIOD_MS ) 

// '13:08:00 15.11.2025 DST: 0    ' = 26 chars + 4 spaces + 1 null
#define MAX_TIME_PRINT_LEN 30

#define DEBOUNCE_DURATION_MS    ( 50 / portTICK_PERIOD_MS )
// every press length between DEBOUNCE_DURATION_MS..LONG_PRESS_DURATION_MS is
// considered a short press
#define LONG_PRESS_DURATION_MS  ( 500 / portTICK_PERIOD_MS )
#define VERY_LONG_PRESS_DURATION_MS (3000 / portTICK_PERIOD_MS )

//---------------------------------------------------------------------------
// Enums
//---------------------------------------------------------------------------

enum
{
    STATUS_START_IDX,
    STATUS_GPS_LOCK = STATUS_START_IDX,
    STATUS_CORRECTION_POS,
    STATUS_CORRECTION_NEG,
    STATUS_TOTAL_UPTIME,
    STATUS_CLOCK_FACE_TIME,
    NUM_STATUS_IDX
};


//---------------------------------------------------------------------------
// Local constants
//---------------------------------------------------------------------------

static const char wait_animation[] = {'|', '/', '-', GRAM_BACKSLASH_INDEX};
static const uint8_t backslash_charmap[] =
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


//---------------------------------------------------------------------------
// Local variables
//---------------------------------------------------------------------------

static TimerHandle_t btn_timer;
static TimerHandle_t refresh_timer;
static char scratch_buff[NUM_COLUMNS + 1 + 2 /*so that compiler does not complain*/]; // to temporarily format the time etc.

//---------------------------------------------------------------------------
// Local functions
//---------------------------------------------------------------------------

static void btn_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    btn_handler(true);
}

static void refresh_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static task_msg_t msg = {.dst = TASK_LCD, .cmd = TASK_CMD_REFRESH_LCD};
    sendTaskMessage(&msg);
}

static void LCD_print_default_displays(char* time_print_buff, int status_screen_idx, GPS_LOCK_STATE_t lock_state_local)
{
    static int curr_src_start = 0; // index where to start copying from the time buffer
    static int wait_animation_idx = 0;

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
    LCD_I2C_print(scratch_buff);

    LCD_I2C_setCursor(0, 1);
    switch(status_screen_idx)
    {
        case STATUS_GPS_LOCK:
        {
            if (lock_state_local == GPS_LOCKED)
            {
                LCD_I2C_print("GPS locked      ");
            }
            else
            {
                LCD_I2C_printf("Await GPS lock %c", wait_animation[wait_animation_idx]);

                wait_animation_idx++;
                if (wait_animation_idx >= ARRAY_LEN(wait_animation))
                    wait_animation_idx = 0;
            }
            break;
        }
        case STATUS_CORRECTION_POS:
        {
            LCD_I2C_printf("Lag:   %8lus", rm.total_pos_time_corrected);
            break;
        }
        case STATUS_CORRECTION_NEG:
        {
            LCD_I2C_printf("Lead:  %8lus", rm.total_neg_time_corrected);
            break;
        }
        case STATUS_TOTAL_UPTIME:
        { // print the uptime in a well readable form
            uint32_t uptime_val =  rm.total_uptime_seconds;
            char uptime_unit = 's';
            if (uptime_val > 3600)
            {
                uptime_val /= 3600;
                if (uptime_val > 24)
                {
                    uptime_val /= 24;
                    uptime_unit = 'd';
                }
                else
                {
                    uptime_unit = 'h';
                }
            }
            LCD_I2C_printf("Uptime %8lu%c", uptime_val, uptime_unit);
            break;
        }
        case STATUS_CLOCK_FACE_TIME:
        {
            uint8_t hours = rm.current_minutes_12o_clock / 60;
            uint8_t minutes = rm.current_minutes_12o_clock % 60;
            LCD_I2C_printf("Clock:     %02u:%02u", hours, minutes);
            break;
        }
        default:
        {
            break;
        }
    }
}

//---------------------------------------------------------------------------
// Global functions
//---------------------------------------------------------------------------
void btn_handler(bool timer_triggered)
{
    static task_msg_t msg = {.dst = TASK_LCD, .cmd = TASK_CMD_BTN_PRESS, .btn_state = BTN_NO_PRESS};


    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int btn_lvl = gpio_get_level(USR_BUTTON_IO);
    uint32_t tim_period = 0;
    bool restart = false;

    switch (msg.btn_state)
    {
        case BTN_NO_PRESS:
        {
            if (timer_triggered)
            { // should not happen, only for sanity: must always be called by ISR
                restart = true;
            }
            else
            {
                msg.btn_state = BTN_DEBOUNCE; // reset state back to start
                tim_period = DEBOUNCE_DURATION_MS; // wait for voltage to stabilize
                gpio_set_intr_type(USR_BUTTON_IO, GPIO_INTR_DISABLE); // ignore any ISR
            }
            break;
        }
        case BTN_DEBOUNCE:
        {
            if (btn_lvl == USR_BUTTON_PRESS_LVL && timer_triggered == true) // check timer state, in case any edge ISR was still pending
            {
                msg.btn_state = BTN_SHORT_PRESS;
                tim_period = LONG_PRESS_DURATION_MS;
                gpio_set_intr_type(USR_BUTTON_IO, GPIO_INTR_POSEDGE); // await rising edge
            }
            else // not yet settled
            {
                restart = true;
            }
            break;
        }
        case BTN_SHORT_PRESS:
        {
            if (btn_lvl == USR_BUTTON_PRESS_LVL && timer_triggered == true) // still pressed, will be a long press
            {
                msg.btn_state = BTN_LONG_PRESS;
                tim_period = VERY_LONG_PRESS_DURATION_MS;
            }
            else if (timer_triggered == false) // button was released
            {
                sendTaskMessageISR(&msg);
                restart = true;
            }
            else // some weird intermediate state, abort
            {
                restart = true;
            }
            break;
        }
        case BTN_LONG_PRESS:
        {
            if (btn_lvl == USR_BUTTON_PRESS_LVL && timer_triggered == true) // still pressed, will be a very long press
            {
                msg.btn_state = BTN_VERY_LONG_PRESS;
            }
            else if (timer_triggered == false) // was released
            {
                sendTaskMessageISR(&msg);
                restart = true;
            }
            else // everything else -> error
            {
                restart = true;
            }
            break;
        }
        case BTN_VERY_LONG_PRESS:
        {
            if (timer_triggered == false) // was released
            {
                sendTaskMessageISR(&msg);
                restart = true;
            }
            else // everything else -> error
            {
                restart = true;
            }
            break;
        }
        default:
        {
            // unknown state
            restart = true;
            break;
        }
    }

    if (restart)
    {
        if (timer_triggered == false) // if we are in a timer context -> its already stopped
        {
            xTimerStopFromISR(btn_timer, &xHigherPriorityTaskWoken); // make sure the timer is off
        }
        msg.btn_state = BTN_NO_PRESS; // reset state
        gpio_set_intr_type(USR_BUTTON_IO, GPIO_INTR_NEGEDGE);
    }
    else if (tim_period)
    {
        if(timer_triggered)
        {
            xTimerChangePeriod(btn_timer, tim_period, 1);
        }
        else
        {
            xTimerChangePeriodFromISR(btn_timer, tim_period, &xHigherPriorityTaskWoken);
        }
    }

    if (xHigherPriorityTaskWoken) // check if task switch happened
    {
        portYIELD_FROM_ISR();
    }
}

void LCD_Task(void *parameter)
{
    // contains the current time, is of fixed length (why +6 -> compiler needs this to remove annoying warning)
    char time_print_buff[MAX_TIME_PRINT_LEN + 1 + 6] = "??:??:?? ??.??.???? DST: ?    ";
    int status_screen_idx = STATUS_START_IDX;
    bool use_display = true;

    task_msg_t msg;
    GPS_LOCK_STATE_t lock_state_local = GPS_LOCK_UNINITIALIZED;
    struct tm tm; // local time struct
    bool is_commissioning = false;

    if (LCD_I2C_begin(NUM_COLUMNS, NUM_ROWS) != ESP_OK)
    { // in case no display was found
        PRINT_LOG("Unable to setup LCD I2C!");
        use_display = false;
    }
    else
    {
        PRINT_LOG("LCD init done");
    }

    btn_timer = xTimerCreate(
                "btn timer",            /* Just a text name, not used by the RTOS kernel. */                    
                DEBOUNCE_DURATION_MS,   /* The timer period in ticks, must be greater than 0. */
                pdFALSE,                /* The timer will not auto-reload themselves when they expire. */
                ( void * ) 0,
                btn_timer_callback
    );

    refresh_timer = xTimerCreate(
                "refresh timer",            /* Just a text name, not used by the RTOS kernel. */                    
                REFRESH_INTERVAL_MS,   /* The timer period in ticks, must be greater than 0. */
                pdTRUE,                /* The timer will not auto-reload themselves when they expire. */
                ( void * ) 0,
                refresh_timer_callback
    );

    xTimerStart(refresh_timer, 10);

    if (use_display)
    {
        LCD_I2C_setCursor(0, 0);
        LCD_I2C_print("GPS Master Clock");
        LCD_I2C_setCursor(0, 1);
        LCD_I2C_print("  2025 D.Weber  ");
    
        // for some reason the driver has no backslash, create one ourself
        // for some other reason writing to GRAM only works after the prints above
        LCD_I2C_createChar(GRAM_BACKSLASH_INDEX, backslash_charmap);
    
        vTaskDelay(1000);
    }

    while(1)
    {
        if (receiveTaskMessage(TASK_LCD, 500, &msg) == true)
        {
            switch(msg.cmd)
            {
                case TASK_CMD_GPS_LOCK_STATE:
                {
                    lock_state_local = msg.lock_state;
                    break;
                }
                case TASK_CMD_LOCAL_TIME:
                {
                    // format the new time into local buffer
                    tm = msg.local_time;
                    snprintf(time_print_buff, sizeof(time_print_buff), "%02u:%02u:%02u %02u.%02u.%04u DST: %1u    ",
                        (uint8_t)tm.tm_hour, (uint8_t)tm.tm_min, (uint8_t)tm.tm_sec,
                        (uint8_t)tm.tm_mday, (uint8_t)(tm.tm_mon + 1), (uint16_t)(tm.tm_year + 1900),
                        (bool)tm.tm_isdst
                    );

                    // Change the status screen
                    if (tm.tm_sec % 5 == 0)
                    {
                        status_screen_idx++;
                        if (status_screen_idx >= NUM_STATUS_IDX)
                        {
                            status_screen_idx = STATUS_START_IDX;
                        }
                    }
                    break;
                }
                case TASK_CMD_SHUTDOWN:
                {
                    if (use_display)
                    {
                        LCD_I2C_backlight(false); // disable backlight to save power
                        LCD_I2C_clear(); // dummy command for backlight to take effect
                    }
                    vTaskSuspend(NULL);

                    if (use_display)
                    {
                        LCD_I2C_backlight(true); // enable again
                    }
                    break;
                }
                case TASK_CMD_REFRESH_LCD:
                {
                    if (use_display == false)
                    {
                        continue;
                    }

                    if (is_commissioning == false)
                    {
                        LCD_print_default_displays(time_print_buff, status_screen_idx, lock_state_local);
                    }
                    else
                    {
                        menu_update();
                    }
                    break;
                }
                case TASK_CMD_BTN_PRESS:
                {
                    // handle press, check if commissioning was started or stopped
                    bool comm_changed = false;
                    is_commissioning = menu_statemachine(msg.btn_state, &comm_changed);

                    // if state changed, notify time keeping task
                    if (comm_changed)
                    {
                        msg.dst = TASK_TIMEKEEP;
                        msg.cmd = TASK_CMD_COMMISSIONING;
                        msg.commissioning = is_commissioning;
                        sendTaskMessage(&msg);
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
}
