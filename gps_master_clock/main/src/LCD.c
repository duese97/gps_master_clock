#include "LCD.h"
#include "LCM1602.h"

#include "custom_main.h"
#include "slave_clk.h" // to take timezone mutex
#include "menu_handler.h"
#include "bsp.h"


//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

#define NUM_COLUMNS 16
#define NUM_ROWS 2

#define REFRESH_INTERVAL_MS             ( 500 / portTICK_PERIOD_MS ) 

// '13:08:00 15.11.2025 DST: 0    ' = 26 chars + 4 spaces + 1 null
#define MAX_TIME_PRINT_LEN              30

#define DEBOUNCE_DURATION_MS            ( 50 / portTICK_PERIOD_MS )
// every press length between DEBOUNCE_DURATION_MS..LONG_PRESS_DURATION_MS is
// considered a short press
#define LONG_PRESS_DURATION_MS          ( 500 / portTICK_PERIOD_MS )
#define VERY_LONG_PRESS_DURATION_MS     (3000 / portTICK_PERIOD_MS )

#define DISPLAY_TURN_OFF_INITAL_MS      (10 * 60 * 1000) // when booting we might want to stay on a bit longer
#define DISPLAY_TURN_OFF_MS             (1 * 60 * 1000)

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
    STATUS_CURRENT_UPTIME,
    STATUS_SLAVE_CLOCK,
    STATUS_DRIFT_TOTAL,
    STATUS_LAST_CONNECTED,
    NUM_STATUS_IDX
};


//---------------------------------------------------------------------------
// Local constants
//---------------------------------------------------------------------------

static const char wait_animation[] = {'|', '/', '-', GRAM_BACKSLASH_IDX};
static const uint8_t backslash_charmap[] =
{
    0b00000,
    0b10000,
    0b01000,
    0b00100,
    0b00010,
    0b00001,
    0b00000,
    0b00000
};
static const uint8_t back_icon_charmap[] =
{
    0b01110,
    0b00001,
    0b00001,
    0b00101,
    0b01101,
    0b11110,
    0b01100,
    0b00100
};
static const uint8_t dish_icon_charmap[] =
{
    0b00000,
    0b01000,
    0b01100,
    0b00110,
    0b01011,
    0b01000,
    0b11100,
    0b00000
};
static const uint8_t wave_icon_charmap[] =
{
    0b10101,
    0b10100,
    0b10011,
    0b01000,
    0b00111,
    0b00000,
    0b00000,
    0b00000
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
            LCD_I2C_printf(SUM_ICO_STR" lag %8lums", ram_mirror.total_pos_time_corrected_ms);
            break;
        }
        case STATUS_CORRECTION_NEG:
        {
            LCD_I2C_printf(SUM_ICO_STR" lead%8lums", ram_mirror.total_neg_time_corrected_ms);
            break;
        }
        case STATUS_TOTAL_UPTIME:
        case STATUS_CURRENT_UPTIME:
        { // print the uptime in a well readable form
            uint32_t uptime_val =  status_screen_idx == STATUS_TOTAL_UPTIME ? ram_mirror.total_operating_seconds : ram_shared.operating_seconds;
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
            if (status_screen_idx == STATUS_TOTAL_UPTIME)
            {
                LCD_I2C_printf(SUM_ICO_STR" uptime %6lu%c", uptime_val, uptime_unit);
            }
            else
            {
                LCD_I2C_printf("Uptime %8lu%c", uptime_val, uptime_unit);
            }
            break;
        }
        case STATUS_SLAVE_CLOCK:
        {
            uint8_t hours = ram_mirror.current_slave_minutes_12o_clock / 60;
            uint8_t minutes = ram_mirror.current_slave_minutes_12o_clock % 60;
            LCD_I2C_printf("Slave Clk  %02u:%02u", hours, minutes);
            break;
        }
        case STATUS_DRIFT_TOTAL:
        {
            if (ram_shared.drift_total_us == INT64_MAX)
            {
                LCD_I2C_printf(SUM_ICO_STR" drift  ?????ms");
            }
            else
            {
                int msec = USEC_TO_MS(ram_shared.drift_total_us);
                LCD_I2C_printf(SUM_ICO_STR" drift  %+5dms", msec);
            }
            break;
        }
        case STATUS_LAST_CONNECTED:
        {
            LCD_I2C_printf(DISH_ICO_STR WAVE_ICO_STR " %8lds ago",
                (int32_t)(USEC_TO_S(ram_shared.gps_last_connected_us)));
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
    static task_msg_t msg_press = {.dst = TASK_LCD, .cmd = TASK_CMD_BTN_PRESS, .btn_state = BTN_NO_PRESS};
    static task_msg_t msg_led_toggle = {.dst = TASK_LCD, .cmd = TASK_CMD_TOGGLE_LED};

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int btn_lvl = gpio_get_level(USR_BUTTON_IO);
    uint32_t tim_period = 0;
    bool restart = false;

    switch (msg_press.btn_state)
    {
        case BTN_NO_PRESS:
        {            
            if (timer_triggered)
            { // should not happen, only for sanity: must always be called by ISR
                restart = true;
            }
            else
            {
                msg_press.btn_state = BTN_DEBOUNCE; // reset state back to start
                tim_period = DEBOUNCE_DURATION_MS; // wait for voltage to stabilize
                gpio_set_intr_type(USR_BUTTON_IO, GPIO_INTR_DISABLE); // ignore any ISR
            }
            break;
        }
        case BTN_DEBOUNCE:
        {
            if (btn_lvl == USR_BUTTON_PRESS_LVL && timer_triggered == true) // check timer state, in case any edge ISR was still pending
            {
                msg_press.btn_state = BTN_SHORT_PRESS;
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
                sendTaskMessageISR(&msg_led_toggle); // indicate that long press is ready

                msg_press.btn_state = BTN_LONG_PRESS;
                tim_period = VERY_LONG_PRESS_DURATION_MS;
            }
            else if (timer_triggered == false) // button was released
            {
                sendTaskMessageISR(&msg_press);
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
                msg_press.btn_state = BTN_VERY_LONG_PRESS;
                sendTaskMessageISR(&msg_led_toggle); // indicate that very long press is ready
            }
            else if (timer_triggered == false) // was released
            {
                sendTaskMessageISR(&msg_press);
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
                sendTaskMessageISR(&msg_press);
            }

            restart = true;
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
        msg_press.btn_state = BTN_NO_PRESS; // reset state
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
    vTaskDelay(2000); // wait for power to stabilize

    // contains the current time, is of fixed length (why +6 -> compiler needs this to remove annoying warning)
    char time_print_buff[MAX_TIME_PRINT_LEN + 1 + 6] = "??:??:?? ??.??.???? DST: ?    ";
    int status_screen_idx = STATUS_START_IDX;
    bool use_display = true;

    task_msg_t msg;
    GPS_LOCK_STATE_t lock_state_local = GPS_LOCK_UNINITIALIZED;
    struct tm tm; // local time struct
    bool is_commissioning = false;

    uint32_t turn_off_time = ESP_IDF_MILLIS() + DISPLAY_TURN_OFF_INITAL_MS; // set inital timeout when screen shall turn off
    
    if (LCD_I2C_begin(NUM_COLUMNS, NUM_ROWS) != ESP_OK)
    { // in case no display was found
        LOG("Unable to setup LCD I2C!");
        use_display = false;
    }
    else
    {
        LOG("LCD init done");
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

    // Special handling: If button is pressed at boot, we directly start the commissioning.
    // Send message to self.
    if (gpio_get_level(USR_BUTTON_IO) == USR_BUTTON_PRESS_LVL)
    {
        msg.dst = TASK_LCD;
        msg.cmd = TASK_CMD_BTN_PRESS;
        msg.btn_state = BTN_VERY_LONG_PRESS;
        sendTaskMessage(&msg);

        LOG("Button pressed at boot, starting commissioning");
    }

    if (use_display)
    {
        LCD_I2C_setCursor(0, 0);
        LCD_I2C_print("GPS Master Clock");
        LCD_I2C_setCursor(0, 1);
        LCD_I2C_print("  2025 D.Weber  ");
    
        // for some reason the driver has no backslash, create one ourself
        // for some other reason writing to GRAM only works after the prints above
        LCD_I2C_createChar(GRAM_BACKSLASH_IDX, backslash_charmap);
        LCD_I2C_createChar(GRAM_BACK_ICON_IDX, back_icon_charmap);
        LCD_I2C_createChar(GRAM_DISH_ICON_IDX, dish_icon_charmap);
        LCD_I2C_createChar(GRAM_WAVE_ICON_IDX, wave_icon_charmap);

        vTaskDelay(1000);

        if (ram_mirror.pwr_bad)
        {
            LCD_I2C_setCursor(0, 0);
            LCD_I2C_print("Power outage    ");
            LCD_I2C_setCursor(0, 1);
            LCD_I2C_print("Recovering...   ");

            ram_mirror.pwr_bad = false; // ACK flag, once we displayed it
            store_ram_mirror();

            vTaskDelay(1000);
        }
    }



    while(1)
    {
        if (receiveTaskMessage(TASK_LCD, portMAX_DELAY, &msg) == false)
            continue; // some error happened

        switch(msg.cmd)
        {
            case TASK_CMD_TOGGLE_LED:
            {
                gpio_set_level(GPIO_LED, LED_ON_LVL); // disable in any case
                vTaskDelay(100); // small delay, mostly used in menus anyway, not too bad stalling the LCD task
                gpio_set_level(GPIO_LED, LED_OFF_LVL); // disable in any case
                break;
            }
            case TASK_CMD_GPS_LOCK_STATE:
            {
                lock_state_local = msg.lock_state;
                break;
            }
            case TASK_CMD_LOCAL_TIME:
            {
                // format the new time into local buffer
                tm = msg.local_time;
                if (use_display)
                {
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
                xTimerStop(refresh_timer, 1); // avoid refresh timer trying to notify LCD task
                vTaskSuspend(NULL);

                if (use_display)
                {
                    LCD_I2C_backlight(true); // enable again
                }
                xTimerStart(refresh_timer, 1); // resume timer
                break;
            }
            case TASK_CMD_REFRESH_LCD:
            {
                if (use_display == false)
                {
                    continue;
                }
                if((turn_off_time != 0) && (ESP_IDF_MILLIS() > turn_off_time))
                {
                    turn_off_time = 0; // "ack", so that we do not need to come here over and over
                    use_display = false;
                    LCD_I2C_backlight(false);
                    LOG("Turning off display due to inactivity");
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
                LOG("Button press: %u", msg.btn_state);

                use_display = true;
                LCD_I2C_backlight(true); // enable backlight (if it was not already on)
                turn_off_time = ESP_IDF_MILLIS() + DISPLAY_TURN_OFF_MS;

                // handle press, check if commissioning was started or stopped
                bool comm_changed = false;
                is_commissioning = menu_statemachine(msg.btn_state, &comm_changed);

                // if state changed, notify time keeping task
                if (comm_changed)
                {
                    msg.dst = TASK_SLAVE_CLK;
                    msg.cmd = TASK_CMD_COMMISSIONING;

                    // initially assume both are commissioning (or not)
                    msg.comm_line_1 = is_commissioning;
                    msg.comm_line_2 = is_commissioning;
                    sendTaskMessage(&msg);
                }
                break;
            }
            default:
            {
                LOG("Unknow message received: %d", msg.cmd);
                break;
            }
        }
    }
}
