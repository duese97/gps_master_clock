#ifndef _CUSTOM_MAIN_H_
#define _CUSTOM_MAIN_H_


//---------------------------------------------------------------------------
// Global includes
//---------------------------------------------------------------------------

#include "freertos/FreeRTOS.h" // for semaphore
#include "esp_timer.h" // for micorsecond timer

#include <time.h> // for time_t

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

#define MAX_LOG_WAIT_MS 10              // time to wait for UART to become available
#define MAX_LOG_LEN 256                 // maximum log message length, includes timestamp + func name

// The maximum time in minutes the local clock can lead in minutes, before a wraparound must happen
#define MAX_LOCAL_CLOCK_LEAD_MINUTES  5

// The amount of time that the local second timebase can drift away from the 'correct' time.
// A bit of 'wiggle' room is left, since the correction requires starting and stopping the
// timer interrupt.
#define MAX_ALLOWED_LOCAL_CLOCK_DRIFT_SECONDS 2

#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))


//---------------------------------------------------------------------------
// Enums
//---------------------------------------------------------------------------
typedef enum
{
  TASK_LCD,
  TASK_TIMEKEEP,
  TASK_NEO6M,
} task_type_t;


/* task messaging */
typedef enum
{
  TASK_CMD_START = 0,
  SECOND_TRIGGER = TASK_CMD_START,
  GPS_LOCK_STATE,
  LOCAL_TIME,
  NUM_TASK_CMD
} task_cmd_t;

typedef enum
{
    GPS_LOCK_UNINITIALIZED, // lock not yet set
    GPS_LOCK_LOST, // no communication (with module) possible
    GPS_LOCKED, // GPS signal received
} GPS_LOCK_STATE_t;

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
typedef struct
{
  task_type_t dst; // destination of message
  task_cmd_t cmd;
  union // payload, can be unused
  {
    time_t utc_time;
    GPS_LOCK_STATE_t lock_state;
    struct tm local_time;
  };
} task_msg_t;

// Data for EEPROM (emulation) storage, mirrored in RAM
typedef struct
{
  int current_minutes_12o_clock; // local minutes after 12 o clock position

  // time related stats
  uint32_t total_pos_time_corrected;
  uint32_t total_neg_time_corrected;
  uint32_t total_uptime_seconds;

  // settings for the pulse waveform
  uint16_t pulse_len_ms;
  uint16_t pulse_pause_ms;

} ram_mirror_t;

//---------------------------------------------------------------------------
// Exported var/func
//---------------------------------------------------------------------------

/* exported variables */
extern SemaphoreHandle_t xUartSemaphore;
extern char print_buf[MAX_LOG_LEN];

extern ram_mirror_t rm;

/* exported functions */
void serial_print_custom(void);
bool receiveTaskMessage(task_type_t dst, uint32_t timeout, task_msg_t *msg);
bool sendTaskMessage(task_msg_t *msg);
bool sendTaskMessageISR(task_msg_t *msg);


/* exported macros */
#define ESP_IDF_MILLIS() (uint32_t)((esp_timer_get_time() / 1000))

// workaround in case no varargs given
#define VA_ARGS(...) , ##__VA_ARGS__

// thread safe printing/sharing of UART
#define PRINT_LOG(fmt, ...)                                                                 \
  do                                                                                        \
  {                                                                                         \
    if (xUartSemaphore == NULL)                                                             \
      break;                                                                                \
    if (xSemaphoreTake(xUartSemaphore, MAX_LOG_WAIT_MS) != pdTRUE)                          \
      break;                                                                                \
    snprintf(print_buf, MAX_LOG_LEN,                                                        \
             "%08lu %s(): " fmt "\n", ESP_IDF_MILLIS(), __FUNCTION__ VA_ARGS(__VA_ARGS__)); \
    serial_print_custom();                                                                  \
    xSemaphoreGive(xUartSemaphore);                                                         \
  } while (0)

#endif // _CUSTOM_MAIN_H_