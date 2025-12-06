#ifndef _CUSTOM_MAIN_H_
#define _CUSTOM_MAIN_H_


//---------------------------------------------------------------------------
// Global includes
//---------------------------------------------------------------------------

#include "freertos/FreeRTOS.h" // for semaphore
#include "esp_timer.h" // for micorsecond timer

#include <time.h> // for time_t

// EEPROM emulation
#include "nvs.h"
#include "nvs_flash.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

#define MAX_LOG_WAIT_MS 10              // time to wait for UART to become available
#define MAX_LOG_LEN 512                 // maximum log message length, includes timestamp + func name

#define MINUTES_PER_12H  (12*60)

// The maximum time in minutes the local clock can lead in minutes, before a wraparound must happen
#define MAX_LOCAL_CLOCK_LEAD_MINUTES  5

// The amount of time that the local second timebase can drift away from the 'correct' time.
// A bit of 'wiggle' room is left, since the correction requires starting and stopping the
// timer interrupt.
#define MAX_ALLOWED_ABS_DIFF_SECONDS 2

#define MAX_PLAUSIBLE_DRIFT     100000
#define DRIFT_CORR_THRESHOLD_US 1000 // minimum threshold, at which a correction will be performed

#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))

#define SET_NVS_DEFAULTS 0 // for debugging, set to 1 and flash to restore NVS defaults
#define RAM_MIRROR_VALID_MAGIC 0xDEADBEEF // value to indicate the RAM mirror can be used

#define NVS_NAMESPACE   "STORAGE"
#define KEY_RAM_MIRROR  "RM"

#define USE_TESTCODE 1


//---------------------------------------------------------------------------
// Enums
//---------------------------------------------------------------------------
typedef enum
{
  TASK_LCD,
  TASK_SLAVE_CLK,
  TASK_TIMER,
} task_type_t;


/* task messaging */
typedef enum
{
  TASK_CMD_START = 0,
  TASK_CMD_SECOND_TICK = TASK_CMD_START,

  TASK_CMD_COMMISSIONING,

  TASK_CMD_SIMULATE_SECOND_TICK,
  TASK_CMD_SLAVE_ADVANCE_MINUTES,

  TASK_CMD_GPS_LOCK_STATE,
  TASK_CMD_BTN_PRESS,
  TASK_CMD_REFRESH_LCD,

  TASK_CMD_GPS_TIME,
  TASK_CMD_LOCAL_TIME,
  TASK_CMD_SHUTDOWN,

  NUM_TASK_CMD
} task_cmd_t;

typedef enum
{
    GPS_LOCK_UNINITIALIZED, // lock not yet set
    GPS_LOCK_LOST, // no communication (with module) possible
    GPS_LOCKED, // GPS signal received
} GPS_LOCK_STATE_t;

typedef enum
{
    BTN_NO_PRESS,
    BTN_DEBOUNCE,
    BTN_SHORT_PRESS,
    BTN_LONG_PRESS,
    BTN_VERY_LONG_PRESS,
}btn_state_t;

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
typedef struct
{
  task_type_t dst; // destination of message
  task_cmd_t cmd;
  union // payload, can be unused
  {
    struct
    {
      time_t utc_time;
      int64_t us_timestamp;
    };
    GPS_LOCK_STATE_t lock_state;
    struct tm local_time;
    btn_state_t btn_state;
    uint8_t slave_advance_minutes;
    bool commissioning;
  };
} task_msg_t;

// Data for EEPROM (emulation) storage, mirrored in RAM
typedef struct
{
  time_t last_connected_utc;
  
  int32_t current_slave_minutes_12o_clock; // local minutes after 12 o clock position

  // time related stats
  uint32_t total_pos_time_corrected_ms;
  uint32_t total_neg_time_corrected_ms;
  uint32_t total_operating_seconds; // total time the device was powered

  uint32_t mirror_saved_times; // how many times the RAM mirror was persisted
  uint32_t magic_word; // to easily determine if the struct contains valid data

  // settings for the pulse waveform
  uint16_t pulse_len_ms;
  uint16_t pulse_pause_ms;

  bool hbridge_last_pol;

} ram_mirror_t;

// global shared RAM data, which is not persisted
typedef struct
{
  int64_t drift_total_us;
  int64_t gps_last_connected_us;
  uint32_t operating_seconds; // seconds since last boot
} ram_shared_t;

//---------------------------------------------------------------------------
// Exported var/func
//---------------------------------------------------------------------------

/* exported variables */
extern SemaphoreHandle_t xUartSemaphore;
extern char print_buf[MAX_LOG_LEN];

extern ram_mirror_t ram_mirror;
extern const ram_mirror_t ram_mirror_default;
extern ram_shared_t ram_shared;


/* exported functions */
void serial_print_custom(void);
bool receiveTaskMessage(task_type_t dst, uint32_t timeout, task_msg_t *msg);
bool sendTaskMessage(task_msg_t *msg);
bool sendTaskMessageISR(task_msg_t *msg);

esp_err_t store_ram_mirror(void);

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

#define SEC_TO_US(sec)    (sec * 1000000LL)
#define SEC_TO_MS(sec)    (sec * 1000)
#define USEC_TO_MS(usec)  (usec / 1000)
#define USEC_TO_S(usec)   (usec / 1000000LL)
#define SEC_TO_H(sec)     (sec / 3600)
#define SEC_TO_DAY(sec)   (sec / 3600 / 24)

#endif // _CUSTOM_MAIN_H_