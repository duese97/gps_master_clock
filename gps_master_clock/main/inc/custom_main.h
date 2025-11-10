#ifndef _CUSTOM_MAIN_H_
#define _CUSTOM_MAIN_H_

#define MAX_LOG_WAIT_MS 10              // time to wait for UART to become available
#define MAX_LOG_LEN 256                 // maximum log message length, includes timestamp + func name

#include "freertos/FreeRTOS.h" // for semaphore
#include "esp_timer.h" // for micorsecond timer

//---------------------------------------------------------------------------
// Exported var/func
//---------------------------------------------------------------------------

/* exported variables */
extern SemaphoreHandle_t xUartSemaphore;
extern char print_buf[MAX_LOG_LEN];

/* exported functions */
void serial_print_custom(void);

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