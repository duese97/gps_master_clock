#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp.h"
#include "custom_main.h"

#include "neo6m.h"


#define SETUP_TASK_VARS(taskname, stacksize, queueElemByteCnt) \
    static StackType_t taskStack##taskname[stacksize];         \
    static TaskHandle_t taskHandle##taskname;                  \
    static StaticTask_t taskBuffer##taskname;                  \
    static QueueHandle_t queueHandle##taskname;                \
    static uint8_t queueStorageArea##taskname[queueElemByteCnt]

#define SETUP_TASK_VARS_NO_QUEUE(taskname, stacksize)  \
    static StackType_t taskStack##taskname[stacksize]; \
    static TaskHandle_t taskHandle##taskname;          \
    static StaticTask_t taskBuffer##taskname;

#define SETUP_QUEUE(taskname, queueElemCnt)                  \
    static StaticQueue_t queue##taskname;                    \
    queueHandle##taskname = xQueueCreateStatic(              \
        sizeof(queueStorageArea##taskname) / QUEUE_ELEM_LEN, \
        QUEUE_ELEM_LEN,                                      \
        queueStorageArea##taskname,                          \
        &queue##taskname)

        /* Messaging */
#define QUEUE_LEN_GENERAL 3
#define QUEUE_ELEM_LEN sizeof(task_msg_t)
#define QUEUE_STORAGE_GENERAL (QUEUE_LEN_GENERAL * QUEUE_ELEM_LEN)
#define QUEUE_MAX_BLOCK_MS 100

// static stack sizes (printf related stuff needs a lot of RAM)
#define STACKSIZE_NEO6M 4096
#define STACKSIZE_LCD   2048

/* TASK */
enum
{
    // priorities (higher number = higher prio)
    TASK_PRIO_LCD = 1,
    TASK_PRIO_NEO6M,
};

// task stacks, task handles (for inter task communication) and messaging
SETUP_TASK_VARS(NEO6M, STACKSIZE_NEO6M, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS(LCD, STACKSIZE_LCD, QUEUE_STORAGE_GENERAL);

// for fast and uncomplicated assignment of task ID<->queue
static const QueueHandle_t *handleLookup[] =
{
        [TASK_LCD] = &queueHandleNEO6M,
        [TASK_NEO6M] = &queueHandleLCD,
};

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

bool receiveTaskMessage(task_type_t dst, uint32_t timeout, task_msg_t *msg)
{
    bool success = false;
    QueueHandle_t handle = NULL;

    if (dst < sizeof(handleLookup) / sizeof(handleLookup[0]))
    {
        handle = *(handleLookup[dst]); // determine queue handle
    }
    if (!handle)
    {
        PRINT_LOG("Invalid destination task %d", dst);
    }
    else if (xQueueReceive(handle, (void *)msg, timeout) == pdTRUE)
    {
        success = true;
    }
    return success;
}

bool sendTaskMessage(task_msg_t *msg)
{
    bool success = false;
    QueueHandle_t handle = NULL;

    if (msg->dst < sizeof(handleLookup) / sizeof(handleLookup[0]))
    {
        handle = *(handleLookup[msg->dst]); // determine queue handle
    }

    if (!handle)
    {
        PRINT_LOG("Invalid destination task %d", msg->dst);
    }
    else if (xQueueSend(handle, (void *)msg, QUEUE_MAX_BLOCK_MS) != pdTRUE)
    {
        PRINT_LOG("Queue send failed, dst: %u, cmd: %u", msg->dst, msg->cmd);
    }
    else
    {
        success = true;
    }
    return success;
}

void app_main(void)
{
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED, 0);

    init_serial_print();

    taskHandleNEO6M = xTaskCreateStatic(
        neo6M_Task,
        "neo6M",
        STACKSIZE_NEO6M,
        NULL,
        TASK_PRIO_NEO6M,
        taskStackNEO6M,
        &taskBufferNEO6M
    );

    while(1)
    {
        vTaskDelay(1000);
        gpio_set_level(GPIO_LED, !gpio_get_level(GPIO_LED));
    }
}