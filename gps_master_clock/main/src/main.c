#include "custom_main.h"

#include <stdio.h>
#include <string.h>

// peripherals
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bsp.h"
#include "esp_system.h" // misc, for reset reason

// for OS methods
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// to get the task handles
#include "neo6m.h"
#include "timekeep.h"
#include "LCD.h"

#define MIN_PWR_BAD_CNT     100     // number of times power bad has to be observed for shutdown
#define MIN_PWR_GOOD_CNT    10000   // number of subsequent power good observations to normally resume

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
#define STACKSIZE_NEO6M     4096
#define STACKSIZE_TIMEKEEP  2028
#define STACKSIZE_LCD       4096

/* TASK */
enum
{
    // priorities (higher number = higher prio)
    TASK_PRIO_LCD = 1,
    TASK_PRIO_TIMEKEEP,
    TASK_PRIO_NEO6M,
};

// task stacks, task handles (for inter task communication) and messaging
SETUP_TASK_VARS(LCD, STACKSIZE_LCD, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS(TIMEKEEP, STACKSIZE_TIMEKEEP, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS(NEO6M, STACKSIZE_NEO6M, QUEUE_STORAGE_GENERAL);

// for fast and uncomplicated assignment of task ID<->queue
static const QueueHandle_t *handleLookup[] =
{
        [TASK_LCD]      = &queueHandleLCD,
        [TASK_TIMEKEEP] = &queueHandleTIMEKEEP,
};

// for logging
SemaphoreHandle_t xUartSemaphore;
char print_buf[MAX_LOG_LEN];

// default values
static const ram_mirror_t rm_dflt =
{
    .pulse_len_ms = 100,
    .pulse_pause_ms = 100,
    .magic_word = RAM_MIRROR_VALID_MAGIC,
};

// Place the ram mirror into RTC RAM. In case of a SW failure we could be able to
// retrieve the last saved values and store them in NVS.
RTC_DATA_ATTR ram_mirror_t rm;

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

#if SET_NVS_DEFAULTS == 0
static esp_err_t load_nvs_data(nvs_handle_t nvs_handle)
{
    size_t value_len = sizeof(ram_mirror_t);
    esp_err_t err = nvs_get_blob(nvs_handle, KEY_RAM_MIRROR, (void *)&rm, &value_len);
    if (err != ESP_OK)
    {
        PRINT_LOG("Unable to obtain data, error: %d", err);
    }    
    return err;
}
#endif // SET_NVS_DEFAULTS == 0

static esp_err_t save_nvs_data(nvs_handle_t nvs_handle)
{
    size_t value_len = sizeof(ram_mirror_t);
    rm.mirror_saved_times++;
    esp_err_t err = nvs_set_blob(nvs_handle, KEY_RAM_MIRROR, (void *)&rm, value_len);
    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
    }
    if (err != ESP_OK)
    {
        PRINT_LOG("Unable to store data, error: %d", err);
    }
    else
    {
        PRINT_LOG("Performed store #%lu", rm.mirror_saved_times);
    }
    return err;
}


static esp_err_t inital_nvs_load(bool soft_reset)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        err = nvs_flash_erase();
        if (err == ESP_OK)
        {
            err = nvs_flash_init();
        }
    }
    
    if (err != ESP_OK)
        return err;
    
    nvs_handle_t nvs_handle = {0};

    // Open NVS handle
    PRINT_LOG("Opening Non-Volatile Storage (NVS) handle...");
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

#if SET_NVS_DEFAULTS == 0
    if (err == ESP_OK)
    {
        bool loaded_from_nvs = false;
    
        // Check if the RAM mirror can be used
        if (soft_reset)
        { // there is hope to load a valid ram mirror
            if (rm.magic_word == RAM_MIRROR_VALID_MAGIC)
            { // back up the data, in case a future power cycle happens
                err = save_nvs_data(nvs_handle);
                PRINT_LOG("Trying to save valid RAM mirror to NVS...");
            }
            else
            { // try to read from NVS
                err = load_nvs_data(nvs_handle);
                loaded_from_nvs = true;
                PRINT_LOG("Trying to load NVS to RAM mirror...");
            }
        }
        else // 'hard' reset, do not even try to load ram mirror
        {
            err = load_nvs_data(nvs_handle);
            loaded_from_nvs = true;
            PRINT_LOG("Trying to load NVS to RAM mirror after hard reset...");
        }
    
        if (err == ESP_OK)
        {
            if (loaded_from_nvs && rm.magic_word != RAM_MIRROR_VALID_MAGIC) // load worked, but somehow got garbage
            {
                err = ESP_ERR_INVALID_CRC;
                PRINT_LOG("Unexpected magic word in loaded data: %08lX", rm.magic_word);
            }
        }
        else if (loaded_from_nvs)
        {
            PRINT_LOG("Nothing to load from");
        }
    }
    
    if (err != ESP_OK) // in case any of the operations failed: Try to re-init with defaults
#endif // SET_NVS_DEFAULTS == 0
    {
        PRINT_LOG("Re-initializing NVS...");
        rm = rm_dflt;
        err = save_nvs_data(nvs_handle);
        if (err == ESP_OK)
        {
            PRINT_LOG("Set defaults done");
        }
        else
        {
            PRINT_LOG("Unable to set defaults");
        }
    }
    PRINT_LOG(
        "Using config:\n"
        "\tcurrent_minutes_12o_clock: %d\n"
        "\ttotal_pos_time_corrected: %lu total_neg_time_corrected: %lu\n"
        "\tmirror_saved_times: %lu\n"
        "\tpulse_len_ms: %u pulse_pause_ms: %u\n"
        "\tlast_connected_utc:%lld",
        rm.current_minutes_12o_clock,
        rm.total_pos_time_corrected, rm.total_neg_time_corrected,
        rm.mirror_saved_times,
        rm.pulse_len_ms, rm.pulse_pause_ms,
        rm.last_connected_utc
    );

    PRINT_LOG("Closing NVS");
    nvs_close(nvs_handle);

    return err;
}

static void wait_shutdown(void)
{
    const TaskHandle_t *checkHandles[] =
    {
        &taskHandleLCD,
        &taskHandleTIMEKEEP,
    };
    uint8_t num_handles =sizeof(checkHandles) / sizeof(checkHandles[0]);

    bool done = false;
    TaskStatus_t tmpStat;

    while (!done)
    {
        done = true;
        for (uint8_t idx = 0; idx < num_handles; idx++)
        {
            const TaskHandle_t *curr_handle = checkHandles[idx];

            if (curr_handle == NULL) // no need to check if handle not initialized
                continue;

            vTaskGetInfo(*curr_handle, &tmpStat, pdFALSE /* skip stack check*/, eInvalid /*get task state */);
            if (tmpStat.eCurrentState != eSuspended) // only need to delay if the task is actually not yet suspended
                vTaskDelay(1);
            done &= tmpStat.eCurrentState == eSuspended;
        }
    }
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

    if (dst < (sizeof(handleLookup) / sizeof(handleLookup[0])))
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

bool sendTaskMessageISR(task_msg_t *msg)
{
    QueueHandle_t handle = NULL;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // not woken any task at start of ISR

    if (msg->dst < sizeof(handleLookup) / sizeof(handleLookup[0]))
    {
        handle = *(handleLookup[msg->dst]); // determine queue handle
    }

    if (handle)
    {
        xQueueSendFromISR(handle, (void *)msg, &xHigherPriorityTaskWoken);
    }

    return xHigherPriorityTaskWoken;
}

esp_err_t store_ram_mirror(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = save_nvs_data(nvs_handle);
    }

    nvs_close(nvs_handle);
    return err;
}

void app_main(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    init_serial_print();
    gpio_set_direction(POWER_GOOD_IO, GPIO_MODE_INPUT);

    PRINT_LOG("\nStarting application. Reset reason: %d\n", reason);

    // If the reset reason is not a power cycle, it's likely due to some SW issue
    bool soft_reset = reason != ESP_RST_UNKNOWN && reason != ESP_RST_POWERON;
    esp_err_t err= inital_nvs_load(soft_reset);
    if (err != ESP_OK)
    {
        PRINT_LOG("Error (%s) while handling NVS!", esp_err_to_name(err));
    }

    SETUP_QUEUE(NEO6M, QUEUE_LEN_GENERAL);
    SETUP_QUEUE(TIMEKEEP, QUEUE_LEN_GENERAL);
    SETUP_QUEUE(LCD, QUEUE_LEN_GENERAL);

    taskHandleNEO6M = xTaskCreateStatic(
        neo6M_Task,
        "neo6M",
        STACKSIZE_NEO6M,
        NULL,
        TASK_PRIO_NEO6M,
        taskStackNEO6M,
        &taskBufferNEO6M
    );

    taskHandleTIMEKEEP = xTaskCreateStatic(
        timekeep_Task,
        "timekeep",
        STACKSIZE_TIMEKEEP,
        NULL,
        TASK_PRIO_TIMEKEEP,
        taskStackTIMEKEEP,
        &taskBufferTIMEKEEP
    );

    taskHandleLCD = xTaskCreateStatic(
        LCD_Task,
        "LCD",
        STACKSIZE_LCD,
        NULL,
        TASK_PRIO_LCD,
        taskStackLCD,
        &taskBufferLCD
    );

    int power_bad_count = 0, power_good_count = 0;
    while(1)
    {
        vTaskDelay(1);
        if (gpio_get_level(POWER_GOOD_IO) == 0) // periodically check the power good pin
        {
            if (power_bad_count < MIN_PWR_BAD_CNT)
            {
                power_good_count = 0; // reset good counter
                power_bad_count++;
                if (power_bad_count == MIN_PWR_BAD_CNT) // only do it once
                {
                    PRINT_LOG("Power bad, shutting down tasks");
                    
                    // Tasks which are a bit more delicate, let them finish what they are doing right now
                    task_msg_t msg = {.cmd = TASK_CMD_SHUTDOWN, .dst = TASK_TIMEKEEP };
                    sendTaskMessage(&msg);
                    msg.dst = TASK_LCD;
                    sendTaskMessage(&msg);

                    wait_shutdown();
                    PRINT_LOG("Shutdown complete, storing..");

                    // it's now OK to save the system state
                    store_ram_mirror();
                }
            }
        }
        else if (power_bad_count)
        {
            if (power_good_count < MIN_PWR_GOOD_CNT)
            {
                power_good_count++;
                if (power_good_count == MIN_PWR_GOOD_CNT) // only do it once
                {
                    PRINT_LOG("Power recovered, resuming tasks");

                    power_bad_count = 0;
                    power_good_count = 0;

                    // resume all tasks
                    vTaskResume(taskHandleLCD);
                    vTaskResume(taskHandleTIMEKEEP);
                }
            }
        } // else: do nothing when no power bad detected
    }
}