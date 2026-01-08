#include "custom_main.h"

#include <stdio.h>
#include <string.h>

// peripherals
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bsp.h"
#include "esp_system.h" // misc, for reset reason
#include "esp_pm.h"

// for OS methods
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// to get the task handles
#include "neo6m.h"
#include "slave_clk.h"
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
    static StaticTask_t taskBuffer##taskname

#define SETUP_QUEUE(taskname, queueElemCnt)                  \
    static StaticQueue_t queue##taskname;                    \
    queueHandle##taskname = xQueueCreateStatic(              \
        sizeof(queueStorageArea##taskname) / QUEUE_ELEM_LEN, \
        QUEUE_ELEM_LEN,                                      \
        queueStorageArea##taskname,                          \
        &queue##taskname)

#define CREATE_TASK_STATIC(taskname)                         \
    xTaskCreateStatic(                                       \
            taskname##_Task,                                 \
            #taskname,                                       \
            STACKSIZE_##taskname,                            \
            NULL,                                            \
            TASK_PRIO_##taskname,                            \
            taskStack##taskname,                             \
            &taskBuffer##taskname                            \
    )

/* Messaging */
#define QUEUE_LEN_GENERAL       3
#define QUEUE_LEN_LOGGING       10
#define QUEUE_ELEM_LEN          sizeof(task_msg_t)
#define QUEUE_STORAGE_GENERAL   (QUEUE_LEN_GENERAL * QUEUE_ELEM_LEN)
#define QUEUE_STORAGE_LOGGING   (QUEUE_LEN_LOGGING * QUEUE_ELEM_LEN)
#define QUEUE_MAX_BLOCK_MS      100

// static stack sizes (printf related stuff needs a lot of RAM)
#define STACKSIZE_NEO6M     4096
#define STACKSIZE_SLAVE_CLK 2028
#define STACKSIZE_LCD       4096
#define STACKSIZE_PWR       2028
#define STACKSIZE_TIMER     4096
#define STACKSIZE_LOGGING   4096

// testing utility
#define MAX_COMMAND_LENGTH  16


enum
{
    TESTCODE_SW_LOCKUP = 0,
    TESTCODE_INC_HOUR,
    TESTCODE_DEC_HOUR,
};

/* TASK */
enum
{
    // priorities (higher number = higher prio)
    TASK_PRIO_LOGGING = 1,
    TASK_PRIO_LCD,
    TASK_PRIO_SLAVE_CLK,
    TASK_PRIO_TIMER,
    TASK_PRIO_NEO6M,
    TASK_PRIO_PWR,
};

// task stacks, task handles (for inter task communication) and messaging
SETUP_TASK_VARS(LCD, STACKSIZE_LCD, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS(SLAVE_CLK, STACKSIZE_SLAVE_CLK, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS(TIMER, STACKSIZE_TIMER, QUEUE_STORAGE_GENERAL);
SETUP_TASK_VARS_NO_QUEUE(NEO6M, STACKSIZE_NEO6M);
SETUP_TASK_VARS_NO_QUEUE(PWR, STACKSIZE_PWR);
SETUP_TASK_VARS(LOGGING, STACKSIZE_LOGGING, QUEUE_STORAGE_LOGGING);


// for fast and uncomplicated assignment of task ID<->queue
static const QueueHandle_t *handleLookup[] =
{
        [TASK_LCD]          = &queueHandleLCD,
        [TASK_SLAVE_CLK]    = &queueHandleSLAVE_CLK,
        [TASK_TIMER]        = &queueHandleTIMER,
        [TASK_LOGGING]      = &queueHandleLOGGING,
};

// for logging
SemaphoreHandle_t xUartSemaphore;
char print_buf[MAX_LOG_LEN];

// default values
const ram_mirror_t ram_mirror_default =
{
    .period_ms  = 300,
    .magic_word = RAM_MIRROR_VALID_MAGIC,
};

// Configure parameters of an UART driver
static const uart_config_t uart_config = {
    .baud_rate  = 115200,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

// Place the ram mirror into RTC RAM. In case of a SW failure we could be able to
// retrieve the last saved values and store them in NVS.
RTC_NOINIT_ATTR ram_mirror_t ram_mirror;


ram_shared_t ram_shared;

static void init_serial_print(void)
{
    xUartSemaphore = xSemaphoreCreateMutex(); // for access to LOGs

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    /* install the driver */
    ESP_ERROR_CHECK(uart_driver_install(LOGGING_UART_PORT, MAX_LOG_LEN /*not needed?*/, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(LOGGING_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LOGGING_UART_PORT, LOGGING_UART_TX, LOGGING_UART_RX, GPIO_NUM_NC, GPIO_NUM_NC));
}

#if SET_NVS_DEFAULTS == 0
static esp_err_t load_nvs_data(nvs_handle_t nvs_handle)
{
    size_t value_len = sizeof(ram_mirror_t);
    esp_err_t err = nvs_get_blob(nvs_handle, KEY_RAM_MIRROR, (void *)&ram_mirror, &value_len);
    if (err != ESP_OK)
    {
        LOG("Unable to obtain data, error: %d", err);
    }    
    return err;
}
#endif // SET_NVS_DEFAULTS == 0

static esp_err_t save_nvs_data(nvs_handle_t nvs_handle)
{
    size_t value_len = sizeof(ram_mirror_t);
    ram_mirror.mirror_saved_times++;
    esp_err_t err = nvs_set_blob(nvs_handle, KEY_RAM_MIRROR, (void *)&ram_mirror, value_len);
    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
    }
    if (err != ESP_OK)
    {
        LOG("Unable to store data, error: %d", err);
    }
    else
    {
        LOG("Performed store #%lu", ram_mirror.mirror_saved_times);
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
    LOG("Opening Non-Volatile Storage (NVS) handle...");
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

#if SET_NVS_DEFAULTS == 0
    if (err == ESP_OK)
    {
        bool loaded_from_nvs = false;
    
        // Check if the RAM mirror can be used
        if (soft_reset)
        { // there is hope to load a valid ram mirror
            LOG("Detected soft reset");
            if (ram_mirror.magic_word == RAM_MIRROR_VALID_MAGIC)
            { // back up the data, in case a future power cycle happens
                LOG("Trying to save valid RAM mirror to NVS...");
                err = save_nvs_data(nvs_handle);
            }
            else
            { // try to read from NVS
                LOG("RAM config could be corrupt (magic word is 0x%08lX), trying to load NVS to RAM mirror...",
                    ram_mirror.magic_word);
                err = load_nvs_data(nvs_handle);
                loaded_from_nvs = true;
            }
        }
        else // 'hard' reset, do not even try to load ram mirror
        {
            err = load_nvs_data(nvs_handle);
            loaded_from_nvs = true;
            LOG("Trying to load NVS to RAM mirror after hard reset...");
        }
    
        if (err == ESP_OK)
        {
            if (loaded_from_nvs && ram_mirror.magic_word != RAM_MIRROR_VALID_MAGIC) // load worked, but somehow got garbage
            {
                err = ESP_ERR_INVALID_CRC;
                LOG("Unexpected magic word in loaded data: %08lX", ram_mirror.magic_word);
            }
        }
        else if (loaded_from_nvs)
        {
            LOG("Nothing to load from");
        }
    }
    
    if (err != ESP_OK) // in case any of the operations failed: Try to re-init with defaults
#endif // SET_NVS_DEFAULTS == 0
    {
        LOG("Re-initializing NVS...");
        ram_mirror = ram_mirror_default;
        err = save_nvs_data(nvs_handle);
        if (err == ESP_OK)
        {
            LOG("Set defaults done");
        }
        else
        {
            LOG("Unable to set defaults");
        }
    }
    LOG(
        "Using config:\n"
        "\tcurrent_slave_minutes_12o_clock: %ld (%02ld:%02ld)\n"
        "\ttotal_pos_time_corrected: %lu total_neg_time_corrected_ms: %lu\n"
        "\tmirror_saved_times: %lu\n"
        "\tperiod_ms: %u\n"
        "\tlast_connected_utc:%lld"
        "\thbridge1_last_pol:%u line2_last_pol:%u"
        "\tmagic_word: 0x%08lX",
        ram_mirror.current_slave_minutes_12o_clock, ram_mirror.current_slave_minutes_12o_clock / 60, ram_mirror.current_slave_minutes_12o_clock % 60,
        ram_mirror.total_pos_time_corrected_ms, ram_mirror.total_neg_time_corrected_ms,
        ram_mirror.mirror_saved_times,
        ram_mirror.period_ms,
        ram_mirror.last_connected_utc,
        ram_mirror.line1_last_pol, ram_mirror.line2_last_pol,
        ram_mirror.magic_word
    );

    LOG("Closing NVS");
    nvs_close(nvs_handle);

    return err;
}

static void wait_shutdown(void)
{
    const TaskHandle_t *checkHandles[] =
    {
        &taskHandleLCD,
        &taskHandleSLAVE_CLK,
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

void esp_task_wdt_isr_user_handler(void)
{
    esp_restart();
}

//---------------------------------------------------------------------------
// Exported
//---------------------------------------------------------------------------

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
        LOG("Invalid destination task %d", dst);
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
        LOG("Invalid destination task %d", msg->dst);
    }
    else if (xQueueSend(handle, (void *)msg, QUEUE_MAX_BLOCK_MS) != pdTRUE)
    {
        LOG("Queue send failed, dst: %u, cmd: %u", msg->dst, msg->cmd);
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

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    gpio_num_t pinNumber = *((gpio_num_t*)args);
    if (pinNumber == USR_BUTTON_IO)
    {
        btn_handler(false);
    }
    else if (pinNumber == POWER_GOOD_IO)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR( taskHandlePWR, 0, eSetBits, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void handle_power_bad(void)
{
    int power_bad_count = 0, power_good_count = 0;
    while(1)
    {
        if (gpio_get_level(POWER_GOOD_IO) != POWER_GOOD_LVL) // periodically check the power good pin
        {
            power_good_count = 0; // reset good counter immediately
            if (power_bad_count < MIN_PWR_BAD_CNT)
            {
                power_bad_count++;
                if (power_bad_count == MIN_PWR_BAD_CNT) // only do it once
                {
                    LOG("Power bad, shutting down tasks");
                    
                    // Tasks which are a bit more delicate, let them finish what they are doing right now
                    task_msg_t msg = {.cmd = TASK_CMD_SHUTDOWN, .dst = TASK_SLAVE_CLK };
                    sendTaskMessage(&msg);
                    msg.dst = TASK_LCD;
                    sendTaskMessage(&msg);

                    wait_shutdown();
                    LOG("Shutdown complete, storing..");

                    ram_mirror.pwr_bad = true; // store power bad flag for later debugging

                    // it's now OK to save the system state
                    store_ram_mirror();
                }
            }
        }
        else if (power_bad_count) // power was bad, we could be recovering
        {
            if (power_good_count < MIN_PWR_GOOD_CNT)
            {
                power_good_count++;
                if (power_good_count == MIN_PWR_GOOD_CNT) // only do it once
                {
                    LOG("Power recovered, resuming tasks");

                    power_bad_count = 0;
                    power_good_count = 0;

                    // resume all tasks
                    vTaskResume(taskHandleLCD);
                    vTaskResume(taskHandleSLAVE_CLK);

                    ram_mirror.pwr_bad = false; // reset flag, recovery OK
                    store_ram_mirror();

                    break;
                }
            }
        } // else: do nothing when no power bad detected
        vTaskDelay(1); // wait some time
    }
}


void PWR_Task(void *parameter)
{
    uint32_t ulNotifiedValue;
    while(1)
    {

         xTaskNotifyWait( 0x00,             /* Don't clear any notification bits on entry. */
                         ULONG_MAX,         /* Reset the notification value to 0 on exit. */
                         &ulNotifiedValue,  /* Notified value pass out in ulNotifiedValue. */
                         portMAX_DELAY      /* Block indefinitely. */
        );
        handle_power_bad();
    }
}

void LOGGING_Task(void *parameter)
{
    task_msg_t msg;
    while(1)
    {
        // wait for new logs
        if (receiveTaskMessage(TASK_LOGGING, portMAX_DELAY, &msg) == false)
            continue;

        // reject any messages not related to logging
        if (msg.cmd != TASK_CMD_LOG)
            continue;

        // print out data, can be blocking since this task is low prio
        uart_write_bytes(UART_NUM_0, msg.log_ptr, msg.log_len);
        vPortFree(msg.log_ptr); // free pointer afterwards
    }
}

static void await_and_handle_testcodes(void)
{
#if USE_TESTCODE == 1
    int received_bytes = 0;
    char command_buf[MAX_COMMAND_LENGTH + 1 /*NULL*/];
    char buf;
    int res;

    LOG(
        "ONLY FOR TESTING!!!\n"
        "\tWaiting for input of firmware test codes, to simulate errors.\n"
        "\tSend the following commands in the quotes to accomplish a function:\n"
        "\t\t'TEST:0\\t' -> lockup software to simulate watchdog triggering\n"
        "\t\t'TEST:1\\t' -> simulate time jumping by +1 hour\n"
        "\t\t'TEST:2\\t' -> simulate time jumping by -1 hour\n"
    );

    while(1)
    {
        // wait for incoming data
        res = uart_read_bytes(LOGGING_UART_PORT, &buf, sizeof(buf), 1000);
        if (res <= 0)
        { // timeout or error
            continue;
        }
        if (received_bytes >= MAX_COMMAND_LENGTH) // sanity check: do not access beyond buffer boundaries
        {
            received_bytes = 0;
            continue;
        }
        command_buf[received_bytes] = buf; // copy character
        received_bytes++; // advance to next position

        if (buf == '\t') // check for terminator
        {
            command_buf[received_bytes] = '\0'; // properly terminate
            LOG("Received command: '%s'", command_buf);

            int cmd_num = 0;
            res = sscanf(command_buf, "TEST:%d", &cmd_num); // parse it
            if (res != 1)
            {
                // does not match, ignore
            }
            else if (cmd_num == TESTCODE_SW_LOCKUP)
            {
                while(1){}; // software lockup
            }
            else if (cmd_num == TESTCODE_INC_HOUR)
            {
                // prepare message
                task_msg_t msg = {
                    .dst = TASK_SLAVE_CLK,
                    .cmd = TASK_CMD_SIMULATE_SECOND_TICK,
                    .utc_time = ram_mirror.last_connected_utc + 3600
                };
                sendTaskMessage(&msg);
            }
            else if (cmd_num == TESTCODE_DEC_HOUR)
            {
                // prepare message
                task_msg_t msg = { .dst = TASK_SLAVE_CLK,
                    .cmd = TASK_CMD_SIMULATE_SECOND_TICK,
                    .utc_time = ram_mirror.last_connected_utc - 3600
                };
                sendTaskMessage(&msg);
            }
            received_bytes = 0; // reset counter to start fresh again
        } // else: await more data
    }
#endif // USE_TESTCODE == 1
}

void app_main(void)
{
    static gpio_num_t pwr_good_io = POWER_GOOD_IO;
    static gpio_num_t usr_btn_io = USR_BUTTON_IO;
    esp_reset_reason_t reason = esp_reset_reason();

    init_serial_print();

    // Setup power good IO as external interrupt
    gpio_set_direction(POWER_GOOD_IO, GPIO_MODE_INPUT);
    gpio_set_intr_type(POWER_GOOD_IO, GPIO_INTR_NEGEDGE);

    // Setup button IO as external interrupt
    gpio_set_direction(USR_BUTTON_IO, GPIO_MODE_INPUT);
    gpio_set_intr_type(USR_BUTTON_IO, GPIO_INTR_NEGEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(USR_BUTTON_IO, gpio_interrupt_handler, (void*)&usr_btn_io);
    gpio_isr_handler_add(POWER_GOOD_IO, gpio_interrupt_handler, (void*)&pwr_good_io);

    esp_pm_config_t pm_config = {
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm_config);

    LOG("\nStarting application. Reset reason: %d\n", reason);

    // If the reset reason is not a power cycle, it's likely due to some SW issue
    bool soft_reset = reason != ESP_RST_UNKNOWN && reason != ESP_RST_POWERON;
    esp_err_t err= inital_nvs_load(soft_reset);
    if (err != ESP_OK)
    {
        LOG("Error (%s) while handling NVS!", esp_err_to_name(err));
    }

    SETUP_QUEUE(SLAVE_CLK, QUEUE_LEN_GENERAL);
    SETUP_QUEUE(TIMER, QUEUE_LEN_GENERAL);
    SETUP_QUEUE(LCD, QUEUE_LEN_GENERAL);
    SETUP_QUEUE(LOGGING, QUEUE_LEN_LOGGING);

    taskHandleNEO6M     = CREATE_TASK_STATIC(NEO6M);
    taskHandleSLAVE_CLK = CREATE_TASK_STATIC(SLAVE_CLK);
    taskHandleTIMER     = CREATE_TASK_STATIC(TIMER);
    taskHandleLCD       = CREATE_TASK_STATIC(LCD);
    taskHandlePWR       = CREATE_TASK_STATIC(PWR);
    taskHandleLOGGING   = CREATE_TASK_STATIC(LOGGING);

    await_and_handle_testcodes();
}