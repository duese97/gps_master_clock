#ifndef _BSP_H_
#define _BSP_H_

#include <stdint.h>
#include "driver/gpio.h"

#define GPIO_LED                GPIO_NUM_2
#define LED_ON_LVL              1
#define LED_OFF_LVL             0

// custom pin mapping for I2C
#define I2C_SCL_IO				GPIO_NUM_22               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO				GPIO_NUM_21               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ				100000           /*!< I2C master clock frequency */
#define I2C_PORT_NUM			I2C_NUM_0        /*!< I2C port number for master dev */
#define MAX_WAIT_TICKS          10 // should usually only take 1 ms or less

#define LCM1602_ADDR            0x27

#define NEO6M_UART              UART_NUM_2
#define NEO6M_RX_PIN            GPIO_NUM_17 // Where the data from NEO6M will be received (modules TX pin)
#define NEO6M_TX_PIN            GPIO_NUM_16 // Where the data to NEO6M will be transmitted (modules RX pin)

#define POWER_GOOD_IO           GPIO_NUM_35
#define POWER_GOOD_LVL          1

#define USR_BUTTON_IO           GPIO_NUM_34
#define USR_BUTTON_PRESS_LVL    1

#define LOGGING_UART_PORT       UART_NUM_0
#define LOGGING_UART_TX         GPIO_NUM_1
#define LOGGING_UART_RX         GPIO_NUM_3

#define H_BRIDGE1_A             GPIO_NUM_26
#define H_BRIDGE1_B             GPIO_NUM_27

#define H_BRIDGE2_C             GPIO_NUM_25
#define H_BRIDGE2_D             GPIO_NUM_33

#endif // _BSP_H_
