#ifndef _LCM1602_H_
#define _LCM1602_H_

#include <inttypes.h>
#include <string.h>
#include "esp_err.h"

// LCD -> PCF8574A Pin Mapping
#define PIN_RS      (1 << 0)
#define PIN_RW      (1 << 1)
#define PIN_EN      (1 << 2)
#define PIN_BL      (1 << 3)
#define PIN_D4      (1 << 4)
#define PIN_D5      (1 << 5)
#define PIN_D6      (1 << 6)
#define PIN_D7      (1 << 7)


/*!
 @defined
 @abstract   All these definitions shouldn't be used unless you are writing
 a driver.
 @discussion All these definitions are for driver implementation only and
 shouldn't be used by applications.
 */
// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x8DOTS 0x00

// Define COMMAND and DATA LCD Rs (used by send method).
// ---------------------------------------------------------------------------
#define COMMAND 0
#define LCD_DATA 1
#define FOUR_BITS 2

/*!
 @definedgpio_num_t rs, gpio_num_t enable, gpio_num_t d0, gpio_num_t d1, gpio_num_t d2, gpio_num_t d3
 @abstract   Defines the duration of the home and clear commands
 @discussion This constant defines the time it takes for the home and clear
 commands in the LCD - Time in microseconds.
 */
#define HOME_CLEAR_EXEC 2000

esp_err_t LCD_I2C_begin(uint8_t cols, uint8_t lines);
esp_err_t LCD_I2C_print(const char* str);

void LCD_I2C_clear();
void LCD_I2C_home();

void LCD_I2C_setCursor(uint8_t col, uint8_t row);

void LCD_I2C_noDisplay();
void LCD_I2C_display();

void LCD_I2C_noCursor();
void LCD_I2C_cursor();

void LCD_I2C_noBlink();
void LCD_I2C_blink();

void LCD_I2C_scrollDisplayLeft(void);
void LCD_I2C_scrollDisplayRight(void);

void LCD_I2C_leftToRight(void);
void LCD_I2C_rightToLeft(void);
void LCD_I2C_moveCursorRight(void);
void LCD_I2C_moveCursorLeft(void);
void LCD_I2C_autoscroll(void);
void LCD_I2C_noAutoscroll(void);

void LCD_I2C_createChar(uint8_t location, const uint8_t charmap[]);
void LCD_I2C_backlight(uint8_t on);

#endif // _LCM1602_H_
