#ifndef _LCM1602_H_
#define _LCM1602_H_

#include <inttypes.h>
#include <string.h>

#include "esp_err.h" // for the error typedef

//Instruction Set
#define CLEARDISPLAY 0X01
#define CURSORSHIFT 0x10

//display entry mode
#define ENTRYMODESET 0X04
#define ENTRYLEFT 0X02
#define ENTRYRIGHT 0X00
#define ENTRYSHIFTINCREMENT 0X01
#define ENTRYSHIFTDECREMENT 0X00

// flags for display/cursor shift
#define DISPLAYMOVE 0x08
#define CURSORMOVE 0x00
#define MOVERIGHT 0x04
#define MOVELEFT 0x00

//display control
#define DISPLAYCONTROL 0X08
#define DISPLAYON 0X04
#define DISPLAYOFF 0X00
#define CURSORON 0X02
#define CURSOROFF 0X00
#define BLINKON 0X01
#define BLINKOFF 0X00
#define SETCGRAMADDR 0x40 
#define SETDDRAMADDR 0x80

//for functionset

#define FUNCTIONSET 0x20
#define _5x10DOTS 0x04
#define _5x8DOTS 0x00
#define _1LINE 0x00
#define _2LINE 0x08
#define _8BITMODE 0x10
#define _4BITMODE 0x00

esp_err_t lcm1602_init(void);
void lcm1602_display();
void lcm1602_noDisplay();
void lcm1602_clear();
void lcm1602_setCursor(uint8_t col, uint8_t row);
void lcm1602_scrollDisplayRight(void);
void lcm1602_scrollDisplayLeft(void);
void lcm1602_print(const char c[]);

#endif // _LCM1602_H_
