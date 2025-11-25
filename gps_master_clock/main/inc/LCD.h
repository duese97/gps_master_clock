#ifndef _LCD_H_
#define _LCD_H_

#include <stdbool.h>

// custom characters
#define GRAM_BACKSLASH_IDX      1
#define GRAM_BACK_ICON_IDX      2

// special chars outside of normal ASCII range
#define SUM_ICON_CHAR           0xF6

// helper for easier integration into LCD strings
#define BACK_ICO_STR            "\x02"
#define SUM_ICO_STR             "\xF6"

// exported functions
void LCD_Task(void *parameter);
void btn_handler(bool timer_triggered);

#endif // _LCD_H_
