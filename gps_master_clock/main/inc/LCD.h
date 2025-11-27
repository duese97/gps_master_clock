#ifndef _LCD_H_
#define _LCD_H_

#include <stdbool.h>

// custom characters
enum
{
    // avoid starting at 0 (which is technically available). this would
    // cause issues with string related functions, where the custom
    // characters are used
    GRAM_BACKSLASH_IDX = 1,
    GRAM_BACK_ICON_IDX,
    GRAM_DISH_ICON_IDX
};

// special chars outside of normal ASCII range
#define SUM_ICON_CHAR           0xF6

// helper for easier integration into LCD strings
#define BACK_ICO_STR            "\x02"
#define DISH_ICO_STR            "\x03"
#define SUM_ICO_STR             "\xF6"

// exported functions
void LCD_Task(void *parameter);
void btn_handler(bool timer_triggered);

#endif // _LCD_H_
