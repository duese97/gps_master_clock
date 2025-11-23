#ifndef _LCD_H_
#define _LCD_H_

#include <stdbool.h>

#define GRAM_BACKSLASH_IDX      1
#define GRAM_BACK_ICON_IDX      2

#define BACK_ICO_STR            "\x02"

void LCD_Task(void *parameter);
void btn_handler(bool timer_triggered);

#endif // _LCD_H_
