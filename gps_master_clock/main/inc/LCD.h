#ifndef _LCD_H_
#define _LCD_H_

#include <stdbool.h>

void LCD_Task(void *parameter);
void btn_handler(bool timer_triggered);

#endif // _LCD_H_
