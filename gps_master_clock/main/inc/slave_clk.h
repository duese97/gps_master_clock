#ifndef _SLAVE_CLK_H_
#define _SLAVE_CLK_H_

#include <time.h> // for tm struct
#include <stdbool.h>

// exported vars
void SLAVE_CLK_Task(void *parameter);
void take_tz_mutex(void);
void give_tz_mutex(void);

#endif // _SLAVE_CLK_H_
