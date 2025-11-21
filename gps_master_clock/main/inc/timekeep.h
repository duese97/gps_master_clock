#ifndef _TIMEKEEP_H_
#define _TIMEKEEP_H_

#include <time.h> // for tm struct
#include <stdbool.h>

// exported vars
void TIMEKEEP_Task(void *parameter);
void take_tz_mutex(void);
void give_tz_mutex(void);

#endif // _TIMEKEEP_H_
