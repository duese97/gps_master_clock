#ifndef _MENU_HANDLER_H_
#define _MENU_HANDLER_H_

#include "custom_main.h"

bool menu_statemachine(btn_state_t btn_state, bool* comm_changed);
void menu_update(void);

#endif // _MENU_HANDLER_H_
