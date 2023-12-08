#ifndef __TIMER_H
#define __TIMER_H
#include "command.h"

void TIM1_Configuration();

void update_timer_state(char command);

extern const struct module_command_executor timer_executor;

#endif
