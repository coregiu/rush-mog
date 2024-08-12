/**
  ******************************************************************************
  *
  * Define of led display executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef _LED_DISPLAY_H
#define _LED_DISPLAY_H

#include "command.h"
#include "oled.h"

extern const struct module_command_executor led_display_executor;

void show_gimbal_info(struct gimbal_info *gimbal_info);

#endif