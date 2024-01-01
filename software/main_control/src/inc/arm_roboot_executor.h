/**
  ******************************************************************************
  *
  * Define of arm roboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef EXECUTOR_ARM_ROBOOT_H
#define EXECUTOR_ARM_ROBOOT_H

#include "command.h"

#define SERVO_1 0
#define SERVO_2 3
#define SERVO_3 4
#define SERVO_4 7
#define SERVO_5 8
#define SERVO_6 11

extern const struct module_command_executor arm_roboot_executor;

#endif