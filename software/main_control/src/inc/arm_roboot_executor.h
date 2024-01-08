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

// ---------------servo define----------------
#define SERVO_RANGE_1_RIGHT   1
#define SERVO_RANGE_1_MIDDLE  90
#define SERVO_RANGE_1_LEFT    179
#define SERVO_RANGE_1_INIT    90

#define SERVO_RANGE_2_BACK    20
#define SERVO_RANGE_2_MIDDLE  90
#define SERVO_RANGE_2_FRONT   160
#define SERVO_RANGE_2_INIT    30

#define SERVO_RANGE_3_BACK    1
#define SERVO_RANGE_3_MIDDLE  90
#define SERVO_RANGE_3_FRONT   179
#define SERVO_RANGE_3_INIT    160

#define SERVO_RANGE_4_BACK    1
#define SERVO_RANGE_4_MIDDLE  90
#define SERVO_RANGE_4_FRONT   179
#define SERVO_RANGE_4_INIT    160

#define SERVO_RANGE_5_HOR     90
#define SERVO_RANGE_5_VER     179
#define SERVO_RANGE_5_INIT    90

#define SERVO_RANGE_6_OPEN    120
#define SERVO_RANGE_6_CLOSE   179
#define SERVO_RANGE_6_INIT    179

extern const struct module_command_executor arm_roboot_executor;

#endif