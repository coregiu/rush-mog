/**
  ******************************************************************************
  *
  * Define of timer manager
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef __PWM_H
#define __PWM_H
#include "command.h"
#include "controller.h"

#define MAX_PWM_RATE 5
#define MIN_PWM_RATE 1
#define STOP_PWM 0
#define NO_PWM 99
#define DEFAULT_TUNE_PWM_STEP 2
#define DEFAULT_PWM_RATE 3


// the main configuration of car.
struct motor_config
{
    uchar pwm_period_times; // 占空比周期次数
    uchar pwm_change_step;  // pwm每次调整步长
};

extern struct motor_config g_motor_config;

extern const struct module_command_executor motor_pwm_executor;

#endif
