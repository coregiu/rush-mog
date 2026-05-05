/**
  ******************************************************************************
  *
  * Define of command, receiver and executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef COMMAND_DEFINE_H
#define COMMAND_DEFINE_H

#include "stm32f10x_conf.h"
#include "delay.h"
#include "sys.h"
#include "uart_log.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "mpu6050.h"
#include "tof.h"

// define the single light
#define LED PCout(13)	//PC13 0-light up; 1-turn off.

#define COMMANDS_LENGTH 24

#define DEFAULT_BUFFER_SIZE 8

#define DEFAULT_BOUND_RATE 9600

#define MAX_COMMAND_QUEUE_SIZE 1000


// 命令处理队列，用于任务间通信
extern QueueHandle_t command_queue;

extern struct gimbal_info gimbal_info;

enum delay_type
{
    DELAY_BEFOR_EXE = '0',
    DELAY_AFTER_EXE = '1'
};

// define commands id
enum commands_def
{
    COMMAND_STOP           = '0',
    COMMAND_RUN            = '1',
    COMMAND_BACK           = '2',
    COMMAND_LEFT_RUN       = '3',
    COMMAND_RIGHT_RUN      = '4',
    COMMAND_LEFT_FRONT     = '5',
    COMMAND_RIGHT_FRONT    = '6',
    COMMAND_LEFT_BACK      = '7',
    COMMAND_RIGHT_BACK     = '8',
    COMMAND_LEFT_TURN      = '9',
    COMMAND_RIGHT_TURN     = 'A',
    COMMAND_TURN_OUT       = 'B',
    COMMAND_GO_BACK        = 'C',
    COMMAND_TEST_VEHICLE   = 'D',
    COMMAND_FAST           = 'E',
    COMMAND_SLOW           = 'F',
    COMMAND_LEFT_MICRO     = 'G',
    COMMAND_RIGHT_MICRO    = 'H',
    COMMAND_DIRECT         = 'I',
    COMMAND_UNKNOWN        = 'Z'
};

// define module id
enum module_def
{
    MODULE_MOTOR_DIRECT  = '0',
    MODULE_MOTOR_PWM     = '1',
    MODULE_ARB_BOT       = '2',
    MODULE_UNKNOWN       = '9'
};

// 放入队列的元素。一个是命令，一个是命令执行后挂起时长。
struct command_context
{
    uchar *commands; // 原始命令
    uchar exe_cmd;
    uchar cmd_length;
    enum module_def module;
    uint16_t time_sleep_milsec;
    enum delay_type delay_type;
    uchar pwm_rate; // PWM占空比，0-5, 5表示100%占空比
};

// define command receiver such as audio receiver and video receiver
struct module_command_receiver
{
	void (*init)();
	char* (*receive_commands)();
};

// define command executor such as vehicle, roboot
struct module_command_executor
{
	void (*init)();
	void (*update_state)(struct command_context *command_context);
};

// init command
void init_command_led_module();

// convert command to the array sequence of command_module_map.
uint convert_command_seq(char command);

// convert pwm_rate to the uint.
uint convert_pwm_rate(char pwm_rate);

void send_to_queue(struct command_context *command);

#endif