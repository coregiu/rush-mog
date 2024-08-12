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

// define command type
enum command_type
{
    COMMAND_TYPE_MANUAL  = '0',
    COMMAND_TYPE_AUTO    = '1'
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
    COMMAND_TEST_ROBOOT    = 'E',
    COMMAND_OPEN_VEDIO     = 'F',
    COMMAND_CLOSE_VEDIO    = 'G',
    COMMAND_OPEN_INTELI    = 'H',
    COMMAND_CLOSE_INTELI   = 'I',
    COMMAND_PLAYING        = 'J',
    COMMAND_ADAPTE_SERVO   = 'K',
    COMMAND_ATTITUDE_INFO  = 'L',
    COMMAND_LED_DISPLAY    = 'M',
    COMMAND_UNKNOWN        = 'Z'
};

// define module id
enum module_def
{
    MODULE_VEHICLE   = '0',
    MODULE_VEDIO     = '1',
    MODULE_ROBOOT    = '2',
    MODULE_INTELI    = '3',
    MODULE_ATTITUDE  = '4',
    MODULE_LED       = '5',
    MODULE_UNKNOWN   = '9'
};

// 放入队列的元素。一个是命令，一个是命令执行后挂起时长。
struct command_context
{
    char command;
    enum module_def module;
    uint16_t time_sleep_milsec;
    enum delay_type delay_type;
    enum command_type command_type;
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

void send_to_queue(struct command_context *command);

#endif