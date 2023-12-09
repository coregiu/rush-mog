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

// define the single light
#define LED PCout(13)	//PC13 0-light up; 1-turn off.

#define COMMANDS_LENGTH 20

#define DEFAULT_BOUND_RATE 9600

#define MAX_COMMAND_QUEUE_SIZE 1000


// 命令处理队列，用于任务间通信
extern QueueHandle_t command_queue;

// 放入队列的元素。一个是命令，一个是命令执行后挂起时长。
struct command_des
{
    uint16_t time_sleep_milsec;
    char command;
};

// define commands id
enum commands_def
{
    COMMAND_STOP         = '0',
    COMMAND_RUN          = '1',
    COMMAND_BACK         = '2',
    COMMAND_LEFT_RUN     = '3',
    COMMAND_RIGHT_RUN    = '4',
    COMMAND_LEFT_FRONT   = '5',
    COMMAND_RIGHT_FRONT  = '6',
    COMMAND_LEFT_BACK    = '7',
    COMMAND_RIGHT_BACK   = '8',
    COMMAND_LEFT_TURN    = '9',
    COMMAND_RIGHT_TURN   = 'A',
    COMMAND_TURN_OUT     = 'B',
    COMMAND_GO_BACK      = 'C',
    COMMAND_TEST_VEHICLE = 'D',
    COMMAND_TEST_ROBOOT  = 'E',
    COMMAND_OPEN_VEDIO   = 'F',
    COMMAND_CLOSE_VEDIO  = 'G',
    COMMAND_OPEN_INTELI  = 'H',
    COMMAND_CLOSE_INTELI = 'I',
    COMMAND_UNKNOWN      = 'Z'
};

// define module id
enum module_def
{
    MODULE_VEHICLE   = '0',
    MODULE_VEDIO     = '1',
    MODULE_ROBOOT    = '2',
    MODULE_UNKNOWN   = '9'
};



// define command receiver such as audio receiver and vedio receiver
struct module_command_receiver
{
	void (*init)();
	char* (*receive_commands)();
};

// define command executor such as vehicle, roboot
struct module_command_executor
{
	void (*init)();
	void (*update_state)(char command);
};

// init command led
void init_command_led();

// init operation system
void init_freertos();

// convert command to the array sequence of command_module_map.
uint convert_command_seq(char command);
#endif