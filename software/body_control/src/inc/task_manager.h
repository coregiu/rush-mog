/**
  ******************************************************************************
  *
  * Define of task manager.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "command.h"

// init freertos, such as command queue and tasks.
void init_freertos();

// reset command queue, such as clear all commands in the queue.
void reset_command_queue();

// consumer task for command queue
void exe_task_from_queue(void *argument);

#endif
