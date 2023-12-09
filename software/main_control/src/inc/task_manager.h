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

// init tasks
void init_tasks();

// consumer task for command queue
void exe_task_from_queue(void *argument);

#endif
