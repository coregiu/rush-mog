/**
  ******************************************************************************
  *
  * command define and command receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef CONTROLLER_DEFINE_H
#define CONTROLLER_DEFINE_H

#include "command.h"
#include "vehicle_executor.h"
#include "video_executor.h"
#include "timer_executor.h"
#include "audio_receiver.h"
#include "video_receiver.h"
#include "arm_roboot_executor.h"

extern const char command_module_map[COMMANDS_LENGTH][2];

/**
 * init uart and all receive and executor modules
 *
 */
void init_modules();

/**
 * receive commands from clients.
 */
char* receive_commands();

/**
 * execute commands;
 *
 */
void execute_commands(char *commands, enum command_type type);

#endif