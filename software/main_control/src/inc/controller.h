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
#include "vedio_executor.h"
#include "roboot_executor.h"
#include "audio_receiver.h"
#include "vedio_receiver.h"

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
void execute_commands(char *commands);

/**
 * notify modules to execute the commands.
 * car_module: the module of car, such as motor, music, led
 * command: the command
 *
 */
void notify_all(enum module_def module, char command);

#endif