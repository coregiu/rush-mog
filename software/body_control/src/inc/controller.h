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
#include "motor_direct_executor.h"
#include "motor_pwm_executor.h"
#include "audio_receiver.h"
#include "ps2_receiver.h"
#include "arm_roboot_executor.h"

extern const char command_module_map[COMMANDS_LENGTH][2];

/**
 * init clock to 72MHz.
 */
void clock_init(void);

/**
 * init protocols such as iic, uart.
 */
void init_protocols();

/**
 * init all receive and executor modules
 *
 */
void init_modules();

/**
 * notify modules to execute the commands.
 * car_module: the module of car, such as motor, music, led
 * command: the command
 *
 */
void execute_command(struct command_context *command_context);


#endif