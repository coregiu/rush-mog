/**
  ******************************************************************************
  *
  * Define of command, receiver and executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <controller.h>
#include "task_manager.h"
// the map of command to module
const char command_module_map[COMMANDS_LENGTH][2] = {{COMMAND_STOP, MODULE_VEHICLE},
                                                      {COMMAND_RUN, MODULE_VEHICLE},
                                                      {COMMAND_BACK, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_RUN, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_RUN, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_FRONT, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_FRONT, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_BACK, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_BACK, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_TURN, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_TURN, MODULE_VEHICLE},
                                                      {COMMAND_GO_BACK, MODULE_VEHICLE},
                                                      {COMMAND_TURN_OUT, MODULE_VEHICLE},
                                                      {COMMAND_TEST_VEHICLE, MODULE_VEHICLE},
                                                      {COMMAND_TEST_ROBOOT, MODULE_ROBOOT},
                                                      {COMMAND_OPEN_VEDIO, MODULE_VEDIO},
                                                      {COMMAND_CLOSE_VEDIO, MODULE_VEDIO},
                                                      {COMMAND_OPEN_INTELI, MODULE_VEDIO},
                                                      {COMMAND_CLOSE_INTELI, MODULE_VEDIO},
                                                      {COMMAND_PLAYING, MODULE_INTELI},
                                                      {COMMAND_UNKNOWN, MODULE_UNKNOWN}};

/**
 * init uart and all receive and executor modules
 *
 */
void init_modules()
{
    init_command_module();

    audio_receiver.init();
    vehicle_executor.init();
    // timer_executor.init();

    init_freertos();

    arm_roboot_executor.init();
}

/**
 * receive commands from clients.
 */
char* receive_commands()
{
    return audio_receiver.receive_commands();
}

/**
 * notify modules to execute the commands.
 * car_module: the module of car, such as motor, music, led
 * command: the command
 *
 */
void notify_all(enum module_def module, char command, enum command_type type)
{
    struct command_context command_context = {0, DELAY_AFTER_EXE, type, command};
    switch (module)
    {
    case MODULE_VEHICLE:
        vehicle_executor.update_state(&command_context);
        break;
    case MODULE_VEDIO:
        video_executor.update_state(&command_context);
        break;
    case MODULE_ROBOOT:
        arm_roboot_executor.update_state(&command_context);
        break;
    case MODULE_INTELI:
        command_context.command = 'D';
        vehicle_executor.update_state(&command_context);
        break;

    default:

        break;
    }

}

/**
 * execute commands;
 *
 */
void execute_commands(char *commands, enum command_type type)
{
    uint cmd_seq = convert_command_seq(commands[0]);
    if (cmd_seq >= 0 && cmd_seq < COMMANDS_LENGTH)
    {
        LED = ~LED;
        notify_all(command_module_map[cmd_seq][1], commands[0], type);
    }
}