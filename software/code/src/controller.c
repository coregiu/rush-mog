/**
  ******************************************************************************
  *
  * main controller
  * author: coregiu
  * 
  * 
  ******************************************************************************
**/

#include <controller.h>

/**
 * init uart and all receive modules
 * 
 */
void init_modules()
{
    motor_driver.init();
    led_group.init();
    music_switch.init();
    servos_driver.init();
}

/**
 * receive ps2 data, convert to commands,
 * and notify modules to execute commands;
 * 
 */
void execute_commands(uint is_has_command, int **commands)
{
    if (is_has_command)
    {
        for (uchar i = 0; i < COMMANDS_LENGTH; i++)
        {
            notify_all(commands[i][1], commands[i][0]);
        }
    }
    else
    {
        notify_all(MODULE_MOTOR, COMMAND_LEFT_2); // stop the car
        notify_all(MODULE_SERVO, COMMAND_LEFT_2); // set servo to receive new command
    }
}

/**
 * notify modules to execute the commands.
 * car_module: the module of car, such as motor, music, led
 * car_cmd: the command
 * 
 */
void notify_all(enum module car_module, uint car_cmd)
{
    switch (car_module)
    {
    case MODULE_MOTOR:
        motor_driver.update_state(car_cmd);
        break;
    case MODULE_LED:
        led_group.update_state(car_cmd);
        break;
    case MODULE_MUSIC:
        music_switch.update_state(car_cmd);
        break;
    case MODULE_SERVO:
        servos_driver.update_state(car_cmd);
        break;
    
    default:
        break;
    }
}