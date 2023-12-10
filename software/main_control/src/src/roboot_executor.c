/**
  ******************************************************************************
  *
  * Implement of reboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/
#include <roboot_executor.h>
void init_roboot_state()
{

}

void update_roboot_state(char command, enum command_type type)
{
    switch (command)
        {
        case COMMAND_STOP:
            // uart_log_string_data("e:1"); // send 1
            // LED_LEFT_TOP = !LED_LEFT_TOP;
            // exec_car_state_update(MOVE);
            break;

        default:
            break;
        }
}

const struct module_command_executor roboot_executor = {init_roboot_state, update_roboot_state};