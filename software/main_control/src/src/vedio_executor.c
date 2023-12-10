/**
  ******************************************************************************
  *
  * Implement of vedio executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/
#include <vedio_executor.h>
void init_vedio_state()
{

}

void update_vedio_state(char command, enum command_type type)
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

const struct module_command_executor vedio_executor = {init_vedio_state, update_vedio_state};