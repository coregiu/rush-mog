/**
  ******************************************************************************
  *
  * Implement of video_executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/
#include <video_executor.h>
void init_video_state()
{

}

void update_video_state(struct command_context *command_context)
{
    switch (command_context->command)
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

const struct module_command_executor video_executor = {init_video_state, update_video_state};