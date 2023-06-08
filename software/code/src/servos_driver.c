/**
  ******************************************************************************
  *
  * the driver of servos
  * author: coregiu
  * 
  * 
  * The command of play: 
  *     RIGHT_LEFT  - turn gimbal left, command 7; 
  *     RIGHT_RIGHT - turn gimbal right, command 8;
  *     RIGHT_TOP - turn gimbal front, command 5; 
  *     RIGHT_DOWN - turn gimbal back, command 6;
  * 
  * The monitor led of this command: P0_4  P0_5 P0_6  P0_7
  * 
  ******************************************************************************
**/

#include <servos_driver.h>

const int SERVO_DOWN_RIGHT_POSITION  = 0;
const int SERVO_DOWN_MIDDLE_POSITION = 95;
const int SERVO_DOWN_LEFT_POSITION   = 176;

const int SERVO_TOP_BACK_POSITION    = 0;
const int SERVO_TOP_MIDDLE_POSITION  = 88;
const int SERVO_TOP_FRONT_POSITION   = 176;

const int SERVO_MOVE_STEP  = 30;

const int SERVO_DOWN_ID  = 12;
const int SERVO_TOP_ID   = 13;

int current_down_servo_position;
int current_top_servo_position;
int current_command_status;

void init_servos_driver()
{
  init_pca9685();
  current_down_servo_position = SERVO_DOWN_MIDDLE_POSITION;
  current_top_servo_position = SERVO_TOP_MIDDLE_POSITION;
  current_command_status = IDLE;
}

void move_down_servo(int steps)
{
  if (current_command_status == HANDLING)
  {
    return;
  }
  current_down_servo_position += steps;
  current_down_servo_position = current_down_servo_position < SERVO_DOWN_RIGHT_POSITION ? SERVO_DOWN_RIGHT_POSITION : current_down_servo_position;
  current_down_servo_position = current_down_servo_position > SERVO_DOWN_LEFT_POSITION ? SERVO_DOWN_LEFT_POSITION : current_down_servo_position;
  servo_control(SERVO_DOWN_ID, current_down_servo_position);
  current_command_status = HANDLING;
}

void move_top_servo(int steps)
{
  if (current_command_status == HANDLING)
  {
    return;
  }
  current_top_servo_position += steps;
  current_top_servo_position = current_top_servo_position < SERVO_TOP_BACK_POSITION ? SERVO_TOP_BACK_POSITION : current_top_servo_position;
  current_top_servo_position = current_top_servo_position > SERVO_TOP_FRONT_POSITION ? SERVO_TOP_FRONT_POSITION : current_top_servo_position;
  servo_control(SERVO_TOP_ID, current_top_servo_position);
  current_command_status = HANDLING;
}

void restore_servos()
{
  if (current_command_status == HANDLING)
  {
    return;
  }
  servo_control(SERVO_DOWN_ID, SERVO_DOWN_MIDDLE_POSITION);
  servo_control(SERVO_TOP_ID, SERVO_TOP_MIDDLE_POSITION);
  current_command_status = HANDLING;
}

void update_servos_driver(uint car_cmd)
{
  switch (car_cmd)
        {
        case COMMAND_RIGHT_TOP:
            // uart_log_string_data("e:5"); // send 5
            LED_RIGHT_TOP = !LED_RIGHT_TOP;
            move_top_servo(SERVO_MOVE_STEP);
            break;
        case COMMAND_RIGHT_DOWN:
            // uart_log_string_data("e:6"); // send 6
            LED_RIGHT_DOWN = !LED_RIGHT_DOWN;
            move_top_servo(-SERVO_MOVE_STEP);
            break;
        case COMMAND_RIGHT_LEFT:
            // uart_log_string_data("e:7"); // send 7
            LED_RIGHT_LEFT = !LED_RIGHT_LEFT;
            move_down_servo(SERVO_MOVE_STEP);
            break;
        case COMMAND_RIGHT_RIGHT:
            // uart_log_string_data("e:8"); // send 8
            LED_RIGHT_RIGHT = !LED_RIGHT_RIGHT;
            move_down_servo(-SERVO_MOVE_STEP);
            break;
        case COMMAND_LEFT_1:
            // uart_log_string_data("e:3"); // send 3
            LED_RIGHT_TOP = !LED_RIGHT_TOP;
            restore_servos();
            break;
        case COMMAND_LEFT_2:
            // uart_log_string_data("e:4"); // send 4
            // LED_RIGHT_DOWN = !LED_RIGHT_DOWN;
            current_command_status = IDLE;
            break;
        default:
            break;
        }
}

const struct module_command_receiver servos_driver = {init_servos_driver, update_servos_driver};