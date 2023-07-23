/**
  ******************************************************************************
  *
  * The driver of motor
  * author: coregiu
  * 
  * MOVE command:
  *     key: LEFT_TOP
  *     monitor led: P0_0
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 1
  *         LEFT_BK = 0
  *         RIGHT_EN = 1
  *         RIGHT_MV = 1
  *         RIGHT_BK = 0
  * 
  * BACK command:
  *     key: LEFT_DOWN
  *     monitor led: P0_1
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 0
  *         LEFT_BK = 1
  *         RIGHT_EN = 1
  *         RIGHT_MV = 0
  *         RIGHT_BK = 1
  * 
  * TURN LEFT command:
  *     key: LEFT_LEFT
  *     monitor led: P0_2
  *     gpio:
  *         LEFT_EN = 0
  *         LEFT_MV = 0
  *         LEFT_BK = 0
  *         RIGHT_EN = 1
  *         RIGHT_MV = 1
  *         RIGHT_BK = 0
  * 
  * TURN RIGHT command:
  *     key: LEFT_RIGHT
  *     monitor led: P0_3
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 1
  *         LEFT_BK = 0
  *         RIGHT_EN = 0
  *         RIGHT_MV = 0
  *         RIGHT_BK = 0
  * 
  * P2_0 : RIGHT_EN  L298N_EnA
  * P2_1 : RIGHT_MV  L298N_IN2
  * P2_2 : RIGHT_BK  L298N_IN1 
  * P2_3 : LEFT_EN   L298N_EnB
  * P2_4 : LEFT_MV   L298N_IN4
  * P2_5 : LEFT_BK   L298N_IN3
  * 
  ******************************************************************************
**/
#include <motor_driver.h>
#include <mcs51/8051.h>

// the position of gpio in CAR_STATE_LIST array.
enum gpio_position
{
    LEFT_EN_POSITION  = 0, 
    LEFT_MV_POSITION  = 1, 
    LEFT_BK_POSITION  = 2, 
    RIGHT_EN_POSITION = 3, 
    RIGHT_MV_POSITION = 4, 
    RIGHT_BK_POSITION = 5
};

// car sate to set to gpio: {LEFT_EN, LEFT_MV, LEFT_BK, RIGHT_EN, RIGHT_MV, RIGHT_BK}
const uchar CAR_STATE_LIST[5][6] = {{0, 0, 0, 0, 0, 0},  // init
                                    {1, 1, 0, 1, 1, 0},  // move
                                    {1, 0, 1, 1, 0, 1},  // back
                                    {0, 0, 0, 1, 1, 0},  // left
                                    {1, 1, 0, 0, 0, 0}}; // right

enum car_run_state current_car_status = STOP;
void stop()
{
    LEFT_EN  = CAR_STATE_LIST[STOP][LEFT_EN_POSITION];
    RIGHT_EN = CAR_STATE_LIST[STOP][RIGHT_EN_POSITION];
    current_car_status = STOP;
}

void exec_car_state_update(enum car_run_state run_state)
{
    if (current_car_status != run_state)
    {
        LEFT_EN  = CAR_STATE_LIST[run_state][LEFT_EN_POSITION];
        LEFT_MV  = CAR_STATE_LIST[run_state][LEFT_MV_POSITION];
        LEFT_BK  = CAR_STATE_LIST[run_state][LEFT_BK_POSITION];
        RIGHT_EN = CAR_STATE_LIST[run_state][RIGHT_EN_POSITION];
        RIGHT_MV = CAR_STATE_LIST[run_state][RIGHT_MV_POSITION];
        RIGHT_BK = CAR_STATE_LIST[run_state][RIGHT_BK_POSITION];
        current_car_status == run_state;
    }
}

void init_motor_switch()
{
    exec_car_state_update(STOP);
}

void update_motor_state(uint car_cmd)
{
    switch (car_cmd)
        {
        case COMMAND_LEFT_TOP:
            // uart_log_string_data("e:1"); // send 1
            LED_LEFT_TOP = !LED_LEFT_TOP;
            exec_car_state_update(MOVE);
            break;
        case COMMAND_LEFT_DOWN:
            // uart_log_string_data("e:2"); // send 2
            LED_LEFT_DOWN = !LED_LEFT_DOWN;
            exec_car_state_update(BACK);
            break;
        case COMMAND_LEFT_LEFT:
            // uart_log_string_data("e:3"); // send 3
            LED_LEFT_LEFT = !LED_LEFT_LEFT;
            exec_car_state_update(LEFT);
            break;
        case COMMAND_LEFT_RIGHT:
            // uart_log_string_data("e:4"); // send 4
            LED_LEFT_RIGHT = !LED_LEFT_RIGHT;
            exec_car_state_update(RIGHT);
            break;
        case COMMAND_LEFT_2:
            // uart_log_string_data("e:4"); // send 4
            // LED_LEFT_DOWN = !LED_LEFT_DOWN;
            stop();
            break;
        default:
            break;
        }
}

const struct module_command_receiver motor_driver = {init_motor_switch, update_motor_state};