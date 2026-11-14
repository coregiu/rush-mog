/**
  ******************************************************************************
  *
  * Implement of vehicle executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/
#include <motor_direct_executor.h>

// the position of gpio in CAR_STATE_LIST array.
enum gpio_position
{
    LEFT_FRONT_1_POSITION  = 0,
    LEFT_FRONT_2_POSITION  = 1,
    LEFT_BACK_1_POSITION   = 2,
    LEFT_BACK_2_POSITION   = 3,
    RIGHT_FRONT_1_POSITION = 4,
    RIGHT_FRONT_2_POSITION = 5,
    RIGHT_BACK_1_POSITION  = 6,
    RIGHT_BACK_2_POSITION  = 7
};

// car sate to set to gpio: {LEFT_EN, LEFT_MV, LEFT_BK, RIGHT_EN, RIGHT_MV, RIGHT_BK}
const char VEHICLE_STATE_LIST[13][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0}, // 0, init/stop
    {1, 0, 1, 0, 1, 0, 1, 0}, // 1, move, all move
    {0, 1, 0, 1, 0, 1, 0, 1}, // 2, back, all back
    {0, 1, 1, 0, 1, 0, 0, 1}, // 3, left
    {1, 0, 0, 1, 0, 1, 1, 0}, // 4, right
    {0, 0, 1, 0, 1, 0, 0, 0}, // 5, left_front
    {1, 0, 0, 0, 0, 0, 1, 0}, // 6, right_front
    {0, 1, 0, 0, 0, 0, 0, 1}, // 7, left_back
    {0, 0, 0, 1, 0, 1, 0, 0}, // 8, right_back
    {0, 0, 0, 0, 1, 0, 1, 0}, // 9, left_turn
    {1, 0, 1, 0, 0, 0, 0, 0}, // 10, right_turn
    {1, 0, 1, 0, 0, 1, 0, 1}, // 11, turn over from left to right
    {0, 1, 0, 1, 1, 0, 1, 0}  // 12, turn over from right to left
};

enum vehicle_state current_car_status = STOP;

/* 悬崖等安全保护：置位时禁止除 STOP/BACK 外的运动状态。 */
static volatile uint8_t s_motion_inhibit = 0;

void motor_direct_executor_set_inhibit(uint8_t inhibit)
{
    s_motion_inhibit = inhibit;
}

void exec_vehicle_state_update(enum vehicle_state run_state)
{
    /* 与悬崖任务的急停互斥：这里保证“检查保护标志 + 写 8 路 IN”是原子的，
     * 避免命令任务和悬崖任务并发时，后写的运动状态覆盖刚写入的 STOP。 */
    taskENTER_CRITICAL();

    if (s_motion_inhibit && run_state != STOP && run_state != BACK)
    {
        run_state = STOP;
    }

    if (current_car_status != run_state)
    {
        IN1 = VEHICLE_STATE_LIST[run_state][LEFT_FRONT_1_POSITION];
        IN2 = VEHICLE_STATE_LIST[run_state][LEFT_FRONT_2_POSITION];
        IN3 = VEHICLE_STATE_LIST[run_state][LEFT_BACK_1_POSITION];
        IN4 = VEHICLE_STATE_LIST[run_state][LEFT_BACK_2_POSITION];
        IN5 = VEHICLE_STATE_LIST[run_state][RIGHT_FRONT_1_POSITION];
        IN6 = VEHICLE_STATE_LIST[run_state][RIGHT_FRONT_2_POSITION];
        IN7 = VEHICLE_STATE_LIST[run_state][RIGHT_BACK_1_POSITION];
        IN8 = VEHICLE_STATE_LIST[run_state][RIGHT_BACK_2_POSITION];

        current_car_status = run_state;
    }

    taskEXIT_CRITICAL();
}

void goback()
{
    switch (current_car_status)
    {
    case MOVE:
        exec_vehicle_state_update(BACK);
        break;
    case BACK:
        exec_vehicle_state_update(MOVE);
        break;
    case LEFT:
        exec_vehicle_state_update(RIGHT);
        break;
    case RIGHT:
        exec_vehicle_state_update(LEFT);
        break;
    case LEFT_FRONT:
        exec_vehicle_state_update(RIGHT_BACK);
        break;
    case RIGHT_BACK:
        exec_vehicle_state_update(LEFT_FRONT);
        break;
    case RIGHT_FRONT:
        exec_vehicle_state_update(LEFT_BACK);
        break;
    case LEFT_BACK:
        exec_vehicle_state_update(RIGHT_FRONT);
        break;
    default:
        break;
    }
}

void put_test_commands()
{
    struct command_context command_context = {0};
    command_context.commands[0] = COMMAND_RUN;
    command_context.cmd_length = 1;
    command_context.module = MODULE_MOTOR_DIRECT;
    command_context.time_sleep_milsec = 1000;
    command_context.delay_type = DELAY_AFTER_EXE;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_BACK;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_LEFT_RUN;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_RIGHT_RUN;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_LEFT_FRONT;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_RIGHT_FRONT;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_LEFT_BACK;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_RIGHT_BACK;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_LEFT_TURN;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_RIGHT_TURN;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_TURN_OUT_L;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_TURN_OUT_R;
    send_to_queue(&command_context);

    command_context.commands[0] = COMMAND_STOP;
    send_to_queue(&command_context);
}

void init_vehicle_state()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); //使能PA,PB, PC端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_IN1;           //GPIO PA11 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);            //根据设定参数初始化GPIOA.IN1
    IN1 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    IN2 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    IN3 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    IN4 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    IN5 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN6;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    IN6 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    IN7 = 0;

    GPIO_InitStructure.GPIO_Pin = GPIO_IN8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    IN8 = 0;
}

void reset_vehicle_state()
{
    exec_vehicle_state_update(STOP);
}

void update_vehicle_state(struct command_context *command_context)
{
    uart_log_data(command_context->commands[0]);
    uart_log_enter_char();
    switch (command_context->commands[0])
    {
    case COMMAND_STOP:
        exec_vehicle_state_update(STOP);
        break;
    case COMMAND_RUN:
        exec_vehicle_state_update(MOVE);
        break;
    case COMMAND_BACK:
        exec_vehicle_state_update(BACK);
        break;
    case COMMAND_LEFT_RUN:
        exec_vehicle_state_update(LEFT);
        break;
    case COMMAND_RIGHT_RUN:
        exec_vehicle_state_update(RIGHT);
        break;
    case COMMAND_LEFT_FRONT:
        exec_vehicle_state_update(LEFT_FRONT);
        break;
    case COMMAND_RIGHT_FRONT:
        exec_vehicle_state_update(RIGHT_FRONT);
        break;
    case COMMAND_LEFT_BACK:
        exec_vehicle_state_update(LEFT_BACK);
        break;
    case COMMAND_RIGHT_BACK:
        exec_vehicle_state_update(RIGHT_BACK);
        break;
    case COMMAND_LEFT_TURN:
        exec_vehicle_state_update(LEFT_TURN);
        break;
    case COMMAND_RIGHT_TURN:
        exec_vehicle_state_update(RIGHT_TURN);
        break;
    case COMMAND_TURN_OUT_L:
        exec_vehicle_state_update(TURN_OVER_L);
        break;
    case COMMAND_TURN_OUT_R:
        exec_vehicle_state_update(TURN_OVER_R);
        break;
    case COMMAND_TEST_VEHICLE:
        put_test_commands();
        break;
    default:
        break;
    }
}

const struct module_command_executor motor_direct_executor = {init_vehicle_state, reset_vehicle_state, update_vehicle_state};
