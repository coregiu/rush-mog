#include <motor_pwm_executor.h>

void init_timer3(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 开时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // 2. PB0 PB1 配置为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. TIM3 基础配置
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 4. CH3 配置（PB0）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 5. CH4 配置（PB1）
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 6. 使能TIM3
    TIM_Cmd(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
}

// 设置 PB0 占空比 500 -> 50% 占空比，1000 -> 100% 占空比
void left_motor_set_pwm(u16 compare)
{
    TIM_SetCompare3(TIM3, compare);
}

// 设置 PB1 占空比 500 -> 50% 占空比，1000 -> 100% 占空比
void right_motor_set_pwm(u16 compare)
{
    TIM_SetCompare4(TIM3, compare);
}

// 当前系统默认占空比次数
const uchar DEFAULT_TUNE_PWM_STEP = 2;

// 每次微调占空比的步长
uchar tune_pwm_step;

// 前一条指令。当前后两条指令相同时不做操作
uchar pre_cmd;

const uchar NO_PWM = 99;

uchar g_left_motor_run_state = DEFAULT_TUNE_PWM_STEP;
uchar g_right_motor_run_state = DEFAULT_TUNE_PWM_STEP;

struct motor_config g_motor_config = {5, 1};
uchar pre_right_cmd;

// 当前系统的占空比，以此占空比来控制电机速度
uchar current_pwm = DEFAULT_TUNE_PWM_STEP;

void init_tune_pwm_step()
{
    init_timer3(999, 71);
    tune_pwm_step = DEFAULT_TUNE_PWM_STEP;
    pre_cmd = COMMAND_STOP;
}

void update_pwm_state(struct command_context *command_context)
{
    if (command_context->command == pre_cmd)
    {
        return;
    }
    pre_cmd = command_context->command;
    switch (command_context->command)
    {

    case COMMAND_FAST:
        current_pwm += g_motor_config.pwm_change_step;
        current_pwm = current_pwm >= g_motor_config.pwm_period_times ? NO_PWM : current_pwm;

        g_left_motor_run_state = current_pwm;
        g_right_motor_run_state = current_pwm;
        break;
    case COMMAND_SLOW:
        current_pwm = current_pwm == NO_PWM ? g_motor_config.pwm_period_times : current_pwm;
        current_pwm -= g_motor_config.pwm_change_step;
        current_pwm = current_pwm <= g_motor_config.pwm_change_step ? g_motor_config.pwm_change_step : current_pwm;

        g_left_motor_run_state = current_pwm;
        g_right_motor_run_state = current_pwm;
        break;
    case COMMAND_LEFT_MICRO:
        g_left_motor_run_state = current_pwm > tune_pwm_step ? (current_pwm == NO_PWM ? g_motor_config.pwm_period_times : current_pwm) - tune_pwm_step : 0;
        g_right_motor_run_state = current_pwm;
        break;
    case COMMAND_RIGHT_MICRO:
        g_left_motor_run_state = current_pwm;
        g_right_motor_run_state = current_pwm > tune_pwm_step ? (current_pwm == NO_PWM ? g_motor_config.pwm_period_times : current_pwm) - tune_pwm_step : 0;
        break;
    case COMMAND_DIRECT:
        g_left_motor_run_state = current_pwm;
        g_right_motor_run_state = current_pwm;
        break;
    default:
        g_left_motor_run_state = current_pwm;
        g_right_motor_run_state = current_pwm;
        pre_cmd = COMMAND_STOP;
        break;
    }
    left_motor_set_pwm(g_left_motor_run_state * (1000 / g_motor_config.pwm_period_times));
    right_motor_set_pwm(g_right_motor_run_state * (1000 / g_motor_config.pwm_period_times));
}

const struct module_command_executor motor_pwm_executor = {init_tune_pwm_step, update_pwm_state};
