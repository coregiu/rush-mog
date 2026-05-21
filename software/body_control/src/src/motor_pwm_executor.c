#include <motor_pwm_executor.h>




// 每次微调占空比的步长
uchar tune_pwm_step;

uchar g_left_motor_run_state = DEFAULT_PWM_RATE;
uchar g_right_motor_run_state = DEFAULT_PWM_RATE;

struct motor_config g_motor_config = {DEFAULT_PWM_RATE, DEFAULT_TUNE_PWM_STEP};

// 当前系统的占空比，以此占空比来控制电机速度
uchar current_pwm = DEFAULT_PWM_RATE;

void init_timer3()
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
    // 定时器参数配置
    // 定时器时钟 = 72MHz; PWM 频率 = 72000000 / (psc+1) / (arr+1)
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
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
    uart_log_string_data("set left motor pwm: ");
    uart_log_number(compare);
    uart_log_enter_char();
    TIM_SetCompare3(TIM3, compare);
}

// 设置 PB1 占空比 500 -> 50% 占空比，1000 -> 100% 占空比
void right_motor_set_pwm(u16 compare)
{
    uart_log_string_data("set right motor pwm: ");
    uart_log_number(compare);
    uart_log_enter_char();
    TIM_SetCompare4(TIM3, compare);
}

void init_tune_pwm_step()
{
    init_timer3();
    left_motor_set_pwm(g_left_motor_run_state * (1000 / g_motor_config.pwm_period_times));
    right_motor_set_pwm(g_right_motor_run_state * (1000 / g_motor_config.pwm_period_times));
    tune_pwm_step = DEFAULT_TUNE_PWM_STEP;
}

void reset_pwm_state()
{
    g_left_motor_run_state = DEFAULT_PWM_RATE;
    g_right_motor_run_state = DEFAULT_PWM_RATE;
    current_pwm = DEFAULT_PWM_RATE;
    left_motor_set_pwm(g_left_motor_run_state * (1000 / g_motor_config.pwm_period_times));
    right_motor_set_pwm(g_right_motor_run_state * (1000 / g_motor_config.pwm_period_times));
}

void update_pwm_state(struct command_context *command_context)
{
    switch (command_context->commands[0])
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
        current_pwm = command_context->pwm_rate;
        g_left_motor_run_state = command_context->pwm_rate;
        g_right_motor_run_state = command_context->pwm_rate;

        break;
    }
    g_left_motor_run_state = g_left_motor_run_state > g_motor_config.pwm_period_times ? g_motor_config.pwm_period_times : g_left_motor_run_state;
    g_left_motor_run_state = g_left_motor_run_state < g_motor_config.pwm_change_step ? STOP_PWM : g_left_motor_run_state;

    g_right_motor_run_state = g_right_motor_run_state > g_motor_config.pwm_period_times  ? g_motor_config.pwm_period_times : g_right_motor_run_state;
    g_right_motor_run_state = g_right_motor_run_state < g_motor_config.pwm_change_step ? STOP_PWM : g_right_motor_run_state;

    left_motor_set_pwm(g_left_motor_run_state * (1000 / g_motor_config.pwm_period_times));
    right_motor_set_pwm(g_right_motor_run_state * (1000 / g_motor_config.pwm_period_times));
}

const struct module_command_executor motor_pwm_executor = {init_tune_pwm_step, reset_pwm_state, update_pwm_state};
