#include "timer_executor.h"

void tim1_configuration()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // 使能TIM1时钟

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 时钟预分频值
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // 时钟分频因子
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 重复计数器

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // TIM1溢出中断允许

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; // TIM1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // 子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // TIM1中断允许
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE); // TIM1使能
}

void update_timer_state(struct command_context *command_context)
{

}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // TIM1溢出中断
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除TIM1溢出中断标志位
        // TODO: 在此处添加中断处理代码
        uart_log_data('T');
    }
}

const struct module_command_executor timer_executor = {tim1_configuration, update_timer_state};