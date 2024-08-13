#include <timer_manager.h>

struct command_context attitude_cmd = {COMMAND_ATTITUDE_INFO, MODULE_ATTITUDE, 0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO};
struct command_context display_cmd = {COMMAND_LED_DISPLAY, MODULE_LED, 0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO};

// static void vTimerCallback(TimerHandle_t xTimer);
// static TimerHandle_t xMyTimer = NULL;

SemaphoreHandle_t xTim2Semaphore;

void create_timer_manager()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 定时器参数配置
    // 假设系统时钟为72MHz，TIM2时钟源为APB1（经过2分频后为36MHz）
    // 设置预分频值为35999，则TIM2的计数频率为36MHz/(35999+1)=1KHz
    // 设置自动重载值为999，则中断周期为(999+1)*1ms=1000ms
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2的更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 中断优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; /* 抢占优先级3 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        /* 子优先级0 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           /* IRQ通道使能 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);           /* 设置优先级分组为4组，即抢占优先级范围为0-15 */
    NVIC_Init(&NVIC_InitStructure);                           /* 根据指定的参数初始化VIC寄存器 */

    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

// static void vTimerCallback(TimerHandle_t xTimer)
// {
//     // LED = ~LED;
//     execute_command(&attitude_cmd);
//     execute_command(&display_cmd);
// }

// void create_timer_manager()
// {
//     const char *pcTimerName = "GimbalTimer";
//     uint32_t ulPeriod = pdMS_TO_TICKS(100); // Period in ticks, e.g., 1000ms
//     uint8_t ucAutoReload = pdTRUE;           // Auto-reload after expiry
//     BaseType_t xTimerCreated = pdFALSE;

//     xMyTimer = xTimerCreate(
//         pcTimerName,      // A text name for the timer (not used by the kernel)
//         ulPeriod,         // The timer period in ticks
//         ucAutoReload,     // Auto-reload setting
//         (void *)NULL,     // The timer ID, can be NULL
//         vTimerCallback    // The callback function that will be called on timer expiry
//     );

//     if(xMyTimer != NULL)
//     {
//         xTimerStart(xMyTimer, 0); // Start the timer
//         xTimerCreated = pdTRUE;
//     }

//     configASSERT(xTimerCreated);
// }

void vTimerTask(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(xTim2Semaphore, portMAX_DELAY))
        {
            LED = ~LED;
            execute_command(&attitude_cmd);
            execute_command(&display_cmd);
        }
    }
}

void init_timer_module()
{
    create_timer_manager();
    // xTim2Semaphore = xSemaphoreCreateBinary();
    // xTaskCreate(vTimerTask,
    //             "GimbalTask",
    //             configMINIMAL_STACK_SIZE,
    //             NULL,
    //             tskIDLE_PRIORITY + 1,
    //             NULL);
}


void update_timer_state(struct command_context *command_context)
{

}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        // 清除更新中断标志位
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // xSemaphoreGiveFromISR(xTim2Semaphore, NULL);
        execute_command(&attitude_cmd);
        execute_command(&display_cmd);
    }
}

const struct module_command_executor timer_manager = {init_timer_module, update_timer_state};
