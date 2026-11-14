/**
  ******************************************************************************
  * TCRT5000 悬崖监测：PB3 左前、PB4 右前，触发后立即封锁前进并停车。
  ******************************************************************************
**/

#include <cliff_sensor.h>
#include "motor_direct_executor.h"

#define CLIFF_LEFT_GPIO  GPIOB
#define CLIFF_LEFT_PIN   GPIO_Pin_3
#define CLIFF_RIGHT_GPIO GPIOB
#define CLIFF_RIGHT_PIN  GPIO_Pin_4

/* 当前稳定状态（CLIFF_LEFT_MASK / CLIFF_RIGHT_MASK 的按位或）。 */
static volatile uint8_t s_stable_mask;

/* 消抖用的候选状态和连续采样次数。 */
static uint8_t s_candidate_mask;
static uint8_t s_candidate_samples;

static uint8_t cliff_read_raw_mask(void)
{
    uint8_t mask = 0;

    if (GPIO_ReadInputDataBit(CLIFF_LEFT_GPIO, CLIFF_LEFT_PIN) == CLIFF_LEFT_TRIGGER_LEVEL)
    {
        mask |= CLIFF_LEFT_MASK;
    }
    if (GPIO_ReadInputDataBit(CLIFF_RIGHT_GPIO, CLIFF_RIGHT_PIN) == CLIFF_RIGHT_TRIGGER_LEVEL)
    {
        mask |= CLIFF_RIGHT_MASK;
    }

    return mask;
}

static void cliff_sensor_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint8_t raw = cliff_read_raw_mask();

        if (raw != s_candidate_mask)
        {
            s_candidate_mask = raw;
            s_candidate_samples = 1;
        }
        else if (s_candidate_samples < CLIFF_DEBOUNCE_SAMPLES)
        {
            s_candidate_samples++;
        }

        if (s_candidate_samples >= CLIFF_DEBOUNCE_SAMPLES &&
            s_candidate_mask != s_stable_mask)
        {
            uint8_t previous = s_stable_mask;
            s_stable_mask = s_candidate_mask;

            if (previous == 0 && s_stable_mask != 0)
            {
                /* 先置保护标志再停车：命令任务即使正好在写电机，
                 * exec_vehicle_state_update() 也会把运动状态压成 STOP。
                 * 清空队列可丢掉触发前排队的旧运动命令。 */
                motor_direct_executor_set_inhibit(1);
                reset_command_queue();
                motor_direct_executor.reset();
            }
            else if (previous != 0 && s_stable_mask == 0)
            {
                /* 传感器恢复（通常是已经倒车离开悬崖），解除保护。 */
                motor_direct_executor_set_inhibit(0);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CLIFF_POLL_PERIOD_MS));
    }
}

void cliff_sensor_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* PB3=JTDO、PB4=NJTRST，复位后默认是 JTAG 引脚。
     * 关闭 JTAG-DP，保留 SW-DP，ST-Link 仍可通过 PA13/PA14 烧录调试。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 输入上拉：模块 DO 为推挽输出时由模块驱动；模块掉线时读高电平。
     * 默认的“高电平=悬崖”配置下，掉线会按触发处理，属于安全侧。 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = CLIFF_LEFT_PIN;
    GPIO_Init(CLIFF_LEFT_GPIO, &gpio);

    gpio.GPIO_Pin = CLIFF_RIGHT_PIN;
    GPIO_Init(CLIFF_RIGHT_GPIO, &gpio);

    /* 上电先同步采样一次：如果开机时就停在悬崖边，调度器启动后不会
     * 再有几十毫秒的“先动起来再触发”窗口。第一次读数直接当作稳定值，
     * 后续由任务做消抖更新。 */
    s_candidate_mask = cliff_read_raw_mask();
    s_stable_mask = s_candidate_mask;
    s_candidate_samples = CLIFF_DEBOUNCE_SAMPLES;

    if (s_stable_mask != 0)
    {
        motor_direct_executor_set_inhibit(1);
    }

    if (xTaskCreate(cliff_sensor_task, "CLIFF", 160, NULL, 3, NULL) != pdPASS)
    {
        /* 监控任务创建失败时不能“带病前进”：封锁运动，只允许停车/后退。 */
        s_stable_mask = CLIFF_ANY_MASK;
        motor_direct_executor_set_inhibit(1);
    }
}

uint8_t cliff_sensor_get_mask(void)
{
    return s_stable_mask;
}

uint8_t cliff_sensor_command_allowed(uchar command)
{
    if (s_stable_mask == 0)
    {
        return 1;
    }

    return (command == COMMAND_STOP || command == COMMAND_BACK) ? 1 : 0;
}
