/**
  ******************************************************************************
  *
  * Define of command, receiver and executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <controller.h>
#include "task_manager.h"
// the map of command to module
const char command_module_map[COMMANDS_LENGTH][2] = {{COMMAND_STOP,          MODULE_MOTOR_DIRECT},
                                                      {COMMAND_RUN,          MODULE_MOTOR_DIRECT},
                                                      {COMMAND_BACK,         MODULE_MOTOR_DIRECT},
                                                      {COMMAND_LEFT_RUN,     MODULE_MOTOR_DIRECT},
                                                      {COMMAND_RIGHT_RUN,    MODULE_MOTOR_DIRECT},
                                                      {COMMAND_LEFT_FRONT,   MODULE_MOTOR_DIRECT},
                                                      {COMMAND_RIGHT_FRONT,  MODULE_MOTOR_DIRECT},
                                                      {COMMAND_LEFT_BACK,    MODULE_MOTOR_DIRECT},
                                                      {COMMAND_RIGHT_BACK,   MODULE_MOTOR_DIRECT},
                                                      {COMMAND_LEFT_TURN,    MODULE_MOTOR_DIRECT},
                                                      {COMMAND_RIGHT_TURN,   MODULE_MOTOR_DIRECT},
                                                      {COMMAND_TURN_OUT_L,   MODULE_MOTOR_DIRECT},
                                                      {COMMAND_TURN_OUT_R,   MODULE_MOTOR_DIRECT},
                                                      {COMMAND_TEST_VEHICLE, MODULE_MOTOR_DIRECT},
                                                      {COMMAND_FAST,         MODULE_MOTOR_PWM},
                                                      {COMMAND_SLOW,         MODULE_MOTOR_PWM},
                                                      {COMMAND_LEFT_MICRO,   MODULE_MOTOR_PWM},
                                                      {COMMAND_RIGHT_MICRO,  MODULE_MOTOR_PWM},
                                                      {COMMAND_DIRECT,       MODULE_MOTOR_PWM},
                                                      {COMMAND_RESET,        MODULE_RESET},
                                                      {COMMAND_AUDIO_NEXT,   MODULE_AUDIO},
                                                      {COMMAND_AUDIO_PREV,   MODULE_AUDIO},
                                                      {COMMAND_AUDIO_VOL_UP, MODULE_AUDIO},
                                                      {COMMAND_AUDIO_VOL_DOWN, MODULE_AUDIO},
                                                      {COMMAND_AUDIO_STOP,   MODULE_AUDIO},
                                                      {COMMAND_AUDIO_PLAY,   MODULE_AUDIO},
                                                      {COMMAND_UNKNOWN,      MODULE_UNKNOWN}};

static BaseType_t priority = 2;
static BaseType_t *const pxHigherPriorityTaskWoken = &priority;

void init_protocols()
{
    init_command_led_module();
    // IIC_Init();
}

/**
 * init uart and all receive and executor modules
 *
 */
void init_modules()
{
    audio_receiver.init();
    ps2_receiver.init();
    motor_direct_executor.init();

    init_freertos();

    motor_pwm_executor.init();
    arm_roboot_executor.init();
    audio_executor.init();
}

/**
 * notify modules to execute the commands.
 * car_module: the module of car, such as motor, music, led
 * command: the command
 *
 */
void execute_command(struct command_context *command_context)
{
    LED = ~LED;
    uchar exe_cmd = command_context->commands[0];
    uart_log_string_data("receive command: ");
    uart_log_data(exe_cmd);

    uint cmd_seq = convert_command_seq(exe_cmd);
    if (cmd_seq < 0 || cmd_seq > COMMANDS_LENGTH)
    {
        return;
    }
    
    command_context->module = command_module_map[cmd_seq][1];

    switch (command_context->module)
    {
    case MODULE_MOTOR_DIRECT:
        if (command_context->cmd_length >= 2)
        {
            uchar pwm_rate = convert_pwm_rate(command_context->commands[1]);
            pwm_rate = pwm_rate <= STOP_PWM ? STOP_PWM : pwm_rate;
            pwm_rate = pwm_rate >= MAX_PWM_RATE ? MAX_PWM_RATE : pwm_rate;
            command_context->pwm_rate = pwm_rate;
            motor_pwm_executor.update_state(command_context);
        }
        motor_direct_executor.update_state(command_context);

        struct command_context audio_command_context = {0};
        audio_command_context.commands[0] = COMMAND_AUDIO_PLAY;
        audio_command_context.commands[1] = '1';
        audio_command_context.cmd_length = 2;
        audio_executor.update_state(&audio_command_context);
        break;
    case MODULE_MOTOR_PWM:
        motor_pwm_executor.update_state(command_context);
        break;
    case MODULE_ARM_BOT:
        arm_roboot_executor.update_state(command_context);
        break;
    case MODULE_AUDIO:
        audio_executor.update_state(command_context);
        break;
    case MODULE_RESET:
        reset_command_queue();
        motor_pwm_executor.reset();
        motor_direct_executor.reset();
        arm_roboot_executor.reset();
        audio_executor.reset();
        break;

    default:

        break;
    }

}

void reset_command_queue()
{
    if (command_queue == NULL)
    {
        uart_log_string_data("command queue is null");
        return;
    }
    BaseType_t xStatus = xQueueReset(command_queue);
    if (xStatus != pdPASS)
    {
        uart_log_string_data("failed to reset command queue");
    }
}

void send_to_queue_isr(struct command_context *command)
{
    if (command_queue == NULL)
    {
        uart_log_string_data("command queue is null");
        return;
    }
    // BaseType_t xStatus = xQueueSend(command_queue, command, pdMS_TO_TICKS(100));
    // BaseType_t xStatus = xQueueSendFromISR(command_queue, &(command->command), xTicksToWait);
    BaseType_t xStatus = xQueueSendFromISR(command_queue, command, pxHigherPriorityTaskWoken);
    if (xStatus != pdPASS)
    {
        uart_log_start_info("failed to send data"); //如果发送数据失败在这里进行错误处理
    }
}

void send_to_queue(struct command_context *command)
{
    if (command_queue == NULL)
    {
        uart_log_string_data("command queue is null");
        return;
    }
    // BaseType_t xStatus = xQueueSend(command_queue, command, pdMS_TO_TICKS(100));
    // BaseType_t xStatus = xQueueSendFromISR(command_queue, &(command->command), xTicksToWait);
    BaseType_t xStatus = xQueueSend(command_queue, command, pdMS_TO_TICKS(100));
    if (xStatus != pdPASS)
    {
        uart_log_start_info("failed to send data"); //如果发送数据失败在这里进行错误处理
    }
}

void send_to_queue_front_isr(struct command_context *command)
{
    if (command_queue == NULL)
    {
        uart_log_string_data("command queue is null");
        return;
    }

    BaseType_t xStatus = xQueueSendToFrontFromISR(command_queue, command, pxHigherPriorityTaskWoken);
    if (xStatus != pdPASS)
    {
        uart_log_start_info("failed to send data"); //如果发送数据失败在这里进行错误处理
    }
}


void clock_init(void)
{
    // Reset the RCC clock configuration to the default reset state.
    // HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz
    // RCC_DeInit();

    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready

    // 2. Configure PLL
    RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Clear PLL source bit
    RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_Div2; // Set HSE/2 as PLL source
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE; // No pre-scaling on HSE
    RCC->CFGR &= ~((0x7 << 18) & RCC->CFGR); // 清除原有的PLL乘法因子位
    RCC->CFGR |= (9 << 18); // 设置PLL乘法因子为9

    // 3. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready

    // 4. Switch to PLL
    RCC->CFGR &= ~RCC_CFGR_SW; // Clear active clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Set PLL as active clock source

    // 5. Wait until PLL is used as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 6. Configure AHB/APB prescalers
    RCC->CFGR &= ~RCC_CFGR_HPRE; // Set AHB prescaler to 1
    RCC->CFGR &= ~RCC_CFGR_PPRE1; // Set APB1 prescaler to 2
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // Set APB2 prescaler to 1

    // 7. Update SystemCoreClock variable
    SystemCoreClockUpdate();

    // BluePill board runs at 72 MHz
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // Capture error
        while (1)
            ;
    }
}
