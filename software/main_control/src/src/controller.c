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
const char command_module_map[COMMANDS_LENGTH][2] = {{COMMAND_STOP, MODULE_VEHICLE},
                                                      {COMMAND_RUN, MODULE_VEHICLE},
                                                      {COMMAND_BACK, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_RUN, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_RUN, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_FRONT, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_FRONT, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_BACK, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_BACK, MODULE_VEHICLE},
                                                      {COMMAND_LEFT_TURN, MODULE_VEHICLE},
                                                      {COMMAND_RIGHT_TURN, MODULE_VEHICLE},
                                                      {COMMAND_GO_BACK, MODULE_VEHICLE},
                                                      {COMMAND_TURN_OUT, MODULE_VEHICLE},
                                                      {COMMAND_TEST_VEHICLE, MODULE_VEHICLE},
                                                      {COMMAND_TEST_ROBOOT, MODULE_ROBOOT},
                                                      {COMMAND_OPEN_VEDIO, MODULE_VEDIO},
                                                      {COMMAND_CLOSE_VEDIO, MODULE_VEDIO},
                                                      {COMMAND_OPEN_INTELI, MODULE_VEDIO},
                                                      {COMMAND_CLOSE_INTELI, MODULE_VEDIO},
                                                      {COMMAND_PLAYING, MODULE_INTELI},
                                                      {COMMAND_ADAPTE_SERVO, MODULE_ROBOOT},
                                                      {COMMAND_ATTITUDE_INFO, MODULE_ATTITUDE},
                                                      {COMMAND_LED_DISPLAY, MODULE_LED},
                                                      {COMMAND_UNKNOWN, MODULE_UNKNOWN}};

static BaseType_t priority = 2;
static BaseType_t *const pxHigherPriorityTaskWoken = &priority;

void init_protocols()
{
    init_command_led_module();
    IIC_Init();
}

/**
 * init uart and all receive and executor modules
 *
 */
void init_modules()
{
    audio_receiver.init();
    vehicle_executor.init();

    init_freertos();

    arm_roboot_executor.init();
    led_display_executor.init();
    timer_manager.init();
    attitude_executor.init();
}

/**
 * receive commands from clients.
 */
char* receive_commands()
{
    return audio_receiver.receive_commands();
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
    switch (command_context->module)
    {
    case MODULE_VEHICLE:
        vehicle_executor.update_state(command_context);
        break;
    case MODULE_VEDIO:
        video_executor.update_state(command_context);
        break;
    case MODULE_ROBOOT:
        arm_roboot_executor.update_state(command_context);
        break;
    case MODULE_INTELI:
        command_context->command = 'D';
        vehicle_executor.update_state(command_context);
        break;
    case MODULE_LED:
        led_display_executor.update_state(command_context);
        break;
    case MODULE_ATTITUDE:
        attitude_executor.update_state(command_context);
        break;

    default:

        break;
    }

}

/**
 * execute commands;
 *
 */
void execute_commands(char *commands, enum command_type type)
{
    uint cmd_seq = convert_command_seq(commands[0]);
    if (cmd_seq >= 0 && cmd_seq < COMMANDS_LENGTH)
    {
        struct command_context command_context = {commands[0], command_module_map[cmd_seq][1], 0, DELAY_AFTER_EXE, type};
        execute_command(&command_context);
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
    BaseType_t xStatus = xQueueSendFromISR(command_queue, command, pxHigherPriorityTaskWoken);
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
