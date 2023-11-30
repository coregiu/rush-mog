/**
  ******************************************************************************
  *
  * The main function of ago.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <controller.h>

static volatile uint32_t s_tick_count = 0;

void SysTick_Handler(void)
{
    s_tick_count++;
}

static void clock_init(void)
{
    /* Reset the RCC clock configuration to the default reset state. */
    /* HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz */
    RCC_DeInit();
    /* BluePill board runs at 72 MHz */
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1)
            ;
    }
}

int main(void)
{
    clock_init();
    init_modules();
    delay_init();

    uart_log_start_info();

    LED = 0;
    delay_ms(1000);
    LED = 1;
    delay_ms(1000);

    while (1)
    {
        char *commands = audio_receiver.receive_commands();
        execute_commands(commands);
    }
}