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
    SystemCoreClockUpdate(); /* BluePill board runs at 72 MHz */

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

	while(1)
	{
		// GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		// delay_ms(2000);
        // GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		// delay_ms(2000);
        LED = 0;
		delay_ms(2000);
        LED = 1;
		delay_ms(2000);
        execute_commands("1C");
		delay_ms(2000);
        execute_commands("2C");
		delay_ms(2000);
        execute_commands("3C");
		delay_ms(2000);
        execute_commands("4C");
		delay_ms(2000);
        execute_commands("5C");
		delay_ms(2000);
        execute_commands("6C");
		delay_ms(2000);
        execute_commands("7C");
		delay_ms(2000);
        execute_commands("8C");
		delay_ms(2000);
        execute_commands("9C");
		delay_ms(2000);
        execute_commands("AC");
		delay_ms(2000);
        execute_commands("BC");
		delay_ms(2000);
        execute_commands("0C");
	}
}