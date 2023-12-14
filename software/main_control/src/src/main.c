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

#include "stm32_pca9685.h"

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
	PCA_MG9XX_Init(60,90);//初始化舵机驱动
	delay_ms(1000);
    LED = 1;
	delay_ms(1000);

	for(int i=0;i<25;i++)
	{

		//PCA_MG9XX(1,0,180,2,5+5*i); //通道1，慢慢的转
		//PCA_MG9XX(2,0,180,2,5+5*i); //通道2，慢慢的转

        LED = ~LED;
		PCA_MG9XX(1,0,180,1,10); //通道1 从0度转到180度
		// PCA_MG9XX(2,0,180,1,10); //通道2 从0度转到180度
		// PCA_MG9XX(1,180,0,1,10); //通道1 从180度转到0度
		// PCA_MG9XX(2,180,0,1,10); //通道2 从180度转到0度
	}

    LED = 0;
    delay_ms(500);
    LED = 1;
    delay_ms(500);

    vTaskStartScheduler();
    while (1)
    {
    }
}