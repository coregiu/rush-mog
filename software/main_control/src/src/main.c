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
    delay_init();

    init_modules();

    uart_log_start_info();

    LED = 0;
	delay_ms(500);
    LED = 1;
	delay_ms(500);

	// for(int i=0;i<25;i++)
	// {

	// 	//PCA_MG9XX(1,0,180,2,5+5*i); //通道1，慢慢的转
	// 	//PCA_MG9XX(2,0,180,2,5+5*i); //通道2，慢慢的转

    //     LED = ~LED;
	// 	pca_mg9xx(0,0,150,1,10); //通道1 从0度转到180度
	// 	// PCA_MG9XX(2,0,180,1,10); //通道2 从0度转到180度
	// 	// PCA_MG9XX(1,180,0,1,10); //通道1 从180度转到0度
	// 	// PCA_MG9XX(2,180,0,1,10); //通道2 从180度转到0度
	// }

    // LED = 0;
    // delay_ms(500);
    // LED = 1;
    // delay_ms(500);

    vTaskStartScheduler();
    while (1)
    {
    }
}