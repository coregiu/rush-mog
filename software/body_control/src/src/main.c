/**
  ******************************************************************************
  *
  * The main function of rush mog.
  * author: coregiu
  *
  *
  * *****************************************************************************
  */

#include <controller.h>

int main(void)
{
    clock_init();
    delay_init();

    init_protocols();
    LED = 0;

    init_modules();
    uart_log_start_info();
    delay_ms(500);

    LED = !LED; // PC13 位带操作：写 0 亮、写 1 灭
    vTaskStartScheduler();
    while (1)
    {
    }
}