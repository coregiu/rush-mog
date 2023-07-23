#include "includes.h"
#include "FreeRTOS.h"
#include "task.h"


void vTaskLed(void * pvParameters)
{
    while(1)
    {
       LED1_ON;
       vTaskDelay(1000);
       LED1_OFF;
       vTaskDelay(1000);
    }
}

int main(void)
{
	/* LED初始化 */
	LED_Init();
	/* 创建任务 */
	xTaskCreate(vTaskLed,"Task Led",512,NULL,1,NULL);
	/* 启动调度， 开始执行任务 */
	vTaskStartScheduler();
	while(1);
}
