#include "led.h"
#include "delay.h"
#include "sys.h"
//ALIENTEK miniSTM32开发板实验1
//跑马灯实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
 int main(void)
 {	
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	//初始化与LED连接的硬件接口
	while(1)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_8); //LED0输出低
		GPIO_SetBits(GPIOD,GPIO_Pin_2);//LED1输出高
		delay_ms(300);
		GPIO_SetBits(GPIOA,GPIO_Pin_8);//LED0输出高
		GPIO_ResetBits(GPIOD,GPIO_Pin_2);//LED1输出低
		delay_ms(300);
	}
 }

