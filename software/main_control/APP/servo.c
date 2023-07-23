#include "servo.h"
#include "usart.h"


/*
*********************************************************************************************************
*	函 数 名: PWM_gpio配置
*	功能说明: PWM输出引脚设置
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void Servo_Gpio_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTD时钟	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;           //GPIOD15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	 //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);                //初始化PD15
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); //GPIOD15复用为定时器4
	
}


/*
*********************************************************************************************************
*	函 数 名: TIM4_Int_Init
*	功能说明: 定时器初始化函数
*	形    参: 1.自动重装值  2.时钟预分频数
*	返 回 值: 无
*********************************************************************************************************
*/ 
void TIM4_Int_Init(u16 reload ,u16 prescaler)
{
	Servo_Gpio_Config();
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = reload; 	     //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=prescaler;   //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	//初始化TIM4 Channel4 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = 0;				//比较初始值
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR4上的预装载寄存器
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 	
	TIM_Cmd(TIM4,ENABLE); 	//使能定时器4
	
}


//设置舵机角度
void Set_Servo_Angle(uint8_t angle)
{
	float T;
	uint32_t count;
	if(angle > 180)
	{
		return;
	}
	T = 0.5 + (angle*2/180);
	count = 999 * T / 20;
	TIM_SetCompare4(TIM4,count);
}

