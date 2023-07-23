#ifndef __LED_H
#define __LED_H	 

#include "includes.h"
#include "stm32f4xx.h"

#define GPIO_Toggle(port, bit) {    if(GPIO_ReadOutputDataBit(port,bit))   \
                                       GPIO_ResetBits(port, bit);          \
                                    else                                   \
                                        GPIO_SetBits(port, bit);           \
                               }

#define  LED1_ON         GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define  LED1_OFF        GPIO_SetBits(GPIOC,GPIO_Pin_2)    
#define  LED1_TOGGLE     GPIO_Toggle(GPIOC,GPIO_Pin_2)                               

#define  LED2_ON         GPIO_ResetBits(GPIOC,GPIO_Pin_0)
#define  LED2_OFF        GPIO_SetBits(GPIOC,GPIO_Pin_0)    
#define  LED2_TOGGLE     GPIO_Toggle(GPIOC,GPIO_Pin_0)   															 
					
#define  LED3_ON         GPIO_ResetBits(GPIOD,GPIO_Pin_3)
#define  LED3_OFF        GPIO_SetBits(GPIOD,GPIO_Pin_3)    
#define  LED3_TOGGLE     GPIO_Toggle(GPIOD,GPIO_Pin_3)  
															 
void LED_Init(void);//≥ı ºªØ
										 
#endif





