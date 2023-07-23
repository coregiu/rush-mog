#ifndef __USART_H
#define __USART_H

#include "includes.h"




/**********外部函数调用开始*************/
void uart1_Init(u32 bound); 		//串口1初始化函数
void UartSendBuff(u8 * buff, u16 len);
void Printf(USART_TypeDef* USARTx, char* fmt, ...);
void uart1_process(void);
/**********外部函数调用结束*************/


#endif
