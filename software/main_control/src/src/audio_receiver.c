/**
  ******************************************************************************
  *
  * Implement of audio receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <audio_receiver.h>
#include "controller.h"

void init_audio_state()
{
    /* GPIO端口设置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); /* 使能USART2，GPIOA时钟 */

    /* PA2 TXD2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 RXD2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Usart2 NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; /* 抢占优先级3 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        /* 子优先级0 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           /* IRQ通道使能 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);           /* 设置优先级分组为4组，即抢占优先级范围为0-15 */
    NVIC_Init(&NVIC_InitStructure);                           /* 根据指定的参数初始化VIC寄存器 */

    /* USART 初始化设置 */
    USART_InitStructure.USART_BaudRate = DEFAULT_BOUND_RATE;                        /* 串口波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /* 字长为8位数据格式 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /* 一个停止位 */
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /* 无奇偶校验位 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /* 无硬件数据流控制 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /* 收发模式 */
    USART_Init(USART2, &USART_InitStructure);                                       /* 初始化串口2 */

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); /* 开启串口接受中断 */
    USART_Cmd(USART2, ENABLE); /* 使能串口 2 */
    USART_ClearFlag(USART2, USART_FLAG_TC);
}

char *receive_audio_command()
{
    return "0";
}

char uart2_receive_data[DEFAULT_BUFFER_SIZE] = {0};
int data_position = 0;
/*
************************************************************
*	函数名称：	USART2_IRQHandler
*
*	函数功能：	串口2收发中断
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：
************************************************************
*/
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收中断
    {
        if (data_position >= DEFAULT_BUFFER_SIZE) //防止数据过多，导致内存溢出
        {
            data_position = 0;
        }
        uart2_receive_data[data_position] = USART_ReceiveData(USART2);
        data_position++;

        /* Check if the previous byte was a newline */
        if (uart2_receive_data[data_position - 1] == '\n' || uart2_receive_data[data_position - 1] == '\r')
        {
            /* Send the line back */
            for (uint i = 0; i < data_position; i++)
            {
                uart_log_data(uart2_receive_data[i]);
            }
            execute_commands(uart2_receive_data, COMMAND_TYPE_MANUAL);
            data_position = 0;
        }

        USART_ClearFlag(USART2, USART_FLAG_RXNE);
    }
}

const struct module_command_receiver audio_receiver = {init_audio_state, receive_audio_command};
