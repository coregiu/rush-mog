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

void init_audio_state()
{
    /* GPIO端口设置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); /* 使能USART1，GPIOA时钟 */

    /* PA2 TXD2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 RXD2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Usart1 NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; /* 抢占优先级3 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        /* 子优先级3 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           /* IRQ通道使能 */
    NVIC_Init(&NVIC_InitStructure);                           /* 根据指定的参数初始化VIC寄存器 */

    /* USART 初始化设置 */
    USART_InitStructure.USART_BaudRate = 9600;                                      /* 串口波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /* 字长为8位数据格式 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /* 一个停止位 */
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /* 无奇偶校验位 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /* 无硬件数据流控制 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /* 收发模式 */

    USART_Init(USART2, &USART_InitStructure);      /* 初始化串口1 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); /* 开启串口接受中断 */
    USART_Cmd(USART2, ENABLE);                     /* 使能串口 2 */
}

char *receive_audio_command()
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint16_t res = USART_ReceiveData(USART2);
        USART_SendData(USART2, res);
    }

    return "1C";
}

const struct module_command_receiver audio_receiver = {init_audio_state, receive_audio_command};
