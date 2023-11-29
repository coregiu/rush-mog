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

static uint DEFAULT_BUFFER_SIZE = 64;

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; /* 抢占优先级0 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        /* 子优先级0 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           /* IRQ通道使能 */
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
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE); /* 使能串口 2 */
    USART_ClearFlag(USART2, USART_FLAG_TC);
}

char *receive_audio_command()
{
    // uart_log_data('1');
    uint16_t rxbuf[DEFAULT_BUFFER_SIZE];
    int rxbuf_pos = 0;
    int i;

    // uart_log_data('2');
    while (1)
    {
        // uart_log_data('3');
        /* Wait until there's data in the receive data register */
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
        // while (USART_GetITStatus(USART2, USART_IT_RXNE) == RESET)
            ;
        LED = 0;
        // uart_log_data('4');
        /* Read a byte */
        rxbuf[rxbuf_pos] = USART_ReceiveData(USART2);
        // uart_log_data('5');
        // uart_log_data(rxbuf[rxbuf_pos]);
        rxbuf_pos++;

        /* Check if the previous byte was a newline */
        if (rxbuf[rxbuf_pos - 1] == '\n' || rxbuf[rxbuf_pos - 1] == '\r' || rxbuf_pos >= DEFAULT_BUFFER_SIZE - 1)
        {
            // uart_log_data('7');
            /* Send the line back */
            for (i = 0; i < rxbuf_pos; i++)
            {
                // uart_log_data('8');
                // uart_log_data(rxbuf[i]);
            }
            // uart_log_data('9');
            rxbuf_pos = 0;
        }
        // uart_log_data('A');
    }

    return "1C";
}

const struct module_command_receiver audio_receiver = {init_audio_state, receive_audio_command};
