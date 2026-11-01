/**
  ******************************************************************************
  *
  * Implement of audio receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <ps2_receiver.h>
#include "controller.h"

void init_ps2_state()
{
    /* GPIO端口设置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // PB10 = TX (推挽复用)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB11 = RX (上拉输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 串口配置
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    // 开启接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    // NVIC 中断配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_ClearFlag(USART3, USART_FLAG_TC);
}

char *receive_ps2_command()
{
    return "0";
}

uchar uart3_receive_data[DEFAULT_BUFFER_SIZE] = {0};
int uart3_data_position = 0;
static int uart3_frame_overflow = 0;

/*
************************************************************
*	函数名称：	USART3_IRQHandler
*
*	函数功能：	串口3收发中断
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：
************************************************************
*/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收中断
    {
        uchar ch = (uchar)USART_ReceiveData(USART3);

        if (ch == '\r')
        {
            // 兼容 CRLF：忽略 CR，只在 LF 时提交一帧
        }
        else if (ch == '\n')
        {
            if (!uart3_frame_overflow && uart3_data_position > 0)
            {
                struct command_context command_context = {0};
                command_context.commands[0] = uart3_receive_data[0];
                command_context.cmd_length = 1;
                command_context.module = MODULE_MOTOR_DIRECT;
                command_context.time_sleep_milsec = 0;
                command_context.delay_type = DELAY_AFTER_EXE;

                if (uart3_data_position >= 2)
                {
                    command_context.commands[1] = uart3_receive_data[1];
                    command_context.cmd_length = 2;
                }

                if (command_context.commands[0] != COMMAND_RESET)
                {
                    send_to_queue_isr(&command_context);
                }
                else
                {
                    send_to_queue_front_isr(&command_context);
                }
            }

            uart3_data_position = 0;
            uart3_frame_overflow = 0;
        }
        else
        {
            if (uart3_data_position < DEFAULT_BUFFER_SIZE)
            {
                uart3_receive_data[uart3_data_position++] = ch;
            }
            else
            {
                // 超长帧丢弃到下一个 LF，避免半帧被当成命令执行
                uart3_frame_overflow = 1;
            }
        }

        USART_ClearFlag(USART3, USART_FLAG_RXNE);
    }
}

const struct module_command_receiver ps2_receiver = {init_ps2_state, receive_ps2_command};
