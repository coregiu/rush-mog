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

// 拷贝 UART3 接收数据到 cmd_buffer
void copy_uart3_data(uchar *cmd_buffer, uchar *uart3_receive_data, int uart3_data_position)
{
    // 清空目标数组
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    
    // 拷贝（只拷贝有效长度，不拷贝多余脏数据）
    memcpy(cmd_buffer, uart3_receive_data, uart3_data_position);
    
    // 拷贝完成后，清空接收缓冲区（关键！）
    memset(uart3_receive_data, 0, sizeof(uart3_receive_data));
    uart3_data_position = 0;
}

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
        if (uart3_data_position >= DEFAULT_BUFFER_SIZE) //防止数据过多，导致内存溢出
        {
            uart3_data_position = 0;
        }
        uart3_receive_data[uart3_data_position] = USART_ReceiveData(USART3);
        uart3_data_position++;

        /* Check if the previous byte was a newline */
        if (uart3_receive_data[uart3_data_position - 1] == '\n' || uart3_receive_data[uart3_data_position - 1] == '\r')
        {
            /* Send the line back */
            // for (uint i = 0; i < uart3_data_position; i++)
            // {
            //     uart_log_data(uart3_receive_data[i]);
            // }
            // 定义一个目标数组（你可以放到全局）
            struct command_context command_context = {0};
            command_context.commands[0] = uart3_receive_data[0];
            command_context.cmd_length = 1;
            command_context.module = MODULE_MOTOR_DIRECT;
            command_context.time_sleep_milsec = 0;
            command_context.delay_type = DELAY_AFTER_EXE;

            if (uart3_data_position > 2)
            {
                command_context.commands[1] = uart3_receive_data[1];
                command_context.cmd_length = 2;
            }
            
            send_to_queue_isr(&command_context);
            // execute_commands(uart3_receive_data, uart3_data_position, COMMAND_TYPE_AUTO);
            uart3_data_position = 0;
        }

        USART_ClearFlag(USART3, USART_FLAG_RXNE);
    }
}

const struct module_command_receiver ps2_receiver = {init_ps2_state, receive_ps2_command};
