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

    // 串口配置（底盘链路，与树莓派 edge_server 的 base.baud 一致）
    USART_InitStructure.USART_BaudRate = BODY_UART_BAUD_RATE;
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

/* 调试计数：用 ST-Link 直接 watch 这三个变量即可定位“命令没反应”。
 *   g_ps2_rx_bytes  在涨 -> 串口有数据进来（接线/波特率没问题）
 *   g_ps2_rx_frames 不涨 -> 一直没收到 '\n' / '\r'（串口工具要勾选“发送换行”）
 *   两个都涨但 LED 不闪 -> 问题在命令队列/执行任务，不在串口接收 */
volatile uint32_t g_ps2_rx_bytes = 0;
volatile uint32_t g_ps2_rx_frames = 0;
volatile uint32_t g_ps2_rx_last_byte = 0;

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
*	说明：以 '\n' 或 '\r' 作为一条命令的结束符。
*	      "\r\n" 只会下发一次命令（旧代码会下发两次，导致 LED 被切换两下
*	      看起来像“没反应”）。
*
************************************************************
*/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收中断
    {
        /* 读 DR 会自动清 RXNE。不要再调用 USART_ClearFlag(USART3, USART_FLAG_RXNE)：
         * 那个函数是往 SR 写 ~FLAG，会把 TC 等标志一起清掉，而编码器遥测任务
         * 正在轮询 USART3 的 TC，会被这里打断。 */
        uchar byte = (uchar)USART_ReceiveData(USART3);

        g_ps2_rx_bytes++;
        g_ps2_rx_last_byte = byte;

        if (byte == '\n' || byte == '\r')
        {
            if (uart3_data_position > 0) // 空行（如 "\r\n" 的第二个字符）直接忽略
            {
                struct command_context command_context = {0};
                command_context.commands[0] = uart3_receive_data[0];
                command_context.cmd_length = 1;
                command_context.module = MODULE_MOTOR_DIRECT;
                command_context.time_sleep_milsec = 0;
                command_context.delay_type = DELAY_AFTER_EXE;

                if (uart3_data_position > 1)
                {
                    command_context.commands[1] = uart3_receive_data[1];
                    command_context.cmd_length = 2;
                }

                g_ps2_rx_frames++;

                if (command_context.commands[0] != COMMAND_RESET)
                {
                    send_to_queue_isr(&command_context);
                }
                else
                {
                    send_to_queue_front_isr(&command_context);
                }

                uart3_data_position = 0;
            }
        }
        else
        {
            if (uart3_data_position >= DEFAULT_BUFFER_SIZE) //防止数据过多，导致内存溢出
            {
                uart3_data_position = 0;
            }
            uart3_receive_data[uart3_data_position] = byte;
            uart3_data_position++;
        }
    }
}

const struct module_command_receiver ps2_receiver = {init_ps2_state, receive_ps2_command};
