/**
  ******************************************************************************
  *
  * Implement of JQ6500 MP3 module executor.
  * Controls JQ6500 via USART2 (shared with audio_receiver on PA2 TX).
  *
  * JQ6500 Serial Protocol:
  *   Frame format: 7E [len] [cmd] [data...] EF
  *   - len: number of bytes after len (cmd + data)
  *   - cmd: command byte
  *   - data: optional data bytes
  *   - All values in hex
  *
  * author: coregiu
  * 修复记录：
  * 1. 移除协议中多余 0xFF 字节（核心帧格式错误）
  * 2. 修正播放模式命令码、曲目索引大小端（大端序）
  * 3. 修正停止、播放模式对应协议指令
  ******************************************************************************
**/

#include <audio_executor.h>
#include "controller.h"


/************************ 底层串口发送函数 ************************/
/* 通过 USART2 发送一个字节（阻塞轮询方式） */
static void uart2_send_byte(uint8_t data)
{
    USART_SendData(USART2, data);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_ClearFlag(USART2, USART_FLAG_TC);
}

/************************ JQ6500 协议帧组包发送 ************************/
static void jq6500_send_frame(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t i;

    // 帧头：仅 0x7E，移除原错误的 0xFF
    uart2_send_byte(0x7E);

    // 长度域：命令字节(1) + 数据字节长度
    uart2_send_byte(2 + data_len);

    // 命令码
    uart2_send_byte(cmd);

    // 附加数据
    for (i = 0; i < data_len; i++)
    {
        uart2_send_byte(data[i]);
    }

    // 帧尾
    uart2_send_byte(0xEF);
}

/* 发送无参数简单命令 */
static void jq6500_send_simple_cmd(uint8_t cmd)
{
    jq6500_send_frame(cmd, 0, NULL);
}

/************************ 功能接口 ************************/
/* 设置音量 范围 0 ~ 30 */
static void jq6500_set_volume(uint8_t volume)
{
    if (volume > 30)
    {
        volume = 30;
    }
    uint8_t vol_data[1] = {volume};
    jq6500_send_frame(JQ_CMD_SET_VOL, 1, vol_data);
}

/**
 * @brief  设置播放模式
 * @param  mode:
 *         0x00 = 单曲循环
 *         0x01 = 播放一次停止
 *         0x02 = 全部循环
 *         0x03 = 随机播放
 */
static void jq6500_set_play_mode(uint8_t mode)
{
    uint8_t mode_data[1] = {mode};
    // 修复：原错误使用 JQ_CMD_PLAY，改为模式专用指令 JQ_CMD_SET_PLAY_MODE
    jq6500_send_frame(JQ_CMD_SET_PLAY_MODE, 1, mode_data);
}

/* 按索引播放曲目 JQ6500 索引从 1 开始，协议为 大端序(高字节在前) */
static void jq6500_play_by_index(uint16_t index)
{
    // 修复：原小端序错误，改为 高字节在前，低字节在后
    uint8_t index_data[2] = {(uint8_t)((index >> 8) & 0xFF), (uint8_t)(index & 0xFF)};
    jq6500_send_frame(JQ_CMD_PLAY_INDEX, 2, index_data);
}

/************************ 模块初始化 ************************/
void init_jq6500_state(void)
{
    /* GPIO和USART2初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 使能 USART2 和 GPIOA 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA2 TXD2 - 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 RXD2 - 浮空输入（接收模块返回状态） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 配置：9600 8N1 (JQ6500固定波特率) */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);

    uart_log_string_data("jq6500 executor init");
    delay_ms(500); // 等待模块上电稳定

    // 默认全部循环模式
    jq6500_set_play_mode(0x02);
    delay_ms(50);

    // 初始化音量最大值
    jq6500_set_volume(30);
    delay_ms(50);

    // 播放第一首初始化状态（JQ6500 曲目索引从1开始）
    jq6500_play_by_index(1);
}

/************************ 状态复位 ************************/
void reset_jq6500_state(void)
{
    jq6500_set_play_mode(0x02);
    jq6500_play_by_index(1);
}

/************************ 命令解析与执行 ************************/
void update_jq6500_state(struct command_context *command_context)
{
    uint8_t cmd = command_context->commands[0];

    switch (cmd)
    {
        case COMMAND_AUDIO_NEXT:
            jq6500_send_simple_cmd(JQ_CMD_NEXT);
            uart_log_string_data("jq6500: next track");
            break;

        case COMMAND_AUDIO_PREV:
            jq6500_send_simple_cmd(JQ_CMD_PREV);
            uart_log_string_data("jq6500: prev track");
            break;

        case COMMAND_AUDIO_VOL_UP:
            jq6500_send_simple_cmd(JQ_CMD_VOL_UP);
            uart_log_string_data("jq6500: vol up");
            break;

        case COMMAND_AUDIO_VOL_DOWN:
            jq6500_send_simple_cmd(JQ_CMD_VOL_DOWN);
            uart_log_string_data("jq6500: vol down");
            break;

        case COMMAND_AUDIO_STOP:
            jq6500_send_simple_cmd(JQ_CMD_STOP);
            uart_log_string_data("jq6500: stop");
            break;

        case COMMAND_AUDIO_PLAY:
            if (command_context->cmd_length >= 2)
            {
                uint8_t track_index = command_context->commands[1] - '0';
                // JQ6500 合法索引范围 1~2999
                if(track_index < 1) track_index = 1;
                jq6500_play_by_index(track_index);
                uart_log_string_data("jq6500: play track ");
                uart_log_number(track_index);
            }
            else
            {
                jq6500_play_by_index(1);
                uart_log_string_data("jq6500: play track 1");
            }
            break;

        default:
            // 数字字符直接映射为曲目播放
            if (cmd >= '0' && cmd <= '9')
            {
                uint8_t track_index = cmd - '0';
                if(track_index < 1) track_index = 1;
                jq6500_play_by_index(track_index);
                uart_log_string_data("jq6500: play track ");
                uart_log_number(track_index);
            }
            break;
    }
}

/************************ 模块接口注册 ************************/
const struct module_command_executor audio_executor = {init_jq6500_state, reset_jq6500_state, update_jq6500_state};