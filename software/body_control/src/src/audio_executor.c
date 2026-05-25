/**
  ******************************************************************************
  *
  * Implement of JQ6500 MP3 module executor.
  * Controls JQ6500 via USART2 (shared with audio_receiver on PA2 TX).
  *
  * JQ6500 Serial Protocol:
  *   Frame format: 7E FF [len] [cmd] [data...] EF
  *   - len: number of bytes after len (cmd + data)
  *   - cmd: command byte
  *   - data: optional data bytes
  *   - All values in hex
  *
  * author: coregiu
  *
  ******************************************************************************
**/

#include <audio_executor.h>
#include "controller.h"

/* 通过 USART2 发送一个字节（阻塞轮询方式） */
static void uart2_send_byte(uint8_t data)
{
    USART_SendData(USART2, data);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_ClearFlag(USART2, USART_FLAG_TC);
}

/* JQ6500 帧尾 */
static void jq6500_send_frame(uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    uint8_t i;

    // 帧头
    uart2_send_byte(0x7E);
    uart2_send_byte(0xFF);

    // 长度 (cmd字节 + data字节数)
    uart2_send_byte(1 + data_len);

    // 命令字节
    uart2_send_byte(cmd);

    // 数据字节
    for (i = 0; i < data_len; i++)
    {
        uart2_send_byte(data[i]);
    }

    // 帧尾
    uart2_send_byte(0xEF);
}

/* 发送无数据命令（下一曲、上一曲、音量加减等） */
static void jq6500_send_simple_cmd(uint8_t cmd)
{
    jq6500_send_frame(cmd, 0, NULL);
}

/* 设置音量 0~30 */
static void jq6500_set_volume(uint8_t volume)
{
    if (volume > 30) volume = 30;
    uint8_t vol_data[1] = {volume};
    jq6500_send_frame(JQ_CMD_SET_VOL, 1, vol_data);
}

/* 播放指定曲目 (0~2999) */
static void jq6500_play_by_index(uint16_t index)
{
    // JQ6500 uses little-endian for index: 7E FF 03 16 [lo] [hi] EF
    uint8_t index_data[2] = {(uint8_t)(index & 0xFF), (uint8_t)((index >> 8) & 0xFF)};
    jq6500_send_frame(JQ_CMD_PLAY_INDEX, 2, index_data);
}

void init_jq6500_state(void)
{
    // USART2 TX 引脚为 PA2，已在 audio_receiver 中初始化
    // 此处只需做必要延迟，等待 JQ6500 模块上电稳定
    uart_log_string_data("jq6500 executor init");
    delay_ms(200);

    // 初始化音量到中等水平 (30)
    jq6500_set_volume(30); // 设置为最大音量，避免初始状态过小听不清
    jq6500_play_by_index(0); // 播放第一首作为重置状态
}

void reset_jq6500_state(void)
{
    jq6500_play_by_index(0); // 播放第一首作为重置状态
}

void update_jq6500_state(struct command_context *command_context)
{
    uchar cmd = command_context->commands[0];

    switch (cmd)
    {
    case COMMAND_AUDIO_NEXT:
        // 下一曲
        jq6500_send_simple_cmd(JQ_CMD_NEXT);
        uart_log_string_data("jq6500: next track");
        break;

    case COMMAND_AUDIO_PREV:
        // 上一曲
        jq6500_send_simple_cmd(JQ_CMD_PREV);
        uart_log_string_data("jq6500: prev track");
        break;

    case COMMAND_AUDIO_VOL_UP:
        // 音量+
        jq6500_send_simple_cmd(JQ_CMD_VOL_UP);
        uart_log_string_data("jq6500: vol up");
        break;

    case COMMAND_AUDIO_VOL_DOWN:
        // 音量-
        jq6500_send_simple_cmd(JQ_CMD_VOL_DOWN);
        uart_log_string_data("jq6500: vol down");
        break;

    case COMMAND_AUDIO_STOP:
        // 停止播放
        jq6500_send_simple_cmd(JQ_CMD_STOP);
        uart_log_string_data("jq6500: stop");
        break;

    case COMMAND_AUDIO_PLAY:
        // 播放指定曲目 - commands[1] 存放曲目索引（ASCII数字）
        if (command_context->cmd_length >= 2)
        {
            // 支持 ASCII 数字 '0'~'9' 转为曲目索引
            uint8_t track_index = command_context->commands[1] - '0';
            jq6500_play_by_index(track_index);
            uart_log_string_data("jq6500: play track ");
            uart_log_number(track_index);
        }
        else
        {
            // 默认播放第一首
            jq6500_play_by_index(1);
            uart_log_string_data("jq6500: play track 1");
        }
        break;

    default:
        // 如果收到的是曲目数字命令（'0'~'9'），也当作播放指定曲目处理
        if (cmd >= '0' && cmd <= '9')
        {
            uint8_t track_index = cmd - '0';
            // 曲目从1开始，但JQ6500索引0表示第一首，所以直接使用
            jq6500_play_by_index(track_index);
            uart_log_string_data("jq6500: play track ");
            uart_log_number(track_index);
        }
        break;
    }
}

const struct module_command_executor audio_executor = {init_jq6500_state, reset_jq6500_state, update_jq6500_state};
