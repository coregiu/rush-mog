/**
  ******************************************************************************
  *
  * Define of JQ6500 MP3 module executor.
  * Controls JQ6500 via USART2 (already initialized by audio_receiver).
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef EXECUTOR_JQ6500_H
#define EXECUTOR_JQ6500_H

#include "command.h"

// JQ6500 命令定义
#define JQ_CMD_NEXT        0x01   // 下一曲
#define JQ_CMD_PREV        0x02   // 上一曲
#define JQ_CMD_VOL_UP      0x03   // 音量+
#define JQ_CMD_VOL_DOWN    0x04   // 音量-
#define JQ_CMD_SET_VOL     0x06   // 设置音量 (0~30)
#define JQ_CMD_PLAY_PAUSE  0x0E   // 播放/暂停
#define JQ_CMD_PLAY_INDEX  0x16   // 指定曲目播放 (0~2999)
#define JQ_CMD_PLAY        0x0D   // 指定播放模式
#define JQ_CMD_STOP        0x0F   // 停止播放

extern const struct module_command_executor audio_executor;

#endif