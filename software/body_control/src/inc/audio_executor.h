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


/************************ JQ6500 串口命令宏定义 ************************/
#define JQ_CMD_NEXT            0x01    // 下一曲
#define JQ_CMD_PREV            0x02    // 上一曲
#define JQ_CMD_PLAY_INDEX      0x03    // 按索引播放曲目
#define JQ_CMD_VOL_UP          0x04    // 音量加
#define JQ_CMD_VOL_DOWN        0x05    // 音量减
#define JQ_CMD_SET_VOL         0x06    // 直接设置音量 0~30
#define JQ_CMD_STOP            0x0B    // 停止播放
#define JQ_CMD_PLAY            0x0D    // 播放/继续
#define JQ_CMD_PAUSE           0x0E    // 暂停
#define JQ_CMD_SET_PLAY_MODE   0x11    // 设置播放模式

extern const struct module_command_executor audio_executor;

#endif