/**
  ******************************************************************************
  *
  * Define of audio receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef RECEIVER_AUDIO_H
#define RECEIVER_AUDIO_H

#include "command.h"

// ASR-Pro 语音板与 JQ6500 MP3 共用 USART2；当前方案已改用网页语音，
// 默认关闭 ASR 接收，避免 PA3 浮空噪声被当成运动命令。
// 如果仍然接着 ASR-Pro，把这里改成 1。
#ifndef USE_ASR_RECEIVER
#define USE_ASR_RECEIVER 0
#endif

extern const struct module_command_receiver audio_receiver;

#endif
