/**
  ******************************************************************************
  * JGB37-520 四路增量式编码器速度/里程感知。
  *
  * 引脚分配（每路 A/B 两相，软件四倍频）：
  *   FL 左前：PA0 (A) / PA1 (B)
  *   FR 右前：PA2 (A) / PA3 (B)
  *   RL 左后：PA4 (A) / PA5 (B)
  *   RR 右后：PA6 (A) / PA7 (B)
  *
  * 说明：
  * - PA0~PA7 在 STM32F103 上属于 TC 类型，**不是 5V 容忍(FT)引脚**。
  *   编码器若要直接用 5V 供电，A/B 相必须分压/电平转换到 3.3V，否则会通过
  *   保护二极管倒灌，轻则计数异常，重则损坏 IO；最省事的做法是编码器用 3.3V 供电。
  * - PA2/PA3 同时是 USART2 的 TX/RX（语音板 ASR PRO 与 JQ6500 MP3）。
  *   编码器占用后，这两脚的 UART 功能失效：语音命令收不到、MP3 不出声。
  *   若仍需语音功能，请把这路编码器换到 PB3/PB4 等空闲脚。
  * - 编码器 EXTI 的抢占优先级固定低于 USART2/USART3，避免边沿中断挤掉
  *   USART3(命令接收) 的收数中断。
  * - 编码器接口使用内部上拉，若编码器为开漏输出且线缆较长，建议外部再加
  *   4.7k~10k 上拉到 3.3V；
  * - 通过 USART3(PB10 TX / PB11 RX) 向 ESP32 上报 ASCII 遥测，帧格式：
  *       $B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>\n
  *   其中四个轮速单位为 mm/s（有符号），distance_mm 为四轮平均绝对里程。
  ******************************************************************************
**/

#ifndef ENCODER_SENSOR_H
#define ENCODER_SENSOR_H

#include "command.h"

/* 编码器每输出轴一圈的计数（四倍频后）。JGB37-520 常见 11PPR、1:30。 */
#define ENCODER_CPR 1320

/* 轮子直径 125mm（半径 62.5mm，与 ROS 底盘参数一致）。 */
#define ENCODER_WHEEL_DIAMETER_MM 125

/* 每个计数对应的里程，单位 0.1um：pi * 直径 / CPR * 10000。
 * 125mm 轮、1320 CPR 时约为 2945。 */
#define ENCODER_UM_X10_PER_COUNT \
    ((ENCODER_WHEEL_DIAMETER_MM * 31416 + ENCODER_CPR / 2) / ENCODER_CPR)

/* 遥测周期；实际周期还会叠加 115200 波特率下的发送时间（约 4~5ms）。 */
#define ENCODER_TELEMETRY_PERIOD_MS 50

/* 如果某个轮子前进时计数为负，把对应符号改成 -1。 */
#define ENCODER_SIGN_FL 1
#define ENCODER_SIGN_FR 1
#define ENCODER_SIGN_RL 1
#define ENCODER_SIGN_RR 1

#define ENCODER_COUNT 4

/**
 * 初始化 8 个编码器 GPIO、EXTI 中断和遥测任务。
 * 必须在 USART3 初始化之后、FreeRTOS 调度器启动之前调用。
 */
void encoder_sensor_init(void);

/**
 * 原子读取四路累计计数（任务上下文使用）。
 */
void encoder_sensor_get_counts(int32_t counts[ENCODER_COUNT]);

#endif /* ENCODER_SENSOR_H */
