/**
  ******************************************************************************
  * TCRT5000 红外反射悬崖模块（左前、右前）。
  *
  * 接线（只使用模块的 DO 数字量输出）：
  *   VCC        -> 3.3V（推荐；PB3/PB4 为 5V 容忍引脚，5V 模块也可）
  *   GND        -> GND（必须与 STM32 共地）
  *   左前 DO    -> PB3
  *   右前 DO    -> PB4
  *
  * 说明：
  * - PB3/PB4 复位后默认是 JTAG 的 JTDO/NJTRST。cliff_sensor_init() 会关闭
  *   JTAG、保留 SWD，所以 ST-Link 仍可通过 PA13/PA14 下载调试，但不能再
  *   使用 JTAG 方式连接。
  * - PB3/PB4 不能做 EXTI 边沿中断：EXTI3/EXTI4 已被 PA3/PA4 编码器占用。
  *   这里用 10ms 轮询 + 3 次消抖，从电平变化到停车最多约 40ms。
  * - TCRT5000 的 DO 是电位器阈值比较后的开关量，不能测距。要得到约
  *   10cm 的停车余量，应把探头朝前下方倾斜，让红外光斑落在车头前方约
  *   10cm 的地面上，再用模块电位器把该距离调到刚好触发；光斑越过桌沿
  *   时模块翻转，固件读到触发后立刻停车。
  * - 常见 LM393/TCRT5000 模块：有反射（桌面/地面）时 DO 为低电平，
  *   悬空（悬崖）时 DO 为高电平。默认 CLIFF_*_TRIGGER_LEVEL = 1，按
  *   悬崖模式配置（无反射/悬空=高=触发）。如果模块空闲电平相反
  *   （放在桌面上 DO 就是高），把对应宏改成 0；两个模块可分别配置。
  ******************************************************************************
**/

#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include "command.h"

/* DO 触发电平：1 = 高电平表示悬崖/障碍，0 = 低电平表示悬崖/障碍。 */
#define CLIFF_LEFT_TRIGGER_LEVEL  1
#define CLIFF_RIGHT_TRIGGER_LEVEL 1

/* 轮询周期和消抖次数（连续 N 次一致才更新状态）。 */
#define CLIFF_POLL_PERIOD_MS   10
#define CLIFF_DEBOUNCE_SAMPLES 3

/* 状态位，便于日志或上层状态展示。 */
#define CLIFF_LEFT_MASK  0x01
#define CLIFF_RIGHT_MASK 0x02
#define CLIFF_ANY_MASK   (CLIFF_LEFT_MASK | CLIFF_RIGHT_MASK)

/**
 * 初始化 PB3/PB4 输入和悬崖监控任务。
 * 必须在 FreeRTOS 调度器启动前调用。
 */
void cliff_sensor_init(void);

/**
 * 返回当前稳定状态：CLIFF_LEFT_MASK / CLIFF_RIGHT_MASK 的按位或，
 * 0 表示两只传感器都没有触发。
 */
uint8_t cliff_sensor_get_mask(void);

/**
 * 悬崖触发时的运动安全策略：
 * 只允许停车（'0'）和后退（'2'），其余运动命令由上层改成停车。
 */
uint8_t cliff_sensor_command_allowed(uchar command);

#endif /* CLIFF_SENSOR_H */
