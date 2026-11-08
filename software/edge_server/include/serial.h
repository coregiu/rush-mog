#ifndef SERIAL_H
#define SERIAL_H

#include <HardwareSerial.h>

// STM32 串口: ESP32 TX=GPIO19, RX=GPIO20 (对端 STM32 USART3)
extern HardwareSerial SerialSTM32;
// 激光雷达串口: ESP32 TX=GPIO1, RX=GPIO2
extern HardwareSerial SerialLiDAR;

void initSerialPorts();
void sendToSTM32(String command);
void sendToLiDAR(String command);

// STM32 编码器遥测（USART3 -> ESP32 UART1 RX）。
// 帧格式：$B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>\n
// 轮速单位 mm/s，distance_mm 为四轮平均绝对里程。
struct BodyTelemetry
{
  bool valid;
  uint32_t seq;
  uint32_t uptimeMs;
  int32_t wheelMmS[4];
  int32_t distanceMm;
  uint32_t receivedMs;
};

// 在控制任务中周期调用，把串口字节拼成行并解析最新遥测。
void serviceSTM32Telemetry();

// 原子读取最新遥测；返回是否收到过有效帧。
bool getSTM32Telemetry(BodyTelemetry *out);

#endif // SERIAL_H