#include "serial.h"

// STM32 串口 (UART1): GPIO43=TX, GPIO44=RX
HardwareSerial SerialSTM32(1);
// 激光雷达串口 (UART2): GPIO19=TX, GPIO20=RX
HardwareSerial SerialLiDAR(2);

void initSerialPorts() {
  // 初始化 STM32 串口：波特率 9600，GPIO43=TX，GPIO44=RX
  SerialSTM32.begin(9600, SERIAL_8N1, 44, 43);
  Serial.println("STM32 Serial (GPIO43 TX, GPIO44 RX) initialized");

  // 发送测试命令验证串口通信
  // SerialSTM32.print("0");
  // SerialSTM32.write('\n');
  // Serial.println("Sent test command '0' to STM32");

  // 初始化激光雷达串口：波特率 115200，GPIO19=TX，GPIO20=RX
  // LiDAR 常用波特率 115200，如有需要可调整
  SerialLiDAR.begin(9600, SERIAL_8N1, 20, 19);
  Serial.println("LiDAR Serial (GPIO19 TX, GPIO20 RX) initialized");
}

void sendToSTM32(String command) {
  SerialSTM32.print(command);
  SerialSTM32.write('\n');
}

void sendToLiDAR(String command) {
  SerialLiDAR.print(command);
  SerialLiDAR.write('\n');
}