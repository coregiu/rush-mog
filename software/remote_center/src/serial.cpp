#include "serial.h"

// STM32 串口 (UART1): GPIO17=TX, GPIO18=RX
HardwareSerial SerialSTM32(1);
// 激光雷达串口 (UART2): GPIO19=TX, GPIO20=RX
HardwareSerial SerialLiDAR(2);

void initSerialPorts() {
  // 初始化 STM32 串口：波特率 9600，GPIO17=TX，GPIO18=RX
  SerialSTM32.begin(9600, SERIAL_8N1, 18, 17);
  Serial.println("STM32 Serial (GPIO17 TX, GPIO18 RX) initialized");

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