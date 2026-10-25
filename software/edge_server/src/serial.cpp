#include "serial.h"

// STM32 串口 (UART1): GPIO20=RX，GPIO19=TX
HardwareSerial SerialSTM32(1);

// UART2：ESP ↔ 激光雷达（新分配：GPIO2=RX，GPIO1=TX）
HardwareSerial SerialLiDAR(2);

void initSerialPorts() {
  // 初始化 STM32 串口：波特率 9600，GPIO20=RX，GPIO19=TX
  SerialSTM32.begin(9600, SERIAL_8N1, 20, 19);
  Serial.println("STM32 Serial (GPIO19 TX, GPIO20 RX) initialized");


  // 初始化激光雷达串口：波特率 9600，GPIO2=RX，GPIO1=TX
  SerialLiDAR.begin(9600, SERIAL_8N1, 2, 1);
  Serial.println("LiDAR Serial (TX=GPIO1, RX=GPIO2) init OK");
}

void sendToSTM32(String command) {
  SerialSTM32.print(command);
  SerialSTM32.write('\n');
}

void sendToLiDAR(String command) {
  SerialLiDAR.print(command);
  SerialLiDAR.write('\n');
}