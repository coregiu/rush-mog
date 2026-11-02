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

#endif // SERIAL_H