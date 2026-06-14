#ifndef SERIAL_H
#define SERIAL_H

#include <HardwareSerial.h>

// STM32 串口: GPIO43=TX, GPIO44=RX
extern HardwareSerial SerialSTM32;
// 激光雷达串口: GPIO19=TX, GPIO20=RX
extern HardwareSerial SerialLiDAR;

void initSerialPorts();
void sendToSTM32(String command);
void sendToLiDAR(String command);

#endif // SERIAL_H