#include "serial.h"

// D3 = RX（接收脚）; D4 = TX（发送脚）
// SoftwareSerial softwareSerial(D3, D4);  
HardwareSerial softwareSerial(1); // 使用硬件串口1

void initSoftwareSerial() {
  softwareSerial.begin(9600, SERIAL_8N1, 1, 2);
  Serial.println("Software Serial Initialized");
}

void sendCommands(String command){
    softwareSerial.print(command); // 发送命令到STM32
    softwareSerial.write('\n');  // 发送换行符，表示命令结束
}
