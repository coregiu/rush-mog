#include "serial.h"

SoftwareSerial softwareSerial(D3, D4);  

void initSoftwareSerial() {
  softwareSerial.begin(9600); // 设置波特率
  Serial.println("Software Serial Initialized");
}

void sendCommands(String command){
    softwareSerial.println(command); // 发送命令到STM32
}
