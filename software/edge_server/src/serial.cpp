#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// STM32 串口 (UART1): GPIO20=RX，GPIO19=TX
HardwareSerial SerialSTM32(1);

// UART2：ESP ↔ 激光雷达（GPIO2=RX，GPIO1=TX）
HardwareSerial SerialLiDAR(2);

// 控制任务和 WiFi 事件可能同时下发命令，串口写入加互斥
static SemaphoreHandle_t serialMutex = nullptr;

void initSerialPorts() {
  serialMutex = xSemaphoreCreateMutex();

  // 初始化 STM32 串口：波特率 9600，GPIO20=RX，GPIO19=TX
  SerialSTM32.begin(9600, SERIAL_8N1, 20, 19);
  Serial.println("STM32 Serial (GPIO19 TX, GPIO20 RX) initialized");

  // 初始化激光雷达串口：波特率 9600，GPIO2=RX，GPIO1=TX
  SerialLiDAR.begin(9600, SERIAL_8N1, 2, 1);
  Serial.println("LiDAR Serial (TX=GPIO1, RX=GPIO2) init OK");
}

static bool lockSerial(TickType_t timeout) {
  if (serialMutex == nullptr) {
    return true;
  }
  return xSemaphoreTake(serialMutex, timeout) == pdTRUE;
}

static void unlockSerial() {
  if (serialMutex != nullptr) {
    xSemaphoreGive(serialMutex);
  }
}

void sendToSTM32(String command) {
  // 20ms 超时：不能因为串口互斥把控制任务卡死
  if (!lockSerial(pdMS_TO_TICKS(20))) {
    Serial.println("STM32 serial busy, command dropped");
    return;
  }
  // 串口调试
  // Serial.println(command); 
  SerialSTM32.print(command);
  SerialSTM32.write('\n');
  unlockSerial();
}

void sendToLiDAR(String command) {
  if (!lockSerial(pdMS_TO_TICKS(20))) {
    return;
  }
  SerialLiDAR.print(command);
  SerialLiDAR.write('\n');
  unlockSerial();
}
