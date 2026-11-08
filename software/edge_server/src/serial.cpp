#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <string.h>

// STM32 串口 (UART1): GPIO20=RX，GPIO19=TX
HardwareSerial SerialSTM32(1);

// UART2：ESP ↔ 激光雷达（GPIO2=RX，GPIO1=TX）
HardwareSerial SerialLiDAR(2);

// 控制任务和 WiFi 事件可能同时下发命令，串口写入加互斥
static SemaphoreHandle_t serialMutex = nullptr;

// STM32 编码器遥测缓存；控制任务写、HTTP 任务读。
static BodyTelemetry bodyTelemetry = {};
static portMUX_TYPE bodyTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
static char stm32Line[96];
static size_t stm32LineLength = 0;

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

// 解析 STM32 发来的 ASCII 遥测行：
//   $B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>
static void parseStm32Telemetry(const char *line) {
  long seq = 0;
  long uptimeMs = 0;
  long wheel[4] = {0, 0, 0, 0};
  long distanceMm = 0;

  int matched = sscanf(line, "$B,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                       &seq, &uptimeMs,
                       &wheel[0], &wheel[1], &wheel[2], &wheel[3],
                       &distanceMm);
  if (matched != 7) {
    return;
  }

  // 简单的物理范围校验，避免电机噪声或半帧数据造成异常跳变。
  for (int i = 0; i < 4; ++i) {
    if (wheel[i] < -5000 || wheel[i] > 5000) {
      return;
    }
  }
  if (seq < 0 || uptimeMs < 0 || distanceMm < 0 || distanceMm > 2000000000L) {
    return;
  }

  portENTER_CRITICAL(&bodyTelemetryMux);
  bodyTelemetry.valid = true;
  bodyTelemetry.seq = (uint32_t)seq;
  bodyTelemetry.uptimeMs = (uint32_t)uptimeMs;
  bodyTelemetry.wheelMmS[0] = (int32_t)wheel[0];
  bodyTelemetry.wheelMmS[1] = (int32_t)wheel[1];
  bodyTelemetry.wheelMmS[2] = (int32_t)wheel[2];
  bodyTelemetry.wheelMmS[3] = (int32_t)wheel[3];
  bodyTelemetry.distanceMm = (int32_t)distanceMm;
  bodyTelemetry.receivedMs = millis();
  portEXIT_CRITICAL(&bodyTelemetryMux);
}

void serviceSTM32Telemetry() {
  while (SerialSTM32.available() > 0) {
    int value = SerialSTM32.read();
    if (value < 0) {
      break;
    }
    char ch = (char)value;
    if (ch == '\n' || ch == '\r') {
      if (stm32LineLength > 0) {
        stm32Line[stm32LineLength] = '\0';
        if (strncmp(stm32Line, "$B,", 3) == 0) {
          parseStm32Telemetry(stm32Line);
        }
        stm32LineLength = 0;
      }
    } else if (stm32LineLength < sizeof(stm32Line) - 1) {
      stm32Line[stm32LineLength++] = ch;
    } else {
      // 超长行，丢弃并重新同步到下一个换行。
      stm32LineLength = 0;
    }
  }
}

bool getSTM32Telemetry(BodyTelemetry *out) {
  if (out == nullptr) {
    return false;
  }
  portENTER_CRITICAL(&bodyTelemetryMux);
  *out = bodyTelemetry;
  portEXIT_CRITICAL(&bodyTelemetryMux);
  return out->valid;
}
