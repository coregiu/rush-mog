#include "web.h"
#include "camera.h"
#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Esp.h>
#include <stdio.h>
#include <string.h>

IPAddress local_IP(192, 168, 4, 1); // 本地IP
IPAddress gateway(192, 168, 4, 1);  // 网关
IPAddress subnet(255, 255, 255, 0); // 子网掩码

WebServer server(80);
WebSocketsServer webSocket(WS_PORT);

bool sdcardReady = false;

// WebSocket 客户端计数由事件回调维护，避免 HTTP 任务直接遍历 WS 内部客户端列表
static volatile int wsClientCount = 0;

// 初始化 LittleFS
bool initFS()
{
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS 挂载失败！");
    return false;
  }
  Serial.println("LittleFS 挂载成功");
  return true;
}

// 初始化 SD 卡
bool initSdcard() {
  // ESP32 Arduino SD_MMC 的参数顺序是：
  //   setPins(clk, cmd, d0, d1, d2, d3)
  // 1-bit 模式只使用 clk/cmd/d0，d1~d3 传 -1 也可以。
  // 注意：这里的数字必须和实际原理图一致，改之前先核对板子。
  SD_MMC.setPins(
    39,  // clk
    38,  // cmd
    40,  // d0
    41,  // d1 (1-bit 模式未使用)
    42,  // d2 (1-bit 模式未使用)
    11   // d3 (1-bit 模式未使用)
  );

  // 注意：2.0.x 的 begin 参数是
  //   begin(mountpoint, mode1bit, format_if_mount_failed,
  //         sdmmc_frequency_khz, maxOpenFiles)
  // 第 4 个不是 maxOpenFiles。这里用 20MHz（较稳）并允许 20 个文件句柄。
  if (!SD_MMC.begin("/sdcard", true, false, 20000, 20)) { // true=1-bit模式
    Serial.println("SD卡挂载失败！");
    sdcardReady = false;
    return false;
  }
  Serial.println("SD卡挂载成功！");
  sdcardReady = true;
  return true;
}

String contentTypeFromPath(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css")) return "text/css";
  if (path.endsWith(".js")) return "application/javascript";
  if (path.endsWith(".json")) return "application/json";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  if (path.endsWith(".svg")) return "image/svg+xml";
  if (path.endsWith(".ico")) return "image/x-icon";
  if (path.endsWith(".wasm")) return "application/wasm";
  if (path.endsWith(".woff2")) return "font/woff2";
  if (path.endsWith(".bin")) return "application/octet-stream";
  return "application/octet-stream";
}

// 网页请求处理
void handleWebRequest()
{
  String path = server.uri();
  if (path == "/") {
    path = "/index.html";
  }

  if (path.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  String contentType = contentTypeFromPath(path);

  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "File Not Found - index.html!");
    return;
  }

  File file = LittleFS.open(path, "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  // 必须在发送响应体之前设置，否则对 streamFile 无效。
  // index.html 不缓存，避免升级后手机还拿旧页面；静态资源长缓存。
  if (path == "/index.html") {
    server.sendHeader("Cache-Control", "no-cache");
  } else {
    server.sendHeader("Cache-Control", "public, max-age=36000");
  }
  // 注意：ESP32 Arduino 2.0.x 的 streamFile() 第三个参数是 HTTP 状态码，
  // 不是 Content-Length；传 file.size() 会让状态行变成 "HTTP/1.1 26566"。
  // Content-Length 由 streamFile() 内部用 file.size() 自己设置。
  server.streamFile(file, contentType);
  file.close();
}

void handleSdcard() {
  if (!sdcardReady) {
    server.send(503, "text/plain", "SD card not mounted");
    return;
  }

  String url = server.uri();          // 例如: /sdcard/tf.min.js
  String path = url.substring(8);     // 去掉 /sdcard/ → tf.min.js
  if (path.length() == 0 || path.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  String fullPath = "/" + path;
  String contentType = contentTypeFromPath(path);

  if (!SD_MMC.exists(fullPath)) {
    server.send(404, "text/plain", "Sdcard Not Found The File");
    return;
  }

  File file = SD_MMC.open(fullPath, "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  // streamFile 会自己写 Content-Length 并分块发送，比手写 512B 分块快很多
  server.sendHeader("Cache-Control", "public, max-age=25920000, immutable");
  server.streamFile(file, contentType);
  file.close();
}

// 按钮命令处理（HTTP 与 WebSocket 共用）
void processCmdButton(const String& key, const String& type) {
  String command;

  // 语音“停止”最终会走这里；没有这个分支，button:stop:down 会被忽略
  if (key == "stop") {
    command = "0";
  } else if (type == "up") { // 松开事件
    if (key == "triangle" || key == "cross" || key == "select" || key == "start") {

    } else if (key == "square" || key == "circle") {
      command = "I";
    } else {
      command = "0";
    }
  } else { // 按下事件
    if (key == "up") {
      command = "1";
    } else if (key == "down") {
      command = "2";
    } else if (key == "left") {
      command = "3";
    } else if (key == "right") {
      command = "4";
    } else if (key == "triangle") {
      command = "E";
    } else if (key == "cross") {
      command = "F";
    } else if (key == "square") {
      command = "G";
    } else if (key == "circle") {
      command = "H";
    } else if (key == "l1") {
      command = "9";
    } else if (key == "r1") {
      command = "A";
    } else if (key == "l2") {
      command = "B";
    } else if (key == "r2") {
      command = "C";
    } else if (key == "start") {
      command = "D";
    } else if (key == "select") {
      command = "J";
    }
  }

  if (!command.isEmpty()) {
    sendToSTM32(command);
  }
}

void handleCmdButton() {
  String key = server.arg("key");
  String type = server.arg("type");
  if (key.isEmpty() || type.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'key' or 'type' parameter");
    return;
  }
  processCmdButton(key, type);
  server.send(200);
}

// 摇杆命令处理（HTTP 与 WebSocket 共用）
void processCmdStick(const String& stickId, const String& direct, const String& stepValue) {
  String command;

  if (stickId != "left-analog") {
    return;
  }

  // 只接受 0~9 单字符，防止客户端传入超长/非法参数
  if (stepValue.length() != 1 || stepValue[0] < '0' || stepValue[0] > '9') {
    return;
  }

  if (direct == "stop" || stepValue == "0") {
    command = "0";
  } else {
    if (direct == "up") {
      command = "1" + stepValue;
    } else if (direct == "down") {
      command = "2" + stepValue;
    } else if (direct == "left") {
      command = "3" + stepValue;
    } else if (direct == "right") {
      command = "4" + stepValue;
    } else if (direct == "uple") {
      command = "5" + stepValue;
    } else if (direct == "uprt") {
      command = "6" + stepValue;
    } else if (direct == "dnle") {
      command = "7" + stepValue;
    } else if (direct == "dnrt") {
      command = "8" + stepValue;
    }
  }

  if (!command.isEmpty()) {
    sendToSTM32(command);
  }
}

void handleCmdStick() {
  String stickId = server.arg("stickId");
  String direct = server.arg("direct");
  String stepValue = server.arg("stepValue");
  if (stickId.isEmpty() || direct.isEmpty() || stepValue.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'stickId', 'direct' or 'stepValue' parameter");
    return;
  }
  processCmdStick(stickId, direct, stepValue);
  server.send(200);
}

// 显式停车接口：浏览器 pagehide/断线兜底可以用
void handleStop() {
  sendToSTM32("0");
  server.send(200, "text/plain", "stopped");
}

void handleStatus() {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"uptime_ms\":%lu,\"free_heap\":%u,\"free_psram\":%u,"
           "\"camera\":%u,\"sdcard\":%u,\"ws_clients\":%d}",
           (unsigned long)millis(),
           (unsigned)ESP.getFreeHeap(),
           (unsigned)ESP.getFreePsram(),
           cameraReady ? 1U : 0U,
           sdcardReady ? 1U : 0U,
           (int)wsClientCount);
  server.send(200, "application/json", String(json));
}

// WebSocket 事件回调
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[WS] Client #%u connected from %s\n", num, ip.toString().c_str());
      wsClientCount++;
      // 新连接先置停止，避免沿用上一次的残留运动状态
      sendToSTM32("0");
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      if (wsClientCount > 0) {
        wsClientCount--;
      }
      // 最后一个控制端断开时立即停车；多客户端时避免误停另一个客户端
      if (wsClientCount == 0) {
        sendToSTM32("0");
      }
      break;
    case WStype_TEXT: {
      if (payload == nullptr || length == 0) {
        break;
      }
      // payload 不保证以 \0 结尾，必须按 length 构造
      String msg((const char*)payload, length);

      // 消息格式：
      //   button:<key>:<type>         例如 button:up:down
      //   stick:<stickId>:<direct>:<stepValue>  例如 stick:left-analog:up:5
      if (msg.startsWith("button:")) {
        int first = msg.indexOf(':', 7);
        if (first > 0) {
          String key = msg.substring(7, first);
          String eventType = msg.substring(first + 1);
          processCmdButton(key, eventType);
        }
      } else if (msg.startsWith("stick:")) {
        int first = msg.indexOf(':', 6);
        int second = msg.indexOf(':', first + 1);
        if (first > 0 && second > first) {
          String stickId = msg.substring(6, first);
          String direct = msg.substring(first + 1, second);
          String stepValue = msg.substring(second + 1);
          processCmdStick(stickId, direct, stepValue);
        }
      }
      break;
    }
    default:
      break;
  }
}

// 读取并丢弃 HTTP 请求头，最多等 300ms
static bool readHttpRequest(WiFiClient& client) {
  uint32_t start = millis();
  int state = 0; // 0:正常, 1:\r, 2:\r\n, 3:\r\n\r
  while (millis() - start < 300) {
    while (client.available()) {
      int ch = client.read();
      if (ch < 0) {
        break;
      }
      if (state == 0 && ch == '\r') {
        state = 1;
      } else if (state == 1 && ch == '\n') {
        state = 2;
      } else if (state == 2 && ch == '\r') {
        state = 3;
      } else if (state == 3 && ch == '\n') {
        return true;
      } else {
        state = (ch == '\r') ? 1 : 0;
      }
    }
    if (!client.connected()) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

static bool writeAll(WiFiClient& client, const uint8_t* data, size_t len) {
  size_t offset = 0;
  while (offset < len) {
    size_t sent = client.write(data + offset, len - offset);
    if (sent == 0) {
      return false;
    }
    offset += sent;
  }
  return true;
}

static const char STREAM_RESPONSE_HEADER[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-cache, no-store, must-revalidate\r\n"
    "Pragma: no-cache\r\n"
    "Connection: close\r\n\r\n";

static const char STREAM_UNAVAILABLE[] =
    "HTTP/1.1 503 Service Unavailable\r\n"
    "Content-Type: text/plain\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n\r\n";

// 摄像头流任务：端口 8080，独立于 HTTP 80 和 WS 81
void cameraStreamTask(void* arg) {
  (void)arg;

  WiFiServer streamServer(STREAM_PORT);
  streamServer.begin();
  Serial.printf("MJPEG 视频流已启动，端口：%d\n", STREAM_PORT);

  WiFiClient client;
  TickType_t lastFrame = 0;
  int failCount = 0;
  const TickType_t frameInterval = pdMS_TO_TICKS(100); // 10 FPS

  for (;;) {
    // 新连接到来时接管；旧连接直接关闭，避免多路流抢帧
    WiFiClient incoming = streamServer.available();
    if (incoming) {
      if (client) {
        client.stop();
      }
      client = incoming;
      client.setNoDelay(true);
      failCount = 0;

      if (!readHttpRequest(client)) {
        client.stop();
        client = WiFiClient();
        continue;
      }

      if (!cameraReady) {
        writeAll(client, (const uint8_t*)STREAM_UNAVAILABLE, strlen(STREAM_UNAVAILABLE));
        client.stop();
        client = WiFiClient();
        continue;
      }

      if (!writeAll(client, (const uint8_t*)STREAM_RESPONSE_HEADER, strlen(STREAM_RESPONSE_HEADER))) {
        client.stop();
        client = WiFiClient();
        continue;
      }

      // 新连接立刻送第一帧
      lastFrame = xTaskGetTickCount() - frameInterval;
    }

    if (!client || !client.connected()) {
      if (client) {
        client.stop();
        client = WiFiClient();
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    TickType_t now = xTaskGetTickCount();
    if (now - lastFrame < frameInterval) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    lastFrame = now;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      failCount++;
      // 连续失败就断开，让浏览器重连；不要每 4 秒卡住整个任务
      if (failCount >= 5) {
        Serial.println("摄像头连续取帧失败，关闭当前视频流");
        client.stop();
        client = WiFiClient();
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    failCount = 0;

    char frameHeader[128];
    int headerLen = snprintf(frameHeader, sizeof(frameHeader),
                             "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                             (unsigned)fb->len);

    bool ok = headerLen > 0 &&
              writeAll(client, (const uint8_t*)frameHeader, (size_t)headerLen) &&
              writeAll(client, fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (ok) {
      ok = writeAll(client, (const uint8_t*)"\r\n", 2);
    }

    // 部分写说明 TCP 已经跟不上，继续发只会把 MJPEG 流写坏
    if (!ok) {
      client.stop();
      client = WiFiClient();
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
