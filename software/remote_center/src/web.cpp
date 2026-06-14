#include "web.h"
#include "camera.h"
#include "serial.h"
#include <esp_heap_caps.h>

IPAddress local_IP(192, 168, 4, 1); // 本地IP
IPAddress gateway(192, 168, 4, 1);  // 网关
IPAddress subnet(255, 255, 255, 0); // 子网掩码

WebServer server(80);

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
  // --------------------------
  // ESP32-S3-CAM 必须这样写！
  // --------------------------
  SD_MMC.setPins(
    39,  // D0
    38,  // D1  (不用时可设 -1)
    40,  // D2  (不用时可设 -1)
    41,  // D3  (不用时可设 -1)
    42,  // CLK
    11   // CMD
  );

  if (!SD_MMC.begin("/sdcard", true, false, 20)) { // true=1-bit模式（S3-CAM板载专用）
    Serial.println("SD卡挂载失败！");
    return false;
  }
  Serial.println("SD卡挂载成功！");
  return true;
}

String contentTypeFromPath(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css")) return "text/css";
  if (path.endsWith(".js")) return "application/javascript";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  if (path.endsWith(".ico")) return "image/x-icon";
  return "text/plain"; // 默认类型
}

// 网页请求处理
void handleWebRequest()
{
  String path = server.uri();
  if (path == "/") {
    path = "/index.html";
  }

  // 自动判断文件类型
  String contentType = contentTypeFromPath(path);
  
  // 读取并返回文件
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "File Not Found - index.html!");
  }
}

void handleSdcard() {
  String url = server.uri();          // 例如: /sdcard/tf.min.js
  String path = url.substring(8);     // 去掉 /sdcard/ → tf.min.js
  String fullPath = "/" + path;       // → /tf.min.js

  // 自动判断文件类型
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

  WiFiClient client = server.client();

  // 发送 HTTP 响应头 + chunked 编码标记
  // Cache-Control: 客户端缓存 300 天（25920000 秒）
  client.printf("HTTP/1.1 200 OK\r\n");
  client.printf("Content-Type: %s\r\n", contentType.c_str());
  client.printf("Cache-Control: public, max-age=25920000, immutable\r\n");
  client.printf("Transfer-Encoding: chunked\r\n");
  client.printf("Connection: close\r\n\r\n");

  // DMA 方式：从 PSRAM 分配大块 DMA 缓冲区（16KB），减少循环次数
  size_t bufSize = 512;  // 16KB DMA 缓冲区
  uint8_t* buffer = (uint8_t*)heap_caps_malloc(bufSize, MALLOC_CAP_DMA);

  if (!buffer) {
    server.send(500, "text/plain", "Failed to allocate DMA buffer");
    file.close();
    return;
  }

  size_t bytesRead;
  while ((bytesRead = file.read(buffer, bufSize)) > 0) {
    // 发送 chunk 大小（十六进制）
    client.printf("%x\r\n", bytesRead);
    // DMA 缓冲区数据直接写入 socket
    client.write(buffer, bytesRead);
    // 发送 chunk 结束标记（\r\n）
    client.printf("\r\n");
  }

  // 发送结束 chunk
  client.printf("0\r\n\r\n");

  free(buffer);
  file.close();
}

void handleCmdButton() {
  // 定义按钮
  String key = server.arg("key");
  // 定义事件类型：按下还是松开
  String type = server.arg("type");
  String command;
  if (key.isEmpty() || type.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'key' or 'type' parameter");
    return;
  }
  if (type == "up") { // 松开事件
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
    Serial.println(command); 
    sendToSTM32(command);
  }

  server.send(200);
}

void handleCmdStick() {
  String stickId = server.arg("stickId");
  String direct = server.arg("direct");
  String stepValue = server.arg("stepValue");
  String command;
  if (stickId.isEmpty() || direct.isEmpty() || stepValue.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'stickId', 'direct' or 'stepValue' parameter");
    return;
  }
  if (stickId != "left-analog") {
    server.send(200);
    return;
  }
  if (direct == "stop") {
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

  Serial.println(command);
  sendToSTM32(command);
  // Serial.println(stickId + " - Direct: " + direct + ", Step Value: " + stepValue); 
  server.send(200);
}

// 摄像头流处理 - MJPEG 流（DMA 优化版）
void handleCameraStream() {
  WiFiClient client = server.client();
  if (!client) {
    server.send(500, "text/plain", "Failed to get client");
    return;
  }

  // 发送 MJPEG 流响应头
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n";
  response += "Cache-Control: no-cache\r\n";
  response += "Connection: close\r\n\r\n";
  
  client.print(response);

  // 预分配 HTTP 头部模板（避免重复创建 String 对象）
  String frameHeader = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
  String frameTrailer = "\r\n\r\n";
  
  // 从 PSRAM 分配 DMA 发送缓冲区（用于分块发送大帧）
  const size_t dmaBufSize = 8192;  // 8KB DMA 发送缓冲区
  uint8_t* dmaBuf = (uint8_t*)heap_caps_malloc(dmaBufSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
  if (!dmaBuf) {
    dmaBuf = (uint8_t*)heap_caps_malloc(dmaBufSize, MALLOC_CAP_DMA);
  }

  unsigned long lastFrameTime = 0;
  const unsigned long frameInterval = 100; // 10FPS，可根据需要调整

  while (client.connected()) {
    unsigned long currentTime = millis();
    if (currentTime - lastFrameTime < frameInterval) {
      delay(10);
      continue;
    }
    lastFrameTime = currentTime;

    // 获取摄像头帧（DMA 已由 esp32-camera 驱动内部处理）
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to get camera frame");
      delay(100);
      continue;
    }

    // 发送 JPEG 帧头
    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %zu\r\n", fb->len);
    client.print("\r\n");
    
    // DMA 方式发送帧数据：如果帧较大则分块写入，避免长阻塞
    if (dmaBuf) {
      size_t remaining = fb->len;
      size_t offset = 0;
      while (remaining > 0) {
        size_t chunk = (remaining > dmaBufSize) ? dmaBufSize : remaining;
        memcpy(dmaBuf, fb->buf + offset, chunk);
        size_t sent = client.write(dmaBuf, chunk);
        if (sent != chunk) {
          Serial.printf("DMA send failed: sent %zu of %zu\n", sent, chunk);
          break;
        }
        remaining -= chunk;
        offset += chunk;
      }
    } else {
      // 无 DMA 缓冲区时的回退
      size_t sent = client.write(fb->buf, fb->len);
      if (sent != fb->len) {
        Serial.printf("Failed to send frame: sent %zu of %zu\n", sent, fb->len);
      }
    }
    
    client.print("\r\n");

    // 释放帧缓冲
    esp_camera_fb_return(fb);
    
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 【关键修复】在摄像头流循环中调用 server.handleClient()
    // 让 PS2 按钮的 /cmd-button 和 /cmd-stick 请求也能被处理
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    server.handleClient();
    
    // 短暂延迟，避免发送过快
    delay(10);
  }

  // 释放 DMA 缓冲区
  if (dmaBuf) free(dmaBuf);
}