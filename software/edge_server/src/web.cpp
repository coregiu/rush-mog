#include "web.h"
#include "camera.h"
#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Esp.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>

IPAddress local_IP(192, 168, 4, 1); // 本地IP
IPAddress gateway(192, 168, 4, 1);  // 网关
IPAddress subnet(255, 255, 255, 0); // 子网掩码

WebServer server(80);

bool sdcardReady = false;

// 可在运行时修改并通过 NVS 持久化的边缘参数。
uint32_t motionKeepaliveMs = 200;
uint32_t streamFps = 20;

// 上位机模块上报缓存：ROS / CENTER 等模块连上 RUSH-MOG Wi-Fi 后
// 定期 POST /api/modules/report，dashboard 通过 /api/modules 读取。
#define EDGE_API_MAX_MODULES 4
#define EDGE_MODULE_STALE_MS 15000UL

struct ModuleRecord {
  bool used;
  String id;
  String name;
  String version;
  String status;
  String summary;
  String paramsJson;
  String healthUrl;
  String paramsUrl;
  uint32_t updatedMs;
};

static ModuleRecord moduleRecords[EDGE_API_MAX_MODULES];

// 当前持续运动命令。按钮只在按下/松开时各发一条 HTTP 命令，摇杆在
// 按下/拖动/松开时发；按住期间不再发心跳。ESP32 按 200ms 把当前运动
// 命令重发给 STM32，喂 STM32 的 800ms 看门狗，直到收到松开/停止命令。
static SemaphoreHandle_t motionMutex = nullptr;
static String activeMotionCommand;
static bool motionActive = false;
// ROS 的 raw: 命令需要保留第二字节（PWM 档位），并带 1s 租约：
// ROS 进程异常退出后不再续租，ESP32 自动停车。
static bool motionPreserveSecondByte = false;
static uint32_t rawMotionLastMs = 0;
#define RAW_MOTION_TIMEOUT_MS 1000

void initWebControl() {
  if (motionMutex == nullptr) {
    motionMutex = xSemaphoreCreateMutex();
  }
}

static bool isSustainedMotionCommand(const String& cmd) {
  // 按钮：1~9 / A / B / C；摇杆："15" 这类两字节命令
  if (cmd.length() == 1) {
    char c = cmd[0];
    return (c >= '1' && c <= '9') || c == 'A' || c == 'B' || c == 'C';
  }
  if (cmd.length() == 2) {
    char c = cmd[0];
    return c >= '1' && c <= '8';
  }
  return false;
}

static bool isValidRawMotionCommand(const String& cmd) {
  if (cmd.length() < 1 || cmd.length() > 2) {
    return false;
  }
  char c = cmd[0];
  bool base_ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'C');
  if (!base_ok) {
    return false;
  }
  if (cmd.length() == 2) {
    if (c == '0') {
      return false;
    }
    char level = cmd[1];
    return level >= '0' && level <= '9';
  }
  return true;
}

static void updateMotionState(const String& cmd) {
  if (motionMutex == nullptr ||
      xSemaphoreTake(motionMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  if (cmd == "0" || cmd == "J" || cmd == "D") {
    // 停止 / 复位 / 测试模式：清掉持续运动
    motionActive = false;
    activeMotionCommand = "";
    motionPreserveSecondByte = false;
    rawMotionLastMs = 0;
  } else if (isSustainedMotionCommand(cmd)) {
    // 按下后一直保持，直到收到松开/停止命令
    motionActive = true;
    activeMotionCommand = cmd;
    motionPreserveSecondByte = false;
    rawMotionLastMs = 0;
  }

  xSemaphoreGive(motionMutex);
}

// ROS 专用：命令已通过 isValidRawMotionCommand 校验，保留第二字节并续租。
static void updateMotionStateRaw(const String& cmd) {
  if (motionMutex == nullptr ||
      xSemaphoreTake(motionMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  if (cmd == "0") {
    motionActive = false;
    activeMotionCommand = "";
    motionPreserveSecondByte = false;
    rawMotionLastMs = 0;
  } else if (isValidRawMotionCommand(cmd)) {
    motionActive = true;
    activeMotionCommand = cmd;
    motionPreserveSecondByte = (cmd.length() == 2);
    rawMotionLastMs = millis();
  }

  xSemaphoreGive(motionMutex);
}

static void clearMotionState() {
  if (motionMutex == nullptr ||
      xSemaphoreTake(motionMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }
  motionActive = false;
  activeMotionCommand = "";
  motionPreserveSecondByte = false;
  rawMotionLastMs = 0;
  xSemaphoreGive(motionMutex);
}

// /status 用：读取当前是否有持续运动（加锁避免和保活任务竞争）
static bool isMotionActive() {
  bool active = false;
  if (motionMutex != nullptr &&
      xSemaphoreTake(motionMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    active = motionActive;
    xSemaphoreGive(motionMutex);
  }
  return active;
}

// 由控制任务周期调用：浏览器只在按下/松开时发命令，ESP32 负责把当前
// 运动命令持续重发给 STM32（喂 800ms 看门狗），直到收到松开/停止命令。
void serviceMotionKeepalive() {
  static TickType_t last = 0;
  TickType_t now = xTaskGetTickCount();
  uint32_t interval = motionKeepaliveMs;
  if (interval < 100) interval = 100;
  if (interval > 600) interval = 600;
  if (now - last < pdMS_TO_TICKS(interval)) {
    return;
  }
  last = now;

  if (motionMutex == nullptr ||
      xSemaphoreTake(motionMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }

  String cmd;
  bool preserveSecondByte = false;
  if (motionActive) {
    cmd = activeMotionCommand;
    preserveSecondByte = motionPreserveSecondByte;
    // raw: 命令是 1s 租约；ROS 停止续租后自动停车（比 STM32 800ms 看门狗更保守）。
    if (preserveSecondByte &&
        (millis() - rawMotionLastMs) > RAW_MOTION_TIMEOUT_MS) {
      motionActive = false;
      activeMotionCommand = "";
      motionPreserveSecondByte = false;
      rawMotionLastMs = 0;
      preserveSecondByte = false;
      cmd = "0";
    }
  }

  xSemaphoreGive(motionMutex);

  if (!cmd.isEmpty()) {
    // 摇杆命令形如 "15"（方向1 + 速度5）。UI 保活只重发方向，避免把用户
    // 后来按三角/方块调过的速度/微调又覆盖掉；ROS raw 命令保留第二字节。
    if (cmd.length() == 2 && !preserveSecondByte) {
      cmd = cmd.substring(0, 1);
    }
    sendToSTM32(cmd);
  }
}

// 读取 NVS 中的边缘参数；范围与 dashboard 暴露的 min/max 保持一致。
void initEdgeConfig() {
  Preferences prefs;
  if (!prefs.begin("edgecfg", true)) {
    Serial.println("边缘参数 NVS 不可用，使用默认值");
    return;
  }
  motionKeepaliveMs = prefs.getUInt("keepalive", 200);
  streamFps = prefs.getUInt("fps", 20);
  prefs.end();

  if (motionKeepaliveMs < 100) motionKeepaliveMs = 100;
  if (motionKeepaliveMs > 600) motionKeepaliveMs = 600;
  if (streamFps < 5) streamFps = 5;
  if (streamFps > 20) streamFps = 20;

  Serial.printf("边缘参数: motion_keepalive_ms=%lu, stream_fps=%lu\n",
                (unsigned long)motionKeepaliveMs, (unsigned long)streamFps);
}

static void saveEdgeConfig() {
  Preferences prefs;
  if (!prefs.begin("edgecfg", false)) {
    Serial.println("边缘参数保存失败：NVS 不可写");
    return;
  }
  prefs.putUInt("keepalive", motionKeepaliveMs);
  prefs.putUInt("fps", streamFps);
  prefs.end();
}

static TickType_t streamFrameInterval() {
  uint32_t fps = streamFps;
  if (fps < 5) fps = 5;
  if (fps > 20) fps = 20;
  return pdMS_TO_TICKS(1000 / fps);
}

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
  // HTML 与 dashboard 资源不缓存，避免升级后手机还拿旧页面；其它静态资源长缓存。
  if (path.endsWith(".html") || path == "/dashboard.js" || path == "/dashboard.css") {
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

// 按钮命令处理（POST /cmd 与 /cmd-button 共用）
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
    updateMotionState(command);
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

// 摇杆命令处理（POST /cmd 与 /cmd-stick 共用）
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
    updateMotionState(command);
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
  clearMotionState();
  sendToSTM32("0");
  server.send(200, "text/plain", "stopped");
}

void handleStatus() {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"uptime_ms\":%lu,\"free_heap\":%u,\"free_psram\":%u,"
           "\"camera\":%u,\"sdcard\":%u,\"motion_active\":%u}",
           (unsigned long)millis(),
           (unsigned)ESP.getFreeHeap(),
           (unsigned)ESP.getFreePsram(),
           cameraReady ? 1U : 0U,
           sdcardReady ? 1U : 0U,
           isMotionActive() ? 1U : 0U);
  server.send(200, "application/json", String(json));
}

// ============================================================================
// 上位机监控/配置 API
//   ROS / CENTER 连上 RUSH-MOG Wi-Fi 后向 192.168.4.1 上报；
//   dashboard.html 只访问 192.168.4.1，由 ESP32 代理模块接口，避免 CORS。
// ============================================================================

static String jsonEscape(const String& input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    uint8_t c = (uint8_t)input[i];
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += (char)c;
        }
    }
  }
  return out;
}

static String jsonQuote(const String& value) {
  return String("\"") + jsonEscape(value) + "\"";
}

static String truncateText(const String& value, size_t maxLen) {
  if (value.length() <= maxLen) {
    return value;
  }
  return value.substring(0, maxLen);
}

static String sanitizeJson(const String& value) {
  String trimmed = value;
  trimmed.trim();
  if (trimmed.length() < 2) {
    return "{}";
  }
  if (trimmed[0] != '{' && trimmed[0] != '[') {
    return "{}";
  }
  return trimmed;
}

static bool isValidModuleId(const String& id) {
  if (id.length() < 1 || id.length() > 16) {
    return false;
  }
  for (size_t i = 0; i < id.length(); ++i) {
    char c = id[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) {
      return false;
    }
  }
  return true;
}

static String edgeParamsObjectJson() {
  String json;
  json.reserve(320);
  json += "{";
  json += "\"motion_keepalive_ms\":" + String((unsigned long)motionKeepaliveMs);
  json += ",\"stream_fps\":" + String((unsigned long)streamFps);
  json += ",\"free_heap\":" + String((unsigned long)ESP.getFreeHeap());
  json += ",\"free_psram\":" + String((unsigned long)ESP.getFreePsram());
  json += ",\"uptime_ms\":" + String((unsigned long)millis());
  json += ",\"wifi_clients\":" + String((unsigned)WiFi.softAPgetStationNum());
  json += ",\"camera\":" + String(cameraReady ? "true" : "false");
  json += ",\"sdcard\":" + String(sdcardReady ? "true" : "false");
  json += ",\"motion_active\":" + String(isMotionActive() ? "true" : "false");
  json += ",\"ap_ssid\":" + jsonQuote(String(AP_SSID));
  json += ",\"ap_ip\":" + jsonQuote(WiFi.softAPIP().toString());
  json += ",\"ap_channel\":" + String((unsigned)AP_CHANNEL);
  json += ",\"max_clients\":" + String((unsigned)AP_MAX_CLIENTS);
  json += "}";
  return json;
}

static String edgeSummary() {
  String summary = "camera=";
  summary += cameraReady ? "on" : "off";
  summary += ", sdcard=";
  summary += sdcardReady ? "on" : "off";
  summary += ", clients=";
  summary += String((unsigned)WiFi.softAPgetStationNum());
  return summary;
}

// STM32 编码器遥测。valid=true 表示收到过帧，stale=true 表示超过 2s 没更新。
static String bodyTelemetryJson() {
  BodyTelemetry telemetry;
  bool valid = getSTM32Telemetry(&telemetry);
  unsigned long ageMs = valid ? (unsigned long)(millis() - telemetry.receivedMs) : 0;
  bool stale = !valid || ageMs > 2000;

  String json;
  json.reserve(240);
  json += "{";
  json += "\"valid\":";
  json += valid ? "true" : "false";
  json += ",\"stale\":";
  json += stale ? "true" : "false";
  json += ",\"source\":\"stm32\"";
  json += ",\"rate_hz\":10";
  json += ",\"seq\":" + String((unsigned long)telemetry.seq);
  json += ",\"uptime_ms\":" + String((unsigned long)telemetry.uptimeMs);
  json += ",\"age_ms\":";
  json += valid ? String(ageMs) : String(-1L);
  json += ",\"wheel_mm_s\":[";
  for (int i = 0; i < 4; ++i) {
    if (i > 0) {
      json += ",";
    }
    json += String((long)telemetry.wheelMmS[i]);
  }
  json += "],\"distance_mm\":" + String((long)telemetry.distanceMm);
  json += "}";
  return json;
}

static String edgeModuleJson() {
  String json;
  json.reserve(1200);
  json += "{";
  json += "\"id\":\"edge\",\"name\":\"Edge Server\",\"version\":\"0.2.0\",";
  json += "\"ip\":" + jsonQuote(WiFi.softAPIP().toString()) + ",";
  json += "\"status\":" + jsonQuote((cameraReady && sdcardReady) ? "ok" : "degraded") + ",";
  json += "\"summary\":" + jsonQuote(edgeSummary()) + ",";
  json += "\"health_url\":" + jsonQuote(String("http://192.168.4.1/status")) + ",";
  json += "\"params_url\":" + jsonQuote(String("http://192.168.4.1/api/modules/edge/params")) + ",";
  json += "\"updated_ms\":" + String((unsigned long)millis()) + ",";
  json += "\"age_ms\":0,\"stale\":false,";
  json += "\"params\":" + edgeParamsObjectJson();
  json += ",\"body\":" + bodyTelemetryJson();
  json += "}";
  return json;
}

static String edgeParamItem(const String& key, const String& label,
                            const String& valueLiteral, const String& type,
                            const String& group, bool editable,
                            const String& unit, const String& minLiteral,
                            const String& maxLiteral, const String& stepLiteral) {
  String item = "{";
  item += "\"key\":" + jsonQuote(key);
  item += ",\"label\":" + jsonQuote(label);
  item += ",\"value\":" + valueLiteral;
  item += ",\"type\":" + jsonQuote(type);
  item += ",\"group\":" + jsonQuote(group);
  item += ",\"editable\":" + String(editable ? "true" : "false");
  if (unit.length() > 0) item += ",\"unit\":" + jsonQuote(unit);
  if (minLiteral.length() > 0) item += ",\"min\":" + minLiteral;
  if (maxLiteral.length() > 0) item += ",\"max\":" + maxLiteral;
  if (stepLiteral.length() > 0) item += ",\"step\":" + stepLiteral;
  item += "}";
  return item;
}

static String edgeParamsJson() {
  String json;
  json.reserve(2000);
  json += "{\"ok\":true,\"module\":\"edge\",\"name\":\"Edge Server\",";
  json += "\"version\":\"0.2.0\",\"editable\":true,\"requires_restart\":false,\"params\":[";
  bool first = true;
  auto add = [&](const String& item) {
    if (!first) json += ",";
    json += item;
    first = false;
  };

  add(edgeParamItem("motion_keepalive_ms", "运动保活周期",
                    String((unsigned long)motionKeepaliveMs), "int", "运动控制", true,
                    "ms", "100", "600", "10"));
  add(edgeParamItem("stream_fps", "视频流帧率",
                    String((unsigned long)streamFps), "int", "视频", true,
                    "fps", "5", "20", "1"));
  add(edgeParamItem("ap_ssid", "WiFi 名称", jsonQuote(String(AP_SSID)), "string",
                    "网络", false, "", "", "", ""));
  add(edgeParamItem("ap_ip", "热点地址", jsonQuote(WiFi.softAPIP().toString()), "string",
                    "网络", false, "", "", "", ""));
  add(edgeParamItem("ap_channel", "WiFi 信道", String((unsigned)AP_CHANNEL), "int",
                    "网络", false, "", "1", "13", "1"));
  add(edgeParamItem("max_clients", "最大客户端数", String((unsigned)AP_MAX_CLIENTS), "int",
                    "网络", false, "", "1", "8", "1"));
  add(edgeParamItem("wifi_clients", "当前客户端数",
                    String((unsigned)WiFi.softAPgetStationNum()), "int", "网络", false,
                    "", "0", "8", "1"));
  add(edgeParamItem("free_heap", "剩余堆内存", String((unsigned long)ESP.getFreeHeap()),
                    "int", "运行状态", false, "bytes", "", "", ""));
  add(edgeParamItem("free_psram", "剩余 PSRAM", String((unsigned long)ESP.getFreePsram()),
                    "int", "运行状态", false, "bytes", "", "", ""));
  add(edgeParamItem("uptime_ms", "运行时长", String((unsigned long)millis()),
                    "int", "运行状态", false, "ms", "", "", ""));
  add(edgeParamItem("camera", "摄像头", String(cameraReady ? "true" : "false"), "bool",
                    "运行状态", false, "", "", "", ""));
  add(edgeParamItem("sdcard", "SD 卡", String(sdcardReady ? "true" : "false"), "bool",
                    "运行状态", false, "", "", "", ""));
  add(edgeParamItem("motion_active", "运动保活中",
                    String(isMotionActive() ? "true" : "false"), "bool",
                    "运行状态", false, "", "", "", ""));
  json += "]}";
  return json;
}

static int findModuleIndex(const String& id) {
  for (int i = 0; i < EDGE_API_MAX_MODULES; ++i) {
    if (moduleRecords[i].used && moduleRecords[i].id == id) {
      return i;
    }
  }
  return -1;
}

static int findFreeModuleIndex() {
  for (int i = 0; i < EDGE_API_MAX_MODULES; ++i) {
    if (!moduleRecords[i].used) {
      return i;
    }
  }
  return -1;
}

static String moduleJson(const ModuleRecord& record, bool includeParams) {
  uint32_t age = millis() - record.updatedMs;
  bool stale = age > EDGE_MODULE_STALE_MS;
  String json;
  json.reserve(includeParams ? 1400 : 600);
  json += "{";
  json += "\"id\":" + jsonQuote(record.id);
  json += ",\"name\":" + jsonQuote(record.name);
  json += ",\"version\":" + jsonQuote(record.version);
  json += ",\"status\":" + jsonQuote(record.status);
  json += ",\"summary\":" + jsonQuote(record.summary);
  json += ",\"health_url\":" + jsonQuote(record.healthUrl);
  json += ",\"params_url\":" + jsonQuote(record.paramsUrl);
  json += ",\"updated_ms\":" + String((unsigned long)record.updatedMs);
  json += ",\"age_ms\":" + String((unsigned long)age);
  json += ",\"stale\":" + String(stale ? "true" : "false");
  if (includeParams) {
    json += ",\"params\":" + sanitizeJson(record.paramsJson);
  }
  json += "}";
  return json;
}

static String modulesJson() {
  String json;
  json.reserve(4096);
  json += "{\"ok\":true,\"server_time_ms\":" + String((unsigned long)millis());
  json += ",\"edge\":" + edgeModuleJson();
  json += ",\"modules\":[";
  bool first = true;
  int count = 0;
  for (int i = 0; i < EDGE_API_MAX_MODULES; ++i) {
    if (!moduleRecords[i].used) {
      continue;
    }
    if (!first) {
      json += ",";
    }
    json += moduleJson(moduleRecords[i], true);
    first = false;
    count++;
  }
  json += "],\"count\":" + String(count) + "}";
  return json;
}

static void sendApiError(int status, const String& code, const String& message) {
  String json = "{\"ok\":false,\"error\":{\"code\":" + jsonQuote(code);
  json += ",\"message\":" + jsonQuote(message) + "}}";
  server.send(status, "application/json", json);
}

static void proxyModule(const ModuleRecord& record, bool wantParams, bool isPost,
                        const String& body, const String& contentType) {
  String url = wantParams ? record.paramsUrl : record.healthUrl;
  if (url.length() == 0) {
    sendApiError(400, "module_url_missing", "模块未上报接口地址");
    return;
  }
  if (!url.startsWith("http://192.168.4.")) {
    sendApiError(400, "invalid_module_url", "只允许代理 RUSH-MOG 网段模块");
    return;
  }
  if (isPost && body.length() == 0) {
    sendApiError(400, "empty_body", "缺少参数请求体");
    return;
  }

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(1500);
  http.setTimeout(2500);
  if (!http.begin(client, url)) {
    sendApiError(502, "module_unreachable", "无法连接模块: " + record.id);
    return;
  }

  int code = 0;
  if (isPost) {
    String outgoingType = contentType.length() > 0 ? contentType : String("application/json");
    http.addHeader("Content-Type", outgoingType);
    code = http.POST(body);
  } else {
    code = http.GET();
  }

  if (code <= 0) {
    http.end();
    sendApiError(502, "module_unreachable", "模块未响应: " + record.id);
    return;
  }

  String payload = http.getString();
  http.end();
  if (payload.length() == 0) {
    payload = "{}";
  }
  server.send(code, "application/json", payload);
}

static void handleEdgeParamsPost() {
  bool updated = false;

  if (server.hasArg("motion_keepalive_ms")) {
    long value = server.arg("motion_keepalive_ms").toInt();
    if (value < 100 || value > 600) {
      sendApiError(400, "invalid_param", "motion_keepalive_ms 必须在 100..600");
      return;
    }
    motionKeepaliveMs = (uint32_t)value;
    updated = true;
  }
  if (server.hasArg("stream_fps")) {
    long value = server.arg("stream_fps").toInt();
    if (value < 5 || value > 20) {
      sendApiError(400, "invalid_param", "stream_fps 必须在 5..20");
      return;
    }
    streamFps = (uint32_t)value;
    updated = true;
  }

  if (!updated) {
    sendApiError(400, "no_updates", "没有可更新的参数");
    return;
  }

  saveEdgeConfig();
  String json = "{\"ok\":true,\"updated\":{\"motion_keepalive_ms\":" +
                String((unsigned long)motionKeepaliveMs) + ",\"stream_fps\":" +
                String((unsigned long)streamFps) + "},\"message\":\"边缘参数已保存\"}";
  server.send(200, "application/json", json);
}

void handleModuleReport() {
  String module = server.arg("module");
  module.trim();
  if (!isValidModuleId(module) || module == "edge") {
    sendApiError(400, "invalid_module", "module 非法或为保留值 edge");
    return;
  }

  String name = truncateText(server.arg("name"), 48);
  if (name.length() == 0) {
    name = module;
  }
  String version = truncateText(server.arg("version"), 24);
  String status = truncateText(server.arg("status"), 16);
  if (status.length() == 0) {
    status = "unknown";
  }
  String summary = truncateText(server.arg("summary"), 240);
  String paramsJson = server.arg("params_json");
  if (paramsJson.length() > 2048) {
    paramsJson = "{}";
  }
  paramsJson = sanitizeJson(paramsJson);

  int healthPort = server.arg("health_port").toInt();
  int paramsPort = server.arg("params_port").toInt();
  if (healthPort <= 0) {
    healthPort = (module == "center") ? 8000 : 8090;
  }
  if (paramsPort <= 0) {
    paramsPort = healthPort;
  }
  if (healthPort > 65535 || paramsPort > 65535) {
    sendApiError(400, "invalid_port", "端口必须在 1..65535");
    return;
  }

  String healthPath = server.arg("health_path");
  String paramsPath = server.arg("params_path");
  if (healthPath.length() == 0) {
    healthPath = (module == "center") ? "/v1/health" : "/ros/health";
  }
  if (paramsPath.length() == 0) {
    paramsPath = (module == "center") ? "/v1/params" : "/ros/params";
  }
  if (!healthPath.startsWith("/") || !paramsPath.startsWith("/") ||
      healthPath.indexOf("..") >= 0 || paramsPath.indexOf("..") >= 0 ||
      healthPath.length() > 80 || paramsPath.length() > 80) {
    sendApiError(400, "invalid_path", "health_path/params_path 非法");
    return;
  }

  IPAddress remote = server.client().remoteIP();
  String ip = remote.toString();
  if (ip == "0.0.0.0") {
    sendApiError(400, "invalid_client", "无法获取模块 IP");
    return;
  }

  int index = findModuleIndex(module);
  if (index < 0) {
    index = findFreeModuleIndex();
  }
  if (index < 0) {
    sendApiError(507, "registry_full", "模块注册表已满");
    return;
  }

  ModuleRecord& record = moduleRecords[index];
  record.used = true;
  record.id = module;
  record.name = name;
  record.version = version;
  record.status = status;
  record.summary = summary;
  record.paramsJson = paramsJson;
  record.healthUrl = "http://" + ip + ":" + String(healthPort) + healthPath;
  record.paramsUrl = "http://" + ip + ":" + String(paramsPort) + paramsPath;
  record.updatedMs = millis();

  Serial.printf("模块上报: %s (%s:%d) status=%s\n",
                module.c_str(), ip.c_str(), healthPort, status.c_str());

  String response = "{\"ok\":true,\"module\":" + jsonQuote(module);
  response += ",\"ip\":" + jsonQuote(ip);
  response += ",\"health_url\":" + jsonQuote(record.healthUrl);
  response += ",\"params_url\":" + jsonQuote(record.paramsUrl);
  response += "}";
  server.send(200, "application/json", response);
}

void handleApiRequest() {
  const String uri = server.uri();
  HTTPMethod method = server.method();

  if (uri == "/api/modules") {
    if (method != HTTP_GET) {
      sendApiError(405, "method_not_allowed", "只支持 GET");
      return;
    }
    server.send(200, "application/json", modulesJson());
    return;
  }

  if (uri == "/api/modules/edge") {
    if (method != HTTP_GET) {
      sendApiError(405, "method_not_allowed", "只支持 GET");
      return;
    }
    server.send(200, "application/json", edgeModuleJson());
    return;
  }

  if (uri == "/api/modules/edge/health") {
    if (method != HTTP_GET) {
      sendApiError(405, "method_not_allowed", "只支持 GET");
      return;
    }
    server.send(200, "application/json", edgeModuleJson());
    return;
  }

  if (uri == "/api/body") {
    if (method != HTTP_GET) {
      sendApiError(405, "method_not_allowed", "只支持 GET");
      return;
    }
    String json = "{\"ok\":true,\"body\":";
    json += bodyTelemetryJson();
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  if (uri == "/api/modules/edge/params") {
    if (method == HTTP_GET) {
      server.send(200, "application/json", edgeParamsJson());
      return;
    }
    if (method == HTTP_POST) {
      handleEdgeParamsPost();
      return;
    }
    sendApiError(405, "method_not_allowed", "支持 GET/POST");
    return;
  }

  const String prefix = "/api/modules/";
  if (!uri.startsWith(prefix)) {
    sendApiError(404, "not_found", "未知 API 路径");
    return;
  }

  String rest = uri.substring(prefix.length());
  bool wantsHealth = rest.endsWith("/health");
  bool wantsParams = rest.endsWith("/params");
  String moduleId = rest;
  if (wantsHealth || wantsParams) {
    int slash = rest.lastIndexOf('/');
    if (slash > 0) {
      moduleId = rest.substring(0, slash);
    }
  }

  moduleId.trim();
  if (!isValidModuleId(moduleId)) {
    sendApiError(404, "unknown_module", "未知模块");
    return;
  }

  int index = findModuleIndex(moduleId);
  if (index < 0) {
    sendApiError(404, "module_not_reported", "模块尚未上报: " + moduleId);
    return;
  }
  ModuleRecord& record = moduleRecords[index];

  if (wantsHealth) {
    if (method != HTTP_GET) {
      sendApiError(405, "method_not_allowed", "只支持 GET");
      return;
    }
    proxyModule(record, false, false, "", "");
    return;
  }

  if (wantsParams) {
    if (method == HTTP_GET) {
      proxyModule(record, true, false, "", "");
      return;
    }
    if (method == HTTP_POST) {
      String body = server.arg("plain");
      if (body.length() > 4096) {
        sendApiError(413, "payload_too_large", "参数请求体过大");
        return;
      }
      proxyModule(record, true, true, body, "application/json");
      return;
    }
    sendApiError(405, "method_not_allowed", "支持 GET/POST");
    return;
  }

  if (method == HTTP_GET) {
    server.send(200, "application/json", moduleJson(record, true));
    return;
  }
  sendApiError(405, "method_not_allowed", "只支持 GET");
}

// 解析一条控制消息（POST /cmd 的 body）
void dispatchControlMessage(const String& msg) {
  //   button:<key>:<type>                   例如 button:up:down
  //   stick:<stickId>:<direct>:<stepValue>  例如 stick:left-analog:up:5
  // 旧页面的 "hold" 心跳已废弃，收到后直接忽略（不匹配下面的前缀）。
  if (msg.startsWith("raw:")) {
    String command = msg.substring(4);
    command.trim();
    if (!isValidRawMotionCommand(command)) {
      return;
    }
    sendToSTM32(command);
    if (command == "0") {
      clearMotionState();
    } else {
      updateMotionStateRaw(command);
    }
  } else if (msg.startsWith("button:")) {
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
}

// 控制入口：POST /cmd，body 为纯文本命令，和网页共用 80 端口。
// 按钮只在按下/松开时发短请求，摇杆在按下/拖动/松开时发；持续运动由控制任务每 200ms 重发给 STM32。
void handleCmd() {
  String msg = server.arg("plain");
  if (msg.isEmpty() && server.args() > 0) {
    // 兼容 curl -d（application/x-www-form-urlencoded）：
    // 这种情况下整条命令会被解析成参数名
    msg = server.argName(0);
  }
  msg.trim();
  if (msg.isEmpty()) {
    server.send(400, "text/plain", "empty command");
    return;
  }
  if (msg.length() > 32) {
    server.send(400, "text/plain", "command too long");
    return;
  }
  dispatchControlMessage(msg);
  server.send(200, "text/plain", "ok");
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

// 摄像头流任务：端口 8080，独立于 HTTP 80
void cameraStreamTask(void* arg) {
  (void)arg;

  WiFiServer streamServer(STREAM_PORT);
  streamServer.begin();
  Serial.printf("MJPEG 视频流已启动，端口：%d\n", STREAM_PORT);

  WiFiClient client;
  TickType_t lastFrame = 0;
  int failCount = 0;
  // 帧间隔由 dashboard 可配置的 streamFps 决定，每轮重新计算以支持热更新

  // 每 5 秒打印一次实际推流 FPS / 平均帧大小，便于按网络情况调参
  uint32_t framesSent = 0;
  uint32_t bytesSent = 0;
  uint32_t fpsWindowStart = millis();

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
      lastFrame = xTaskGetTickCount() - streamFrameInterval();
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
    if (now - lastFrame < streamFrameInterval()) {
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
    size_t frameLen = fb->len;
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

    framesSent++;
    bytesSent += (uint32_t)frameLen;
    uint32_t nowMs = millis();
    if (nowMs - fpsWindowStart >= 5000) {
      uint32_t elapsed = nowMs - fpsWindowStart;
      Serial.printf("MJPEG: %lu FPS, %lu B/frame\n",
                    (unsigned long)(framesSent * 1000UL / elapsed),
                    (unsigned long)(bytesSent / framesSent));
      framesSent = 0;
      bytesSent = 0;
      fpsWindowStart = nowMs;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
