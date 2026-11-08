#include "web.h"
#include "camera.h"
#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static bool initWiFi()
{
  WiFi.mode(WIFI_AP); // 强制 AP 模式
  WiFi.setSleep(false); // 关闭省电，降低控制/视频抖动

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP 网络配置失败");
    return false;
  }

  // 只调用一次 softAP；手机 + ROS + CENTER 至少 3 个客户端，留 1 个调试余量
  if (!WiFi.softAP(AP_SSID, AP_PSW, AP_CHANNEL, 0, AP_MAX_CLIENTS)) {
    Serial.println("AP 启动失败");
    return false;
  }

  Serial.println("WiFi 热点已开启！");
  Serial.print("热点名称：");
  Serial.println(AP_SSID);
  Serial.print("热点密码：");
  Serial.println(AP_PSW);
  Serial.print("手机访问地址：");
  Serial.println(WiFi.softAPIP());
  return true;
}

// 控制任务：负责 STM32 遥测接收和运动保活；HTTP 控制命令在 80 端口处理
static void controlTask(void* arg)
{
  (void)arg;
  for (;;) {
    serviceSTM32Telemetry();
    serviceMotionKeepalive();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup()
{
  Serial.begin(115200); // 调试口提速，避免日志阻塞控制
  delay(100);
  initSerialPorts();
  initWebControl();

  // 先起网络和控制，再初始化摄像头/SD；摄像头或 SD 异常不影响控制可用性
  initFS();
  initEdgeConfig();
  initWiFi();

  server.on("/cmd", HTTP_POST, handleCmd);
  server.on("/cmd-button", handleCmdButton);
  server.on("/cmd-stick", handleCmdStick);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);

  // 上位机监控/配置 API
  server.on("/api/modules", HTTP_GET, handleApiRequest);
  server.on("/api/modules/report", HTTP_POST, handleModuleReport);
  server.on("/api/modules/edge", HTTP_GET, handleApiRequest);
  server.on("/api/modules/edge/health", HTTP_GET, handleApiRequest);
  server.on("/api/modules/edge/params", HTTP_GET, handleApiRequest);
  server.on("/api/modules/edge/params", HTTP_POST, handleApiRequest);
  server.on("/api/body", HTTP_GET, handleApiRequest);

  // 兼容旧页面：旧版本从 80 端口请求 /stream，重定向到独立视频端口
  server.on("/stream", []() {
    String location = String("http://") + WiFi.softAPIP().toString() + ":8080/stream";
    server.sendHeader("Location", location);
    server.send(302, "text/plain", "");
  });

  server.onNotFound([]() {
    String uri = server.uri();
    if (uri.startsWith("/api/")) {
      handleApiRequest();
    } else if (uri.startsWith("/sdcard/")) {
      handleSdcard();
    } else {
      handleWebRequest();
    }
  });

  server.begin();

  BaseType_t ok = xTaskCreatePinnedToCore(
      controlTask, "ctrl", 4096, nullptr, 4, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("控制任务创建失败");
  }

  // 摄像头初始化可能较慢；失败时仍保留网页和控制，视频接口返回 503
  if (!initCamera()) {
    Serial.println("摄像头不可用，视频流将返回 503");
  }
  initSdcard();

  ok = xTaskCreatePinnedToCore(
      cameraStreamTask, "mjpeg", 4096, nullptr, 2, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("视频流任务创建失败");
  }

  Serial.println("Web 服务器已启动");
  Serial.println("控制接口：POST http://<ip>/cmd（与网页共用 80 端口）");
  Serial.printf("视频流端口：%d\n", STREAM_PORT);
}

void loop()
{
  // 负责 HTTP（静态文件/状态/SD/控制命令）；视频推流和运动保活在独立任务
  server.handleClient();
  delay(1);
}
