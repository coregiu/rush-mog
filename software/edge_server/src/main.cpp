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

  // 只调用一次 softAP；信道固定 6，限制连接数，避免其他设备抢空口
  if (!WiFi.softAP(AP_SSID, AP_PSW, 6, 0, 2)) {
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

// 控制任务：只处理 WebSocket，不会被视频/大文件阻塞
static void wsControlTask(void* arg)
{
  (void)arg;
  for (;;) {
    webSocket.loop();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup()
{
  Serial.begin(115200); // 调试口提速，避免日志阻塞控制
  delay(100);
  initSerialPorts();

  // 先起网络和控制，再初始化摄像头/SD；摄像头或 SD 异常不影响控制可用性
  initFS();
  initWiFi();

  server.on("/cmd-button", handleCmdButton);
  server.on("/cmd-stick", handleCmdStick);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);

  // 兼容旧页面：旧版本从 80 端口请求 /stream，重定向到独立视频端口
  server.on("/stream", []() {
    String location = String("http://") + WiFi.softAPIP().toString() + ":8080/stream";
    server.sendHeader("Location", location);
    server.send(302, "text/plain", "");
  });

  server.onNotFound([]() {
    String uri = server.uri();
    if (uri.startsWith("/sdcard/")) {
      handleSdcard();
    } else {
      handleWebRequest();
    }
  });

  server.begin();

  // WebSocket 控制服务（端口 81）
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  webSocket.enableHeartbeat(5000, 3000, 2);

  BaseType_t ok = xTaskCreatePinnedToCore(
      wsControlTask, "ws_ctrl", 6144, nullptr, 4, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("WebSocket 控制任务创建失败");
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
  Serial.printf("WebSocket 端口：%d\n", WS_PORT);
  Serial.printf("视频流端口：%d\n", STREAM_PORT);
}

void loop()
{
  // 只负责 HTTP 静态文件/状态/SD 文件；视频和控制在独立任务
  server.handleClient();
  delay(1);
}
