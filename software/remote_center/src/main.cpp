#include "web.h"
#include "camera.h"

void setup()
{
  Serial.begin(9600);
  delay(50);
  initSoftwareSerial();

  // 挂载文件系统
  initFS();
  initCamera();
  initSdcard();

  // ======================
  // 开启 WiFi 热点 (AP 模式)
  // ======================
  WiFi.mode(WIFI_AP); // 强制设为 AP 模式
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PSW);

  if (WiFi.softAP(AP_SSID, AP_PSW))
  {
    Serial.println("WiFi 热点已开启！");
    Serial.print("热点名称：");
    Serial.println(AP_SSID);
    Serial.print("热点密码：");
    Serial.println(AP_PSW);
    Serial.print("手机访问地址：");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("AP启动失败");
  }

  // 启动网页服务
  server.on("/cmd-button", handleCmdButton);
  server.on("/cmd-stick", handleCmdStick);
  server.on("/stream", handleCameraStream);

  server.onNotFound([&]() {
  String uri = server.uri();
  if (uri.startsWith("/sdcard/")) {
    handleSdcard();
  } else {
    handleWebRequest();
  }
});

  server.begin();

  Serial.println("Web 服务器已启动");
}

void loop()
{
  server.handleClient();
}