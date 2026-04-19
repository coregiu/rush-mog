#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

// WiFi配置（替换成你的WiFi信息）
const char* ssid = "rush-mog";
const char* password = "coregiu";

WebServer server(80); // 创建Web服务器，监听80端口

// 初始化LittleFS文件系统
bool initFS() {
  if (!LittleFS.begin(true)) { // true表示格式化（首次使用建议开启）
    Serial.println("LittleFS挂载失败！");
    return false;
  }
  Serial.println("LittleFS挂载成功");
  return true;
}

// 处理所有网页请求（自动匹配文件路径）
void handleWebRequest() {
  String path = server.uri();
  // 如果访问根目录，默认返回index.html
  if (path == "/") path = "/index.html";
  
  // 从LittleFS读取文件并返回给客户端
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    // server.streamFile(file, getContentType(path));
    server.streamFile(file, "text/html");
    file.close();
  } else {
    // 文件不存在返回404
    server.send(404, "text/plain", "404 Not Found");
  }
}

// 根据文件后缀返回对应的Content-Type（确保浏览器正确解析文件）
// String getContentType(String filename) {
//   if (filename.endsWith(".html")) return "text/html";
//   if (filename.endsWith(".css")) return "text/css";
//   if (filename.endsWith(".js")) return "application/javascript";
//   if (filename.endsWith(".png")) return "image/png";
//   if (filename.endsWith(".jpg")) return "image/jpeg";
//   if (filename.endsWith(".ico")) return "image/x-icon";
//   return "text/plain";
// }

void setup() {
  Serial.begin(115200);
  
  // 初始化文件系统
  if (!initFS()) {
    while (1) delay(1000); // 挂载失败则停止运行
  }

  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi连接成功！");
  Serial.print("ESP32 IP地址：");
  Serial.println(WiFi.localIP()); // 打印ESP32的IP地址

  // 注册请求处理器（所有路径都走这个处理函数）
  server.onNotFound(handleWebRequest);

  // 启动Web服务器
  server.begin();
  Serial.println("Web服务器已启动");
}

void loop() {
  server.handleClient(); // 处理客户端请求
}