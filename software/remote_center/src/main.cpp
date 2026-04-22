#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

// 【自己的 WiFi 热点设置】手机直接连这个
#define AP_SSID "RUSH-MOG"          // 热点名称
#define AP_PSW "coregiu_927"        // 热点密码（至少8位）
IPAddress local_IP(192, 168, 4, 1); // 本地IP
IPAddress gateway(192, 168, 4, 1);  // 网关
IPAddress subnet(255, 255, 255, 0); // 子网掩码

ESP8266WebServer server(80);

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

// 网页请求处理
void handleWebRequest()
{
  String path = server.uri();
  if (path == "/")
    path = "/index.html";

  if (LittleFS.exists(path))
  {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, "text/html");
    file.close();
  }
  else
  {
    server.send(404, "text/plain", "404 Not Found - index.html");
  }
}

void handleCmdButton() {
  // 定义按钮
  String key = server.arg("key");
  // 定义事件类型：按下还是松开
  String type = server.arg("type");
  if (key.isEmpty() || type.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'key' or 'type' parameter");
    return;
  }
  if (type == "up") {
    if (key == "triangle" || key == "cross") {
      
    } else if (key == "square" || key == "circle") {
      Serial.println('I');  
    } else {
      Serial.println('0'); 
    }     
  } else {
    if (key == "up") {
      Serial.println('1');  
    } else if (key == "down") {
      Serial.println('2');  
    } else if (key == "left") {
      Serial.println('3');  
    } else if (key == "right") {
      Serial.println('4');  
    } else if (key == "triangle") {
      Serial.println('E');  
    } else if (key == "square") {
      Serial.println('F');  
    } else if (key == "circle") {
      Serial.println('G');  
    } else if (key == "cross") {
      Serial.println('H');  
    } else if (key == "l1") {
      Serial.println('9');  
    } else if (key == "r1") {
      Serial.println('A');  
    } else if (key == "l2") {
      Serial.println('B');  
    } else if (key == "r2") {
      Serial.println('C');  
    }
  }

  server.send(200);
}

void handleCmdStick() {
  String stickId = server.arg("stickId");
  String direct = server.arg("direct");
  String stepValue = server.arg("stepValue");
  if (stickId.isEmpty() || direct.isEmpty() || stepValue.isEmpty()) {
    server.send(400, "text/plain", "400 Bad Request - Missing 'stickId', 'direct' or 'stepValue' parameter");
    return;
  }
  if (stickId != "left-analog") {
    server.send(200);
    return;
  }
  if (direct == "stop") {
    Serial.println('0'); 
  } else {
    if (direct == "up") {
      Serial.println('1'+stepValue); 
    } else if (direct == "down") {
      Serial.println('2'+stepValue); 
    } else if (direct == "left") {
      Serial.println('3'+stepValue); 
    } else if (direct == "right") {
      Serial.println('4'+stepValue); 
    } else if (direct == "uple") {
      Serial.println('5'+stepValue); 
    } else if (direct == "uprt") {
      Serial.println('6'+stepValue); 
    } else if (direct == "dnle") {
      Serial.println('7'+stepValue); 
    } else if (direct == "dnrt") {
      Serial.println('8'+stepValue); 
    }
  }
  // Serial.println(stickId + " - Direct: " + direct + ", Step Value: " + stepValue); 
  server.send(200);
}

void setup()
{
  Serial.begin(9600);
  delay(50);

  // 挂载文件系统
  initFS();

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
  server.onNotFound(handleWebRequest);
  server.on("/cmd-button", handleCmdButton);
  server.on("/cmd-stick", handleCmdStick);
  server.begin();
  Serial.println("Web 服务器已启动");
}

void loop()
{
  server.handleClient();
}