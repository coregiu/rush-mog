#ifndef WEB_H
#define WEB_H

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "serial.h"
#include "SD_MMC.h"
#include "FS.h"

// 【自己的 WiFi 热点设置】手机直接连这个
#define AP_SSID "RUSH-MOG"          // 热点名称
#define AP_PSW "coregiu_927"        // 热点密码（至少8位）
extern IPAddress local_IP; // 本地IP
extern IPAddress gateway;  // 网关
extern IPAddress subnet; // 子网掩码

extern WebServer server;

// 控制通道直接复用网页的 80 端口：POST /cmd，body 为纯文本命令。
// 不再单独占用 81 端口，也不再需要 WebSocket。
#define STREAM_PORT 8080

bool initFS();
bool initSdcard();
void initWebControl();
void serviceMotionKeepalive();
void handleWebRequest();
void handleCmdButton();
void handleCmdStick();
void handleCmd();
void handleStop();
void handleStatus();
void handleSdcard();

// 命令处理（HTTP 接口共用）
void processCmdButton(const String& key, const String& type);
void processCmdStick(const String& stickId, const String& direct, const String& stepValue);
// 解析一条控制消息：
//   button:<key>:<type> / stick:<stickId>:<direct>:<stepValue> / hold
void dispatchControlMessage(const String& msg);

// 视频流任务：独立端口 + 独立任务，避免阻塞控制
void cameraStreamTask(void* arg);

// SD 卡是否挂载成功
extern bool sdcardReady;

#endif // WEB_H