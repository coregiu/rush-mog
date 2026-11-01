#ifndef WEB_H
#define WEB_H

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
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
extern WebSocketsServer webSocket;

#define WS_PORT 81
#define STREAM_PORT 8080

bool initFS();
bool initSdcard();
void handleWebRequest();
void handleCmdButton();
void handleCmdStick();
void handleStop();
void handleStatus();
void handleSdcard();

// 命令处理（HTTP 与 WebSocket 共用）
void processCmdButton(const String& key, const String& type);
void processCmdStick(const String& stickId, const String& direct, const String& stepValue);
// WebSocket 事件回调
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// 视频流任务：独立端口 + 独立任务，避免阻塞控制
void cameraStreamTask(void* arg);

// SD 卡是否挂载成功
extern bool sdcardReady;

#endif // WEB_H