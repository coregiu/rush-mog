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

bool initFS();
bool initSdcard();
void handleWebRequest();
void handleCmdButton();
void handleCmdStick();
void handleCameraStream();
void handleSdcard();

#endif // WEB_H