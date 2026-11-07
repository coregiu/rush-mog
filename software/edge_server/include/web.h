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
// 手机 + ROS + CENTER SERVER 至少 3 个客户端；留 1 个余量给调试电脑。
#define AP_CHANNEL 6
#define AP_MAX_CLIENTS 4
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
void initEdgeConfig();
void serviceMotionKeepalive();
void handleWebRequest();
void handleCmdButton();
void handleCmdStick();
void handleCmd();
void handleStop();
void handleStatus();
void handleSdcard();

// 上位机监控/配置 API（供 dashboard.html 与 ROS/CENTER 上报使用）
//   POST /api/modules/report          模块上报（表单）
//   GET  /api/modules                 模块注册表 + Edge 自身状态
//   GET  /api/modules/{id}            单模块缓存
//   GET  /api/modules/{id}/health     代理模块健康接口
//   GET  /api/modules/{id}/params     代理/本地参数描述
//   POST /api/modules/{id}/params     代理/本地参数修改
void handleApiRequest();
void handleModuleReport();

// 可在运行时修改的边缘参数（NVS 持久化）
extern uint32_t motionKeepaliveMs;
extern uint32_t streamFps;

// 命令处理（HTTP 接口共用）
void processCmdButton(const String& key, const String& type);
void processCmdStick(const String& stickId, const String& direct, const String& stepValue);
// 解析一条控制消息：
//   button:<key>:<type> / stick:<stickId>:<direct>:<stepValue>
// 旧页面的 hold 心跳已废弃，收到会被忽略。
void dispatchControlMessage(const String& msg);

// 视频流任务：独立端口 + 独立任务，避免阻塞控制
void cameraStreamTask(void* arg);

// SD 卡是否挂载成功
extern bool sdcardReady;

#endif // WEB_H