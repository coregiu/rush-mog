#ifndef CAMERA_H
#define CAMERA_H

// ============================================================================
// 摄像头模块开关
//   1 = 启用：初始化摄像头，并在 8080 端口启动 MJPEG 推流
//   0 = 禁用：不初始化摄像头、不占用摄像头引脚、不创建推流任务
// 临时不用摄像头时保持 0；需要恢复时改回 1 即可，相关源码都保留着。
// 也可以在 build_flags 里用 -DENABLE_CAMERA=1 临时覆盖，不必改源码。
// ============================================================================
#ifndef ENABLE_CAMERA
#define ENABLE_CAMERA 0
#endif

#include "Arduino.h"

#if ENABLE_CAMERA
#include "esp_camera.h"
#include "driver/i2c.h"

// GOOUUU V1.3 专用引脚
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      15
#define SIOD_GPIO_NUM      4   // SDA
#define SIOC_GPIO_NUM      5   // SCL

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM      6
#define HREF_GPIO_NUM       7
#define PCLK_GPIO_NUM      13
#endif // ENABLE_CAMERA

// 摄像头是否初始化成功；视频任务在推流前会检查。
// 模块禁用时没有初始化流程，恒为 false，状态接口会如实上报。
extern bool cameraReady;

#if ENABLE_CAMERA
bool initCamera();
#endif

#endif // CAMERA_H