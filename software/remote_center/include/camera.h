#ifndef CAMERA_H
#define CAMERA_H

#include "esp_camera.h"
#include "Arduino.h"
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

void initCamera();

#endif // CAMERA_H