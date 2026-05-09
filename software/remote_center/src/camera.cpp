#include "camera.h"

void initCamera() {
  delay(1000);
  Serial.println("\nGOOUUU V1.3 ESP32-S3 CAM 初始化...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // ===================== 【流畅核心】分辨率 =====================
  config.frame_size = FRAMESIZE_QVGA;     // 320x240 → 超级流畅
  //config.frame_size = FRAMESIZE_CIF;    // 400x296 → 也很流畅
  //config.frame_size = FRAMESIZE_VGA;    // 640x480 → 稍卡但清晰

  config.jpeg_quality = 10;    // 越小越清晰，越大越小越快
  config.fb_count = 2;        // 双缓冲 → 不卡顿

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("初始化失败: 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  camera_fb_t *fb = esp_camera_fb_get();
  // 额外优化参数
  s->set_brightness(s, 1);    // 稍微提亮，减少噪点
  s->set_quality(s, 10);

  Serial.println("------------------------");
  Serial.print("摄像头型号: ");
  if (s->id.PID == OV2640_PID)      Serial.println("OV2640 ✅");
  else if (s->id.PID == OV3660_PID) Serial.println("OV3660 ✅");
  else                              Serial.println("未知");
  Serial.printf("ID: 0x%04X\n", s->id.PID);
  Serial.printf("分辨率: %dx%d\n", fb ? fb->width : 0, fb ? fb->height : 0);
  Serial.println("------------------------");
}