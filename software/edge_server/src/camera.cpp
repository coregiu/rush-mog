#include "camera.h"

bool cameraReady = false;

#if ENABLE_CAMERA
bool initCamera() {
  delay(1000);
  Serial.println("\nGOOUUU V1.3 ESP32-S3 CAM 初始化...");

  camera_config_t config = {};
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
  config.frame_size = FRAMESIZE_QVGA;     // 320x240
  //config.frame_size = FRAMESIZE_CIF;    // 400x296
  //config.frame_size = FRAMESIZE_VGA;    // 640x480

  // QVGA 已经够网页 320px 宽显示；quality 12 比 10 的帧更小，
  // 在 20 FPS 推流时能明显降低 WiFi 带宽和发送阻塞。
  config.jpeg_quality = 12;    // 越小越清晰但帧越大；越大越糊但越流畅
  config.fb_count = 2;         // 双缓冲
  config.fb_location = CAMERA_FB_IN_PSRAM;

  // 推流场景总是拿最新帧，避免旧帧在队列里累积延迟
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败: 0x%x\n", err);
    cameraReady = false;
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s == nullptr) {
    Serial.println("摄像头 sensor 获取失败");
    cameraReady = false;
    return false;
  }

  // 额外优化参数
  s->set_brightness(s, 1);    // 稍微提亮，减少噪点
  s->set_quality(s, 12);

  Serial.println("------------------------");
  Serial.print("摄像头型号: ");
  if (s->id.PID == OV2640_PID)      Serial.println("OV2640 ✅");
  else if (s->id.PID == OV3660_PID) Serial.println("OV3660 ✅");
  else                              Serial.println("未知");
  Serial.printf("ID: 0x%04X\n", s->id.PID);

  // 不要再调用 esp_camera_fb_get() 做自检：无帧时会阻塞最长 4 秒，
  // 而且拿到帧后必须归还。分辨率直接从配置表打印即可。
  Serial.printf("分辨率: %dx%d\n",
                resolution[config.frame_size].width,
                resolution[config.frame_size].height);
  Serial.println("------------------------");

  cameraReady = true;
  return true;
}
#endif // ENABLE_CAMERA
