#include "ESP32CamSD.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

ESP32CamSD::ESP32CamSD() 
  : pictureNumber(0),
    cameraInitialized(false),
    sdInitialized(false) {
}

bool ESP32CamSD::begin() {
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  if (!beginCamera()) {
    return false;
  }
  
  if (!beginSDCard()) {
    return false;
  }
  
  return true;
}

bool ESP32CamSD::beginCamera() {
  camera_config_t config = getDefaultConfig();
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  cameraInitialized = true;
  
  // Apply sensor settings
  applySensorSettings();
  
  return true;
}

bool ESP32CamSD::beginSDCard() {
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return false;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return false;
  }
  
  sdInitialized = true;
  
  // Initialize EEPROM and read picture number
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0);
  
  return true;
}

bool ESP32CamSD::takePhotoSaveSD() {
  if (!cameraInitialized || !sdInitialized) {
    Serial.println("Camera or SD card not initialized");
    return false;
  }
  
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  
  // Increment picture number
  pictureNumber++;
  
  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) + ".jpg";
  
  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    esp_camera_fb_return(fb);
    return false;
  }
  
  file.write(fb->buf, fb->len); // payload (image), payload length
  Serial.printf("Saved file to path: %s\n", path.c_str());
  
  // Update EEPROM with new picture number
  EEPROM.write(0, pictureNumber);
  EEPROM.commit();
  
  file.close();
  esp_camera_fb_return(fb);
  
  return true;
}

bool ESP32CamSD::takePhotoSaveSD(const char* filename) {
  if (!cameraInitialized || !sdInitialized) {
    Serial.println("Camera or SD card not initialized");
    return false;
  }
  
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  
  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", filename);
  
  File file = fs.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    esp_camera_fb_return(fb);
    return false;
  }
  
  file.write(fb->buf, fb->len);
  Serial.printf("Saved file to path: %s\n", filename);
  
  file.close();
  esp_camera_fb_return(fb);
  
  return true;
}

void ESP32CamSD::resetPictureCounter() {
  pictureNumber = 0;
  EEPROM.write(0, 0);
  EEPROM.commit();
  Serial.println("Picture counter reset to 0");
}

camera_config_t ESP32CamSD::getDefaultConfig() {
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
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_XGA;  // Changed from UXGA for better quality
    config.jpeg_quality = 12;           // 0-63, lower = better
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }
  
  return config;
}

void ESP32CamSD::applySensorSettings() {
  // Get sensor and adjust settings to fix color issues
  sensor_t * s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Failed to get camera sensor");
    return;
  }
  
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 15);
  s->set_hmirror(s, 1);
  s->set_vflip(s, 1);
  
  // White balance (this is why color looks good)
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);  // AUTO
  
  // Exposure & gain (safe defaults)
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_gain_ctrl(s, 1);
  
  // Neutral color (no green tint)
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  
  // Corrections
  s->set_lenc(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  
  // Wait for AWB to adjust (especially important for first photo)
  delay(1000);
}

void ESP32CamSD::setCameraConfig(camera_config_t config)
{
    // Reinitialize camera with new config
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera re-init failed with error 0x%x\n", err);
        cameraInitialized = false;
    } else {
        cameraInitialized = true;
        applySensorSettings();
    }
}

void ESP32CamSD::setFrameSize(framesize_t size)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s)
    {
        s->set_framesize(s, size);
    }
}

void ESP32CamSD::setJpegQuality(int quality) {
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_quality(s, quality);
  }
}

void ESP32CamSD::setHorizontalMirror(bool enable) {
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, enable ? 1 : 0);
  }
}

void ESP32CamSD::setVerticalFlip(bool enable) {
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, enable ? 1 : 0);
  }
}

void ESP32CamSD::setWhiteBalance(bool enable) {
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, enable ? 1 : 0);
  }
}

sensor_t* ESP32CamSD::getSensor() {
  return esp_camera_sensor_get();
}
