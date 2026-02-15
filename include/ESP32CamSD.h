#ifndef ESP32_CAM_SD_H
#define ESP32_CAM_SD_H

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <EEPROM.h>

class ESP32CamSD {
public:
  ESP32CamSD();
  
  // Initialize camera and SD card
  bool begin();
  bool beginCamera();
  bool beginSDCard();
  
  // Take and save photo
  bool takePhotoSaveSD();
  bool takePhotoSaveSD(const char* filename);
  
  // Get current picture number
  int getPictureNumber() const { return pictureNumber; }
  
  // Reset picture counter
  void resetPictureCounter();
  
  // Camera configuration
  void setCameraConfig(camera_config_t config);
  void setFrameSize(framesize_t size);
  void setJpegQuality(int quality);
  void setHorizontalMirror(bool enable);
  void setVerticalFlip(bool enable);
  void setWhiteBalance(bool enable);
  
  // Get sensor for advanced control
  sensor_t* getSensor();
  
private:
  int pictureNumber;
  static const int EEPROM_SIZE = 1;
  bool cameraInitialized;
  bool sdInitialized;
  
  // Default camera configuration for AI Thinker
  camera_config_t getDefaultConfig();
  void applySensorSettings();
};

#endif // ESP32_CAM_SD_H
