/*********
  ESP32-CAM with UART Image Transfer
  Using UART0 (GPIO1/GPIO3) - No Serial Monitor
*********/

#include <Arduino.h>
#include "ESP32CamSD.h"
#include "UARTImageTransfer.h"

// Create camera instance
ESP32CamSD camera;

// Use Serial0 (UART0) for communication with main ESP
// GPIO1 = TX, GPIO3 = RX (default for Serial)
// But we need to reconfigure since Serial is used for USB

String status = "INIT";

void sendLatestImage(int picNum);

void setup() {
  // Initialize Serial for debugging - but this will conflict!
  // Serial.begin(115200);  // DON'T USE - conflicts with UART
  
  // Instead, initialize UART0 for communication
  Serial.begin(115200, SERIAL_8N1);  // This will use GPIO1/GPIO3

  // pinMode(33, OUTPUT);  // LED disabled
  // digitalWrite(33, HIGH);  // Power-on indicator
  
  // Initialize camera and SD card
  if (!camera.begin()) {
    // Camera initialization failed - halt without LED indication
    while(1) {
      delay(1000);
    }
  }
  
  // System ready - can't print confirmation
  // Just blink LED once to indicate ready
  // delay(100);
  // digitalWrite(33, LOW);
  // delay(100);
  // digitalWrite(33, HIGH);
  // delay(100);
  // digitalWrite(33, LOW);   // End with OFF

  // send initial status
  status = "READY";
  Serial.println("READY");
}

void loop() {
  // Check for UART commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      // Handle TAKE_PHOTO command
      if (command == "TAKE_PHOTO") {
        if (camera.takePhotoSaveSD()) {
          Serial.println("OK");
        } else {
          Serial.println("ERROR");
        }
      }
      
      // Handle SEND_IMAGE command
      else if (command == "SEND_IMAGE") {
        int picNum = camera.getPictureNumber();
        if (picNum > 0) {
          sendLatestImage(picNum);
        } else {
          Serial.println("ERROR");
        }
      }
      
      // Handle GET_COUNT command
      else if (command == "GET_COUNT") {
        int count = camera.getPictureNumber();
        Serial.println(String(count));
      }

      // Handle GET_STATUS command
      else if (command == "GET_STATUS") {
        // String status = camera.isCameraPresent() ? "Camera OK" : "Camera Missing";
        Serial.println(status);
      }
      
      // Unknown command
      else {
        Serial.println("ERROR");
      }
    }
  }
  
  delay(10);
}

void sendLatestImage(int picNum) {
  // Construct filename (assuming your ESP32CamSD library uses this format)
  String filename = "/picture" + String(picNum) + ".jpg";
  
  // Open file from SD card
  File file = SD_MMC.open(filename);
  if (!file) {
    Serial.println("ERROR");
    return;
  }
  
  size_t fileSize = file.size();
  
  // Send header
  Serial.println("--START--");
  Serial.println("FILE:" + filename);
  Serial.println("SIZE:" + String(fileSize));
  Serial.println("--DATA--");
  
  // Send file data in chunks
  uint8_t buffer[1024];
  size_t bytesRead;
  size_t totalSent = 0;
  
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    Serial.write(buffer, bytesRead);
    totalSent += bytesRead;
    delay(2);  // Reduced delay for faster transfer
  }
  
  file.close();
  
  // Send end marker
  Serial.println("--END--");
}