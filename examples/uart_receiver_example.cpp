/*********
  ESP32 UART Image Receiver (for the other ESP)
  
  This code receives images from ESP32-CAM via UART
  and can send commands to trigger photo capture
  
  Hardware connections:
  - UART2 TX: GPIO17 (connect to ESP32-CAM RX/GPIO16)
  - UART2 RX: GPIO16 (connect to ESP32-CAM TX/GPIO17)
  - Don't forget common ground!
  
  Commands you can send:
  - TAKE_PHOTO: Tell ESP32-CAM to take a photo
  - SEND_IMAGE: Request the latest image
  - GET_COUNT: Get the current picture count
*********/

#include <Arduino.h>

// UART2 for communication with ESP32-CAM
HardwareSerial CamSerial(2);

#define UART_BAUD 115200
#define CHUNK_SIZE 1024

// Function declarations
void sendCommand(const char* command);
String readResponse();
void receiveImage();

void setup() {
  // Serial for debugging (USB)
  Serial.begin(115200);
  Serial.println("\n\nESP32 UART Image Receiver");
  
  // Initialize UART2 for communication with ESP32-CAM
  // RX=GPIO16, TX=GPIO17
  CamSerial.begin(UART_BAUD, SERIAL_8N1, 16, 17);
  
  Serial.println("System ready!");
  Serial.println("\nCommands:");
  Serial.println("  1 - Take photo");
  Serial.println("  2 - Receive latest image");
  Serial.println("  3 - Get picture count");
  Serial.println();
}

void loop() {
  // Check for user input from Serial Monitor
  if (Serial.available()) {
    char input = Serial.read();
    
    switch (input) {
      case '1':
        Serial.println("\n=== Taking Photo ===");
        sendCommand("TAKE_PHOTO");
        delay(100);
        Serial.print("Response: ");
        Serial.println(readResponse());
        break;
        
      case '2':
        Serial.println("\n=== Requesting Image ===");
        sendCommand("SEND_IMAGE");
        delay(100);
        receiveImage();
        break;
        
      case '3':
        Serial.println("\n=== Getting Count ===");
        sendCommand("GET_COUNT");
        delay(100);
        Serial.print("Response: ");
        Serial.println(readResponse());
        break;
        
      case '\r':
      case '\n':
        // Ignore newlines
        break;
        
      default:
        Serial.println("Invalid command. Press 1, 2, or 3.");
        break;
    }
  }
  
  delay(10);
}

void sendCommand(const char* command) {
  CamSerial.println(command);
  CamSerial.flush();
  Serial.printf("Sent command: %s\n", command);
}

String readResponse() {
  unsigned long timeout = millis() + 5000;  // 5 second timeout
  
  while (millis() < timeout) {
    if (CamSerial.available()) {
      String response = CamSerial.readStringUntil('\n');
      response.trim();
      return response;
    }
    delay(10);
  }
  
  return "TIMEOUT";
}

void receiveImage() {
  Serial.println("Waiting for image header...");
  
  unsigned long timeout = millis() + 10000;  // 10 second timeout
  String line;
  String filename = "";
  size_t fileSize = 0;
  
  // Read header
  while (millis() < timeout) {
    if (CamSerial.available()) {
      line = CamSerial.readStringUntil('\n');
      line.trim();
      
      if (line == "--START--") {
        Serial.println("Header received");
        
        // Read filename
        line = CamSerial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("FILE:")) {
          filename = line.substring(5);
          Serial.println("Filename: " + filename);
        }
        
        // Read file size
        line = CamSerial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("SIZE:")) {
          fileSize = line.substring(5).toInt();
          Serial.printf("File size: %d bytes\n", fileSize);
        }
        
        // Read data marker
        line = CamSerial.readStringUntil('\n');
        line.trim();
        if (line == "--DATA--") {
          Serial.println("Starting data reception...");
          break;
        }
      } else if (line == "ERROR") {
        Serial.println("Error: No image available!");
        return;
      }
    }
    delay(10);
  }
  
  if (fileSize == 0) {
    Serial.println("Error: Invalid header or timeout");
    return;
  }
  
  // Receive image data
  size_t totalReceived = 0;
  uint8_t buffer[CHUNK_SIZE];
  unsigned long lastPrint = 0;
  
  Serial.println("Receiving image data...");
  timeout = millis() + 30000;  // 30 second timeout for data
  
  while (totalReceived < fileSize && millis() < timeout) {
    if (CamSerial.available()) {
      size_t bytesToRead = min((size_t)CamSerial.available(), fileSize - totalReceived);
      bytesToRead = min(bytesToRead, (size_t)CHUNK_SIZE);
      
      size_t bytesRead = CamSerial.readBytes(buffer, bytesToRead);
      totalReceived += bytesRead;
      
      // Show progress every second
      if (millis() - lastPrint > 1000 || totalReceived == fileSize) {
        Serial.printf("Received: %d / %d bytes (%.1f%%)\n", 
                      totalReceived, fileSize, (totalReceived * 100.0) / fileSize);
        lastPrint = millis();
      }
      
      timeout = millis() + 30000;  // Reset timeout on each successful read
    }
    delay(1);
  }
  
  // Read end marker
  delay(100);
  if (CamSerial.available()) {
    line = CamSerial.readStringUntil('\n');
    line.trim();
    if (line == "--END--") {
      Serial.println("Transfer complete!");
    }
  }
  
  if (totalReceived == fileSize) {
    Serial.println("=== Image received successfully! ===");
    Serial.printf("Total bytes: %d\n", totalReceived);
  } else {
    Serial.printf("Warning: Expected %d bytes, received %d bytes\n", fileSize, totalReceived);
  }
}
