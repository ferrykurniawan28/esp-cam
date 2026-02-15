#ifndef UART_IMAGE_TRANSFER_H
#define UART_IMAGE_TRANSFER_H

#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"

// Protocol commands
#define CMD_TAKE_PHOTO "TAKE_PHOTO"
#define CMD_SEND_IMAGE "SEND_IMAGE"
#define CMD_GET_COUNT "GET_COUNT"
#define RESPONSE_OK "OK"
#define RESPONSE_ERROR "ERROR"

// Transfer settings
#define CHUNK_SIZE 1024  // Send 1KB at a time
#define UART_BAUD 115200

class UARTImageTransfer {
public:
  UARTImageTransfer(HardwareSerial &serial);
  
  // Initialize UART
  void begin(unsigned long baudRate = UART_BAUD);
  
  // Send image file via UART
  bool sendImageFile(const char* filepath);
  bool sendLatestImage(int pictureNumber);
  
  // Check for and parse commands
  String readCommand();
  
  // Send responses
  void sendResponse(const char* response);
  void sendOK();
  void sendError();
  
  // Send picture count
  void sendPictureCount(int count);
  
private:
  HardwareSerial &uart;
  
  // Helper to send file info header
  void sendFileHeader(const char* filename, size_t fileSize);
};

#endif // UART_IMAGE_TRANSFER_H
