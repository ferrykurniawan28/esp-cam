#include "UARTImageTransfer.h"

UARTImageTransfer::UARTImageTransfer(HardwareSerial &serial) 
  : uart(serial) {
}

void UARTImageTransfer::begin(unsigned long baudRate) {
  // UART2: RX=GPIO16 (U2RXD), TX=GPIO17 (U2TXD) - These are not used by camera
  uart.begin(baudRate, SERIAL_8N1, 16, 17);
  Serial.printf("UART initialized at %lu baud on RX=16, TX=17\n", baudRate);
}

bool UARTImageTransfer::sendImageFile(const char* filepath) {
  fs::FS &fs = SD_MMC;
  
  File file = fs.open(filepath, FILE_READ);
  if (!file) {
    Serial.printf("Failed to open file: %s\n", filepath);
    sendError();
    return false;
  }
  
  size_t fileSize = file.size();
  Serial.printf("Sending file: %s, size: %d bytes\n", filepath, fileSize);
  
  // Send file header
  sendFileHeader(filepath, fileSize);
  
  // Send file data in chunks
  uint8_t buffer[CHUNK_SIZE];
  size_t totalSent = 0;
  
  while (file.available()) {
    size_t bytesRead = file.read(buffer, CHUNK_SIZE);
    uart.write(buffer, bytesRead);
    totalSent += bytesRead;
    
    // Show progress
    if (totalSent % (CHUNK_SIZE * 10) == 0) {
      Serial.printf("Sent: %d / %d bytes (%.1f%%)\n", 
                    totalSent, fileSize, (totalSent * 100.0) / fileSize);
    }
  }
  
  file.close();
  
  // Send end marker
  uart.println("\r\n--END--");
  uart.flush();
  
  Serial.printf("Transfer complete: %d bytes sent\n", totalSent);
  return true;
}

bool UARTImageTransfer::sendLatestImage(int pictureNumber) {
  String path = "/picture" + String(pictureNumber) + ".jpg";
  return sendImageFile(path.c_str());
}

String UARTImageTransfer::readCommand() {
  if (uart.available()) {
    String command = uart.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.printf("Received command: %s\n", command.c_str());
      return command;
    }
  }
  return "";
}

void UARTImageTransfer::sendResponse(const char* response) {
  uart.println(response);
  uart.flush();
  Serial.printf("Sent response: %s\n", response);
}

void UARTImageTransfer::sendOK() {
  sendResponse(RESPONSE_OK);
}

void UARTImageTransfer::sendError() {
  sendResponse(RESPONSE_ERROR);
}

void UARTImageTransfer::sendPictureCount(int count) {
  String response = "COUNT:" + String(count);
  sendResponse(response.c_str());
}

void UARTImageTransfer::sendFileHeader(const char* filename, size_t fileSize) {
  // Send header with file info
  uart.println("--START--");
  uart.print("FILE:");
  uart.println(filename);
  uart.print("SIZE:");
  uart.println(fileSize);
  uart.println("--DATA--");
  uart.flush();
  
  Serial.println("File header sent");
}
