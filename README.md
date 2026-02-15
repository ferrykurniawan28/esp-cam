# ESP32-CAM UART Image Transfer

This project enables ESP32-CAM to send images and receive commands via UART from another ESP32.

## Features

- Take photos via UART command
- Send images via UART to another ESP
- Auto-numbered image saving to SD card
- Simple text-based protocol
- Progress reporting during transfer

## Hardware Setup

### ESP32-CAM Connections
```
ESP32-CAM (Sender)     →     Other ESP32 (Receiver)
GPIO17 (TX2)           →     GPIO16 (RX2)
GPIO16 (RX2)           →     GPIO17 (TX2)
GND                    →     GND
```

**Important**: These GPIO pins are available on ESP32-CAM and don't conflict with the camera!

## UART Protocol

### Commands (send to ESP32-CAM)
- `TAKE_PHOTO` - Take a new photo and save to SD card
- `SEND_IMAGE` - Send the latest image via UART
- `GET_COUNT` - Get current picture count

### Responses
- `OK` - Command successful
- `ERROR` - Command failed
- `COUNT:X` - Picture count response

### Image Transfer Format
```
--START--
FILE:/pictureX.jpg
SIZE:12345
--DATA--
[binary image data]
--END--
```

## Usage

### 1. ESP32-CAM (Sender) Setup

Upload the main code to ESP32-CAM:
```cpp
#include <Arduino.h>
#include "ESP32CamSD.h"
#include "UARTImageTransfer.h"

ESP32CamSD camera;
UARTImageTransfer uartTransfer(Serial2);

void setup() {
  Serial.begin(115200);
  camera.begin();
  uartTransfer.begin(115200);
}

void loop() {
  String command = uartTransfer.readCommand();
  
  if (command == CMD_TAKE_PHOTO) {
    if (camera.takePhotoSaveSD()) {
      uartTransfer.sendOK();
    }
  }
  else if (command == CMD_SEND_IMAGE) {
    uartTransfer.sendLatestImage(camera.getPictureNumber());
  }
}
```

### 2. Receiver ESP32 Setup

Use the example code in `examples/uart_receiver_example.cpp`

Open Serial Monitor and send:
- `1` - Take photo
- `2` - Receive image
- `3` - Get count

## Files Structure

```
esp-cam/
├── include/
│   ├── ESP32CamSD.h          # Camera & SD card management
│   └── UARTImageTransfer.h   # UART image transfer
├── src/
│   ├── main.cpp              # Main ESP32-CAM code
│   ├── ESP32CamSD.cpp
│   └── UARTImageTransfer.cpp
└── examples/
    └── uart_receiver_example.cpp  # Example receiver code
```

## Transfer Speed

- **Baud Rate**: 115200 bps
- **Chunk Size**: 1KB
- **Typical Speed**: ~11 KB/s
- **VGA Image (30KB)**: ~3 seconds
- **SVGA Image (50KB)**: ~5 seconds
- **XGA Image (80KB)**: ~7 seconds

## Troubleshooting

### No response from ESP32-CAM
- Check wiring (TX→RX, RX→TX)
- Verify common ground
- Check baud rate (115200)
- Ensure commands end with newline `\n`

### Image transfer incomplete
- Check UART buffer size
- Increase timeout values
- Reduce image quality/size
- Check for loose connections

### Camera initialization fails
- Verify SD card is inserted
- Check camera connections
- Try formatting SD card (FAT32)
- Reset ESP32-CAM

## Advanced Usage

### Change Image Quality
```cpp
camera.setFrameSize(FRAMESIZE_SVGA);  // QVGA, VGA, SVGA, XGA
camera.setJpegQuality(10);            // 0-63, lower = better
```

### Custom Filename
```cpp
camera.takePhotoSaveSD("/custom.jpg");
```

### Direct Sensor Access
```cpp
sensor_t* sensor = camera.getSensor();
sensor->set_brightness(sensor, 1);
sensor->set_contrast(sensor, 1);
```

## Integration with Cat Activity Tracker

To combine with the cat activity tracker, trigger photos based on activity:

```cpp
void loop() {
  tracker.update();
  
  // Take photo when cat is very active
  if (tracker.getRunningCount() > 5) {
    camera.takePhotoSaveSD();
    tracker.resetCounters();
  }
  
  // Handle UART commands
  String command = uartTransfer.readCommand();
  // ... handle commands
}
```

## License

Based on example code from RandomNerdTutorials.com
Modified for UART image transfer capabilities
