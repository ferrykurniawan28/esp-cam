#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"

// Try to include face detection if available
#if CONFIG_ESP_FACE_DETECTION_ENABLED
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp" 
#define FACE_DETECTION_ENABLED 1
#else
#define FACE_DETECTION_ENABLED 0
#endif

// Camera pin definitions for AI-Thinker ESP32-CAM
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

// WiFi credentials
const char* ssid = "NETWORK_NAME";
const char* password = "PASSWORD";

// Web server on port 80
WebServer server(80);

// LED pin for flash (GPIO 4)
#define FLASH_LED 4

// Face detection variables
bool faceDetectionEnabled = true;

// Simplified face detection function (placeholder for now)
bool detectFaces(camera_fb_t *fb, int &face_count) {
  if (!fb || !faceDetectionEnabled) {
    face_count = 0;
    return false;
  }
  
#if FACE_DETECTION_ENABLED
  // This would contain actual face detection code when libraries are available
  // For now, we'll simulate detection for demonstration
  face_count = 0; // No faces detected in this simplified version
  return false;
#else
  face_count = 0;
  return false;
#endif
}

// Function to add text overlay (basic version)
void addTextOverlay(camera_fb_t *fb, const char* text) {
  if (!fb) return;
  Serial.printf("Overlay: %s\n", text);
}

void initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size and quality settings (lower resolution for better performance)
  if(psramFound()) {
    config.frame_size = FRAMESIZE_VGA; // 640x480
    config.jpeg_quality = 15;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  Serial.println("Camera initialized successfully!");
  
#if FACE_DETECTION_ENABLED
  Serial.println("Face detection libraries available!");
#else
  Serial.println("Face detection libraries not available - using basic mode");
#endif
}

void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32-CAM</title></head>";
  html += "<body style='font-family: Arial'>";
  html += "<h1>ESP32-CAM Web Server</h1>";
  html += "<p><a href='/capture'>Capture Photo</a></p>";
  html += "<p><a href='/face-detect'>Face Detection Photo</a></p>";
  html += "<p><a href='/stream'>Live Stream</a></p>";
  html += "<p><a href='/stream-faces'>Live Stream with Face Detection</a></p>";
  html += "<p><a href='/flash-on'>Flash ON</a> | <a href='/flash-off'>Flash OFF</a></p>";
  html += "<p>Face Detection: " + String(faceDetectionEnabled ? "ON" : "OFF") + 
          " | <a href='/face-toggle'>Toggle</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
  Serial.println("Photo captured and sent");
}

void handleStream() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32-CAM Stream</title></head>";
  html += "<body style='font-family: Arial'>";
  html += "<h1>ESP32-CAM Live Stream</h1>";
  html += "<img src='/stream-feed' style='width:100%; max-width:800px;'>";
  html += "<p><a href='/'>Back to Home</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStreamFeed() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    
    String head = "--frame\r\n";
    head += "Content-Type: image/jpeg\r\n";
    head += "Content-Length: " + String(fb->len) + "\r\n\r\n";
    
    client.print(head);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    
    esp_camera_fb_return(fb);
    
    if (!client.connected()) break;
    delay(100);
  }
}

void handleFaceDetection() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // Perform face detection (simplified version)
  int face_count = 0;
  bool detected = detectFaces(fb, face_count);
  
  if (detected) {
    Serial.printf("Detected %d faces\n", face_count);
    addTextOverlay(fb, "Face detected!");
  } else {
    Serial.println("No faces detected");
    addTextOverlay(fb, "No faces found");
  }

  server.sendHeader("Content-Disposition", "inline; filename=face_detect.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
  Serial.println("Face detection photo captured and sent");
}

void handleStreamFaces() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32-CAM Face Detection Stream</title></head>";
  html += "<body style='font-family: Arial'>";
  html += "<h1>ESP32-CAM Live Stream with Face Detection</h1>";
  html += "<img src='/stream-faces-feed' style='width:100%; max-width:800px;'>";
  html += "<p><a href='/'>Back to Home</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStreamFacesFeed() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    
    // Perform face detection if enabled
    if (faceDetectionEnabled) {
      int face_count = 0;
      bool detected = detectFaces(fb, face_count);
      if (detected) {
        Serial.printf("Stream: Detected %d faces\n", face_count);
        addTextOverlay(fb, "Face detected!");
      }
    }
    
    String head = "--frame\r\n";
    head += "Content-Type: image/jpeg\r\n";
    head += "Content-Length: " + String(fb->len) + "\r\n\r\n";
    
    client.print(head);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    
    esp_camera_fb_return(fb);
    
    if (!client.connected()) break;
    delay(200); // Slower for face detection processing
  }
}

void handleFaceToggle() {
  faceDetectionEnabled = !faceDetectionEnabled;
  String message = "Face Detection " + String(faceDetectionEnabled ? "ON" : "OFF");
  server.send(200, "text/plain", message);
}

void handleFlashOn() {
  digitalWrite(FLASH_LED, HIGH);
  server.send(200, "text/plain", "Flash ON");
}

void handleFlashOff() {
  digitalWrite(FLASH_LED, LOW);
  server.send(200, "text/plain", "Flash OFF");
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Starting...");
  
  // Initialize flash LED
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);
  
  // Initialize camera
  initCamera();
  
  // Initialize WiFi
  initWiFi();
  
  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/face-detect", handleFaceDetection);
  server.on("/stream", handleStream);
  server.on("/stream-feed", handleStreamFeed);
  server.on("/stream-faces", handleStreamFaces);
  server.on("/stream-faces-feed", handleStreamFacesFeed);
  server.on("/face-toggle", handleFaceToggle);
  server.on("/flash-on", handleFlashOn);
  server.on("/flash-off", handleFlashOff);
  
  // Start server
  server.begin();
  Serial.println("Web server started");
  Serial.println("Open your browser to: http://" + WiFi.localIP().toString());
}

void loop() {
  server.handleClient();
}