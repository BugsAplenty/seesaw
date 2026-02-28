#include <Arduino.h>
#include "Config.h"
#include "IMUManager.h"
#include "ServoController.h"
#include "LidarScanner.h"
#include "UDPManager.h"

// ── Single RPlidar instance ──────────────────────────────────────────────────
RPlidar lidar(Serial2);

// ── Subsystem instances ──────────────────────────────────────────────────────
IMUManager      imu;
ServoController servo;
LidarScanner    lidarScanner;
UDPManager      udp;

// ── Shared state (DOUBLE BUFFER) ─────────────────────────────────────────────
static float          g_phi         = 180.0f;

#define POINTS_PER_PACKET 73
#define PAYLOAD_SIZE (POINTS_PER_PACKET * 20) // 1460 bytes, safely under 1472 MTU

static uint8_t        udpBufferA[PAYLOAD_SIZE];
static uint8_t        udpBufferB[PAYLOAD_SIZE];
static uint8_t*       currentBuffer = udpBufferA;
static size_t         currentPoints = 0;

// ── LIDAR point callback ─────────────────────────────────────────────────────
void processLidarPoint(float dist, float angle, byte quality) {
    if (dist <= 0) return;

    uint32_t now_us = micros();
    uint32_t phi_i = (uint32_t)(g_phi * 100);  
    uint32_t angle_i = (uint32_t)(angle * 100);
    uint32_t dist_i = (uint32_t)dist;

    size_t offset = currentPoints * 20;
    memcpy(&currentBuffer[offset + 0], &now_us, 4);
    memcpy(&currentBuffer[offset + 4], &phi_i, 4);
    memcpy(&currentBuffer[offset + 8], &angle_i, 4);
    memcpy(&currentBuffer[offset +12], &dist_i, 4);
    currentBuffer[offset +16] = quality;
    currentBuffer[offset +17] = 0;
    currentBuffer[offset +18] = 0;
    currentBuffer[offset +19] = 0;
    
    currentPoints++;

    // Swap and Send when exactly full
    if (currentPoints >= POINTS_PER_PACKET) {
        uint8_t* bufferToSend = currentBuffer;
        
        // Swap buffers instantly
        currentBuffer = (currentBuffer == udpBufferA) ? udpBufferB : udpBufferA;
        currentPoints = 0; 
        
        // Send payload
        udp.sendLidarPoints(bufferToSend, PAYLOAD_SIZE);
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  imu.init();
  servo.init();
  lidarScanner.init();
  udp.init();  
}

void loop() {
  static unsigned long last_imu_send = 0;
  
  bool new_imu_data = imu.update(); 
  
  if (new_imu_data) {
      g_phi = imu.getFilteredAngle();
      
      // Send IMU every 5ms
      if (millis() - last_imu_send >= 5) {
          IMUData data = imu.getData();
          uint8_t imuPayload[UDP_PACKET_SIZE_IMU]; 
          uint32_t ts_ms = millis();
          
          memcpy(&imuPayload[0],  &ts_ms, 4);
          memcpy(&imuPayload[4],  &data.accelX, 4);
          memcpy(&imuPayload[8],  &data.accelY, 4);
          memcpy(&imuPayload[12], &data.accelZ, 4);
          memcpy(&imuPayload[16], &data.gyroX, 4);
          memcpy(&imuPayload[20], &data.gyroY, 4);
          memcpy(&imuPayload[24], &data.gyroZ, 4);
          
          udp.sendImuData(imuPayload, UDP_PACKET_SIZE_IMU);
          last_imu_send = millis();
      }
  }
  
  servo.update(g_phi);
  udp.update(); 
  int decode_count = 0;
  while (lidarScanner.update() > 0 && decode_count < 100) {
      decode_count++;
  }
}
