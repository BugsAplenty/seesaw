#include <Arduino.h>
#include "Config.h"
#include "IMUManager.h"
#include "ServoController.h"
#include "LidarScanner.h"
#include "UDPManager.h"

// ── Single RPlidar instance ──────────────────────────────────────────────────
// Declared extern in LidarScanner.h; defined exactly once here.
RPlidar lidar(Serial2);

// ── Subsystem instances ──────────────────────────────────────────────────────
IMUManager      imu;
ServoController servo;
LidarScanner    lidarScanner;
UDPManager      udp;

// ── Shared state ─────────────────────────────────────────────────────────────
static float          g_phi         = 180.0f;
static uint8_t        udpBuffer[UDP_PACKET_SIZE_LIDAR];
static size_t         udpBufferLen = 0;
static unsigned long  lastUdpFlush = 0;

// ── LIDAR point callback (called from LidarScanner's postParseCallback) ──────
void processLidarPoint(float dist, float angle, byte quality) {
    if (dist <= 0 ) return;

    uint32_t now_us = micros();
    uint32_t phi_i = (uint32_t)(g_phi * 100);  
    uint32_t angle_i = (uint32_t)(angle * 100);
    uint32_t dist_i = (uint32_t)dist;

    // FIX: Check against Lidar specific size
    if (udpBufferLen + 20 > UDP_PACKET_SIZE_LIDAR) {
        udp.sendLidarPoints(udpBuffer, udpBufferLen);
        udpBufferLen = 0;
    }

    memcpy(&udpBuffer[udpBufferLen + 0], &now_us, 4);
    memcpy(&udpBuffer[udpBufferLen + 4], &phi_i, 4);
    memcpy(&udpBuffer[udpBufferLen + 8], &angle_i, 4);
    memcpy(&udpBuffer[udpBufferLen +12], &dist_i, 4);
    udpBuffer[udpBufferLen +16] = quality;
    udpBuffer[udpBufferLen +17] = 0;
    udpBuffer[udpBufferLen +18] = 0;
    udpBuffer[udpBufferLen +19] = 0;
    
    udpBufferLen += 20;

    if (millis() - lastUdpFlush > 2) {
        if (udpBufferLen > 0) {
            udp.sendLidarPoints(udpBuffer, udpBufferLen);
            udpBufferLen = 0;
        }
        lastUdpFlush = millis();
    }
}

// ── Arduino setup ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  imu.init();
  servo.init();
  lidarScanner.init();
  udp.init();  
}

// ── Arduino loop ─────────────────────────────────────────────────────────────
void loop() {
  static unsigned long last_imu_send = 0;
  static unsigned long last_lidar_debug = 0;
  
  bool new_imu_data = imu.update(); 
  
  if (new_imu_data) {
      g_phi = imu.getFilteredAngle();
      
      // 🚨 THROTTLE: Only send every 5ms to match ROS 2 200Hz publish rate
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
          
          // Serial.print("[IMU SEND] ts=");
          // Serial.print(ts_ms);
          // Serial.print(" accel=[");
          // Serial.print(data.accelX, 2);
          // Serial.print(",");
          // Serial.print(data.accelY, 2);
          // Serial.print(",");
          // Serial.print(data.accelZ, 2);
          // Serial.println("]");
          
          last_imu_send = millis();
      }
  }
  
  servo.update(g_phi);
  udp.update(); 

  {
    int decoded = 0;
    do {
      decoded = lidarScanner.update();
    } while (decoded > 0);
  }
}

