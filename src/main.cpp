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
static uint8_t        udpBuffer[UDP_PACKET_SIZE];
static size_t         udpBufferLen = 0;
static unsigned long  lastUdpFlush = 0;
// ── LIDAR point callback (called from LidarScanner's postParseCallback) ──────
void processLidarPoint(float dist, float angle, byte quality) {
    // Full fidelity: pack every valid point (dist>0, quality>=10) into UDP buffer
    if (dist <= 0 || quality < 10) return;

    // Binary packet format: [timestamp_us:4][phi:4][angle:4][dist_mm:4][quality:1][pad:3] = 20 bytes/point
    uint32_t now_us = micros();
    uint32_t phi_i = (uint32_t)(g_phi * 100);  // 0.01 deg precision
    uint32_t angle_i = (uint32_t)(angle * 100);
    uint32_t dist_i = (uint32_t)dist;

    if (udpBufferLen + 20 > UDP_PACKET_SIZE) {
        // Buffer full: send immediately
        udp.sendLidarPoints(udpBuffer, udpBufferLen);
        udpBufferLen = 0;
    }

    // Pack point (little-endian)
    memcpy(&udpBuffer[udpBufferLen + 0], &now_us, 4);
    memcpy(&udpBuffer[udpBufferLen + 4], &phi_i, 4);
    memcpy(&udpBuffer[udpBufferLen + 8], &angle_i, 4);
    memcpy(&udpBuffer[udpBufferLen +12], &dist_i, 4);
    udpBuffer[udpBufferLen +16] = quality;
    udpBufferLen += 20;

    // Periodic flush (<5ms latency)
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
  udp.init();  // Add this
}


// ── Arduino loop ─────────────────────────────────────────────────────────────
void loop() {
  imu.update();
  g_phi = imu.getFilteredAngle();
  servo.update(g_phi);
  udp.update();  // Add this

  {
    int decoded = 0;
    do {
      decoded = lidarScanner.update();
    } while (decoded > 0);
  }
}

