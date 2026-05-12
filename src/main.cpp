#include <Arduino.h>
#include "Config.h"
#include "ServoController.h"
#include "LidarScanner.h"
#include "UDPManager.h"

RPlidar lidar(Serial2);

ServoController servo;
LidarScanner    lidarScanner;
UDPManager      udp;

static float g_phi = 0.0f;

#define POINTS_PER_PACKET 73
#define BYTES_PER_POINT   20
#define PAYLOAD_SIZE      (POINTS_PER_PACKET * BYTES_PER_POINT)

static uint8_t  udpBufferA[PAYLOAD_SIZE];
static uint8_t  udpBufferB[PAYLOAD_SIZE];
static uint8_t* currentBuffer = udpBufferA;
static size_t   currentPoints = 0;

static volatile bool     sendLidarNow = false;
static uint8_t*          bufferToSend = nullptr;
static volatile uint32_t g_lidarPointCount = 0;
static uint32_t          g_lastStatsMs = 0;
static uint32_t          g_lastForcedUdpMs = 0;

void processLidarPoint(float dist, float angle, byte quality) {
    if (dist <= 0.0f) return;

    g_lidarPointCount++;

    uint32_t now_us  = micros();
    uint32_t phi_i   = (uint32_t)((g_phi + 180.0f) * 100.0f);
    uint32_t angle_i = (uint32_t)(angle * 100.0f);
    uint32_t dist_i  = (uint32_t)dist;

    size_t offset = currentPoints * BYTES_PER_POINT;
    memcpy(&currentBuffer[offset + 0],  &now_us,  4);
    memcpy(&currentBuffer[offset + 4],  &phi_i,   4);
    memcpy(&currentBuffer[offset + 8],  &angle_i, 4);
    memcpy(&currentBuffer[offset + 12], &dist_i,  4);
    currentBuffer[offset + 16] = quality;
    currentBuffer[offset + 17] = 0;
    currentBuffer[offset + 18] = 0;
    currentBuffer[offset + 19] = 0;

    currentPoints++;

    if (currentPoints >= POINTS_PER_PACKET) {
        bufferToSend = currentBuffer;
        currentBuffer = (currentBuffer == udpBufferA) ? udpBufferB : udpBufferA;
        currentPoints = 0;
        sendLidarNow = true;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("[BOOT] Rocking Lidar starting...");

    servo.init();
    lidarScanner.init();
    udp.init();

    g_phi = servo.getRelativeAngleDeg();
    g_lastStatsMs = millis();
    g_lastForcedUdpMs = millis();
}

void loop() {
    if (servo.update()) {
        g_phi = servo.getRelativeAngleDeg();
    }

    for (int i = 0; i < 10; i++) {
        lidarScanner.update();
    }

    udp.update();

    if (sendLidarNow && bufferToSend != nullptr) {
        udp.sendLidarPoints(bufferToSend, PAYLOAD_SIZE);
        sendLidarNow = false;
    }

    if (udp.isConnected() && millis() - g_lastForcedUdpMs >= 1000) {
        const char* msg = "lidar-alive";
        udp.sendTestPacket((const uint8_t*)msg, strlen(msg));
        g_lastForcedUdpMs = millis();
    }

    if (millis() - g_lastStatsMs >= 1000) {
        Serial.printf(
            "[STAT] servo_online=%d pos=%d phi=%.2f wifi=%d udp=%d lidar_points=%lu partial=%u queued=%d\n",
            servo.isOnline(),
            servo.getTargetPosition(),
            g_phi,
            WiFi.status(),
            udp.isConnected(),
            (unsigned long)g_lidarPointCount,
            (unsigned)currentPoints,
            sendLidarNow ? 1 : 0
        );
        g_lastStatsMs = millis();
    }
}