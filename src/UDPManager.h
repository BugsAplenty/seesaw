#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"

class UDPManager {
private:
    WiFiUDP lidarUdp;
    WiFiUDP imuUdp;
    unsigned long lastConnectAttempt = 0;
    bool connected = false;

public:
    void init() {
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        Serial.printf("[UDP] Connecting to WiFi %s...\n", WIFI_SSID);
    }

    bool update() {
        if (WiFi.status() != WL_CONNECTED) {
            if (millis() - lastConnectAttempt > 10000) {
                WiFi.disconnect();
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                lastConnectAttempt = millis();
            }
            return false;
        }
        if (!connected) {
            lidarUdp.begin(UDP_LOCAL_PORT_LIDAR);
            imuUdp.begin(UDP_LOCAL_PORT_IMU);
            Serial.printf("[UDP] WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[UDP] Lidar: local %d -> %s:%d\n", UDP_LOCAL_PORT_LIDAR, 
                          UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_LIDAR);
            Serial.printf("[UDP] IMU:   local %d -> %s:%d\n", UDP_LOCAL_PORT_IMU, 
                          UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_IMU);
            connected = true;
        }
        return true;
    }

    void sendImuData(const uint8_t* data, size_t len) {
        if (!connected) return;  // ← guard
        imuUdp.beginPacket(UDP_REMOTE_IP, UDP_REMOTE_PORT_IMU);
        imuUdp.write(data, len);
        imuUdp.endPacket();
    }

    void sendLidarPoints(const uint8_t* data, size_t len) {
        if (!connected) return;  // ← guard
        lidarUdp.beginPacket(UDP_REMOTE_IP, UDP_REMOTE_PORT_LIDAR);
        lidarUdp.write(data, len);
        lidarUdp.endPacket();
    }
};
