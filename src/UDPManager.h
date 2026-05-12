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
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        lastConnectAttempt = millis();
        Serial.printf("[UDP] Connecting to WiFi SSID=%s\n", WIFI_SSID);
    }

    bool update() {
        wl_status_t st = WiFi.status();

        if (st != WL_CONNECTED) {
            if (connected) {
                connected = false;
                Serial.printf("[UDP] WiFi lost, status=%d\n", st);
            }

            if (millis() - lastConnectAttempt > 10000) {
                Serial.printf("[UDP] Reconnecting WiFi, status=%d\n", st);
                WiFi.disconnect(true, false);
                delay(100);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                lastConnectAttempt = millis();
            }
            return false;
        }

        if (!connected) {
            if (!lidarUdp.begin(UDP_LOCAL_PORT_LIDAR)) {
                Serial.printf("[UDP] lidarUdp.begin(%d) failed\n", UDP_LOCAL_PORT_LIDAR);
                return false;
            }
            if (!imuUdp.begin(UDP_LOCAL_PORT_IMU)) {
                Serial.printf("[UDP] imuUdp.begin(%d) failed\n", UDP_LOCAL_PORT_IMU);
                return false;
            }

            String localIpStr = WiFi.localIP().toString();
            String remoteIpStr = UDP_REMOTE_IP.toString();

            Serial.printf("[UDP] WiFi connected. IP: %s RSSI=%d\n", localIpStr.c_str(), WiFi.RSSI());
            Serial.printf("[UDP] Lidar: local %d -> %s:%d\n", UDP_LOCAL_PORT_LIDAR, remoteIpStr.c_str(), UDP_REMOTE_PORT_LIDAR);
            Serial.printf("[UDP] IMU:   local %d -> %s:%d\n", UDP_LOCAL_PORT_IMU, remoteIpStr.c_str(), UDP_REMOTE_PORT_IMU);
            connected = true;
        }

        return true;
    }

    bool isConnected() const {
        return connected;
    }

    void sendImuData(const uint8_t* data, size_t len) {
        if (!connected) return;
        if (!lidarOrImuSend(imuUdp, UDP_REMOTE_IP, UDP_REMOTE_PORT_IMU, data, len, "IMU")) {
            connected = false;
        }
    }

    void sendLidarPoints(const uint8_t* data, size_t len) {
        if (!connected) return;
        if (!lidarOrImuSend(lidarUdp, UDP_REMOTE_IP, UDP_REMOTE_PORT_LIDAR, data, len, "LIDAR")) {
            connected = false;
        }
    }

    void sendTestPacket(const uint8_t* data, size_t len) {
        if (!connected) return;
        lidarOrImuSend(lidarUdp, UDP_REMOTE_IP, UDP_REMOTE_PORT_LIDAR, data, len, "TEST");
    }

private:
    bool lidarOrImuSend(WiFiUDP& udp, const IPAddress& ip, uint16_t port, const uint8_t* data, size_t len, const char* tag) {
        if (!udp.beginPacket(ip, port)) {
            Serial.printf("[UDP] %s beginPacket failed\n", tag);
            return false;
        }

        size_t written = udp.write(data, len);
        if (written != len) {
            Serial.printf("[UDP] %s write short: %u/%u\n", tag, (unsigned)written, (unsigned)len);
        }

        if (!udp.endPacket()) {
            Serial.printf("[UDP] %s endPacket failed\n", tag);
            return false;
        }

        return written == len;
    }
};