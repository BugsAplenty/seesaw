#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"

class UDPManager {
private:
    WiFiUDP udp;
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
            udp.begin(UDP_LOCAL_PORT);
            Serial.printf("[UDP] WiFi connected. IP: %s. UDP ready on port %d\n", 
                          WiFi.localIP().toString().c_str(), UDP_LOCAL_PORT);
            connected = true;
        }
        return true;
    }

    void sendLidarPoints(const uint8_t* data, size_t len) {
        udp.beginPacket(UDP_REMOTE_IP, UDP_REMOTE_PORT);
        udp.write(data, len);
        udp.endPacket();
    }
};
