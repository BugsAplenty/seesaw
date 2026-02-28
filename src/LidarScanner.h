#pragma once
#include <Arduino.h>
#include <thijs_rplidar.h>
#include "Config.h"

// Defined in main.cpp — one instance only
extern RPlidar lidar;

#define EXPRESS_SCAN_WORKING_MODE_DENSE 2

// Implemented in main.cpp
extern void processLidarPoint(float dist, float angle, byte quality);

class LidarScanner {
public:
  void init() {
    Serial.println("[LIDAR] init()");

    pinMode(LIDAR_MOTOR_PIN, OUTPUT);
    digitalWrite(LIDAR_MOTOR_PIN, HIGH);
    Serial.println("[LIDAR] motor pin HIGH, spinning up...");
    delay(LIDAR_SPINUP_MS);
    Serial2.setRxBufferSize(2048);
    Serial.printf("[LIDAR] Serial2 init on RX=%d TX=%d @115200\n", LIDAR_RX_PIN, LIDAR_TX_PIN);
    lidar.init(LIDAR_RX_PIN, LIDAR_TX_PIN);

    lidar.printLidarInfo();
    if (lidar.connectionCheck()) Serial.println("[LIDAR] connectionCheck OK");
    else                         Serial.println("[LIDAR] connectionCheck FAILED");

    lidar.postParseCallback = [](RPlidar* ptr,
                                  uint16_t dist,
                                  uint16_t angle_q6,
                                  uint8_t  newRotFlag,
                                  int8_t   quality) {
      (void)ptr; (void)newRotFlag;
      // static int raw_count = 0;
      // raw_count++;
      // if (raw_count % 100 == 0) {
      //   Serial.printf("[LIDAR RAW] dist=%d angle=%.1f qual=%d\n", 
      //                 dist, angle_q6 / 64.0f, quality);
      // }
      if (dist > 0) {
        processLidarPoint((float)dist, angle_q6 / 64.0f, (byte)quality);
      }
    };

    Serial.println("[LIDAR] startExpressScan(DENSE)...");
    if (!lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_DENSE)) {
      Serial.println("[LIDAR] express failed -> startStandardScan()");
      lidar.startStandardScan();
    } else {
      Serial.println("[LIDAR] express started");
    }
  }

  int update() {
    return lidar.handleData(false, false);
  }
};
