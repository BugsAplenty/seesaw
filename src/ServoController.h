#pragma once
#include <Arduino.h>
#include <SCServo.h>
#include "Config.h"

class ServoController {
private:
    SCSCL sc;
    unsigned long lastUpdate = 0;
    bool online = false;

    int currentPos = SERVO_CENTER_POS;
    int step = SERVO_STEP_COUNTS;

    static constexpr float COUNTS_PER_DEG = 1023.0f / 180.0f;

    static int degToCounts(float deg) {
        return (int)(deg * COUNTS_PER_DEG + (deg >= 0 ? 0.5f : -0.5f));
    }

    int minPos() const {
        return SERVO_CENTER_POS - degToCounts(SERVO_SWEEP_HALF_DEG);
    }

    int maxPos() const {
        return SERVO_CENTER_POS + degToCounts(SERVO_SWEEP_HALF_DEG);
    }

public:
    void init() {
        Serial.printf("[SERVO] Serial1 begin @1000000 RX=%d TX=%d ID=%d\n", SERVO_RX_PIN, SERVO_TX_PIN, SERVO_ID);
        Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
        sc.pSerial = &Serial1;
        delay(200);

        int ping = sc.Ping(SERVO_ID);
        online = (ping != -1);

        if (online) {
            Serial.printf("[SERVO] Online. Ping returned ID=%d\n", ping);
            sc.WritePos(SERVO_ID, currentPos, 0, SERVO_MOVE_TIME_MS);
        } else {
            Serial.printf("[SERVO] Ping failed for ID %d. Check power, wiring, ID, and RX/TX mapping.\n", SERVO_ID);
        }
    }

    bool update() {
        if (!online) return false;
        if (millis() - lastUpdate < SERVO_INTERVAL_MS) return false;
        lastUpdate = millis();

        currentPos += step;

        if (currentPos >= maxPos()) {
            currentPos = maxPos();
            step = -abs(step);
        } else if (currentPos <= minPos()) {
            currentPos = minPos();
            step = abs(step);
        }

        sc.WritePos(SERVO_ID, currentPos, 0, SERVO_MOVE_TIME_MS);
        return true;
    }

    float getRelativeAngleDeg() const {
        return (currentPos - SERVO_CENTER_POS) / COUNTS_PER_DEG;
    }

    int getTargetPosition() const {
        return currentPos;
    }

    bool isOnline() const {
        return online;
    }
};