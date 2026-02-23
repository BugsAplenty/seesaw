#pragma once
#include <ESP32Servo.h>
#include "Config.h"

class ServoController {
private:
    float targetAngle    = 150.0f;
    float targetIncrement = 1.5f;
    unsigned long lastUpdate = 0;

    static constexpr float TARGET_MIN = 143.0f;
    static constexpr float TARGET_MAX = 210.0f;

    static constexpr float ANGLE_MIN = 70.0f;
    static constexpr float ANGLE_MAX = 130.0f;
    static constexpr float ANGLE_MID = 100.0f;

    static constexpr float Kp = 0.8f;

    float currentAngle = ANGLE_MID;

public:
    Servo rockingServo;

    void init() {
        pinMode(SERVO_PIN, OUTPUT);
        digitalWrite(SERVO_PIN, LOW);
        delay(100);
        rockingServo.setPeriodHertz(50);
        rockingServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
        rockingServo.write((int)currentAngle);
        delay(300);
    }

    bool update(float filteredAngle) {
        if (millis() - lastUpdate < SERVO_INTERVAL_MS) return false;
        lastUpdate = millis();

        targetAngle += targetIncrement;
        if (targetAngle >= TARGET_MAX) { targetAngle = TARGET_MAX; targetIncrement = -fabsf(targetIncrement); }
        if (targetAngle <= TARGET_MIN) { targetAngle = TARGET_MIN; targetIncrement =  fabsf(targetIncrement); }

        float error  = targetAngle - filteredAngle;
        currentAngle = constrain(ANGLE_MID + Kp * error, ANGLE_MIN, ANGLE_MAX);
        rockingServo.write((int)currentAngle);
        return true;
    }

    float getTargetAngle() { return targetAngle; }
    int   getAngle()       { return (int)currentAngle; }
};
