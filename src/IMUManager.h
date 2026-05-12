#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "Config.h"

struct IMUData {
    float accelX, accelY, accelZ;
    float gyroX,  gyroY,  gyroZ;
};

static inline float wrap360(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static inline float angleDiffDeg(float target, float current) {
    float d = wrap360(target) - wrap360(current);
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

class IMUManager {
private:
    Adafruit_MPU6050 mpu;
    unsigned long lastRead = 0;
    IMUData data{};
    float filteredAngle = 0.0f;
    bool hasInit = false;

public:
    void init() {
        Wire.begin(IMU_SDA, IMU_SCL);
        if (!mpu.begin()) {
            Serial.println("[IMU] MPU6050 init failed!");
            while (1) delay(1000);
        }
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        lastRead = millis();
        Serial.println("[IMU] MPU6050 initialized");
    }

    bool update() {
        if (millis() - lastRead < IMU_INTERVAL_MS) return false;

        unsigned long now = millis();
        float dt = (now - lastRead) / 1000.0f;
        lastRead = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        if (a.acceleration.x == 0.0f && a.acceleration.y == 0.0f && a.acceleration.z == 0.0f &&
            g.gyro.x == 0.0f && g.gyro.y == 0.0f && g.gyro.z == 0.0f) {
            Serial.println("[IMU] Suspicious all-zero read; reinitializing I2C and MPU6050");
            Wire.end();
            delay(10);
            Wire.begin(IMU_SDA, IMU_SCL);
            mpu.begin();
            return false;
        }

        data.accelX = a.acceleration.x;
        data.accelY = a.acceleration.y;
        data.accelZ = a.acceleration.z;
        data.gyroX  = g.gyro.x;
        data.gyroY  = g.gyro.y;
        data.gyroZ  = g.gyro.z;

        float accelAngle = atan2(data.accelX, data.accelZ) * 180.0f / PI;
        float accel360   = wrap360(accelAngle);

        if (!hasInit) {
            filteredAngle = accel360;
            hasInit = true;
            return true;
        }

        float gyroDegPerSec = data.gyroY * 180.0f / PI;
        float pred = wrap360(filteredAngle + gyroDegPerSec * dt);
        float err  = angleDiffDeg(accel360, pred);
        filteredAngle = wrap360(pred + (1.0f - COMPLEMENTARY_ALPHA) * err);

        return true;
    }

    float getFilteredAngle() const { return filteredAngle; }
    IMUData getData() const        { return data; }
};