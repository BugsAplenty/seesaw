#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SCServo.h>
#include <thijs_rplidar.h>

// ===================== USER CONFIG =====================

#define UDP_REMOTE_IP         IPAddress(192, 168, 68, 59)
#define UDP_LOCAL_PORT_LIDAR  8888
#define UDP_REMOTE_PORT_LIDAR 12345
#define UDP_LOCAL_PORT_IMU    8889
#define UDP_REMOTE_PORT_IMU   12346
#define UDP_LOCAL_PORT_SERVO   8890
#define UDP_REMOTE_PORT_SERVO  12347


#define IMU_SDA               21
#define IMU_SCL               22
#define IMU_READ_INTERVAL_MS  10
#define IMU_SEND_INTERVAL_MS  20

// ---------------- SERVO ----------------
SMS_STS st;

#define SERVO_RX_PIN 18
#define SERVO_TX_PIN 19
#define SERVO_ID     1

#define POS_LEFT                1100
#define POS_CENTER              2048
#define POS_RIGHT               2600
#define SERVO_SPEED             2400
#define SERVO_ACCEL             30
#define SERVO_DWELL_MS          3000
#define SERVO_SEND_INTERVAL_MS  50

// ---------------- LIDAR ----------------
#define LIDAR_MOTOR_PIN 26
#define LIDAR_RX_PIN    25
#define LIDAR_TX_PIN    14
#define LIDAR_SPINUP_MS 700

RPlidar lidar(Serial2);

// ---------------- UDP / BUFFERS ----------------
WiFiUDP lidarUdp;
WiFiUDP imuUdp;
WiFiUDP servoUdp;
Adafruit_MPU6050 mpu;

#define POINTS_PER_PACKET 73
#define LIDAR_POINT_SIZE  20
#define LIDAR_PAYLOAD_SIZE (POINTS_PER_PACKET * LIDAR_POINT_SIZE)

uint8_t lidarBufA[LIDAR_PAYLOAD_SIZE];
uint8_t lidarBufB[LIDAR_PAYLOAD_SIZE];
uint8_t* currentLidarBuf = lidarBufA;
uint8_t* sendLidarBuf = nullptr;

size_t currentLidarPoints = 0;
volatile bool sendLidarNow = false;
bool wifiReady = false;

// ---------------- STATE ----------------
unsigned long lastServoMoveMs = 0;
unsigned long lastLidarPrintMs = 0;
unsigned long lastServoFeedbackMs = 0;
unsigned long lastServoSendMs = 0;
unsigned long lastImuReadMs = 0;
unsigned long lastImuSendMs = 0;
unsigned long lastWifiRetryMs = 0;

uint8_t servoPhase = 0;
volatile uint32_t lidarPointCount = 0;

float currentServoDeg = 0.0f;
sensors_event_t a, g, temp;

struct __attribute__((packed)) ServoPacket {
    uint32_t t_us;
    float angle_deg;
    int16_t pos;
    uint8_t phase;
    uint8_t reserved[1];
};

// ===================== HELPERS =====================
float servoPosToDeg(int pos) {
    if (pos >= POS_CENTER) {
        return 20.0f * (float)(pos - POS_CENTER) / (float)(POS_RIGHT - POS_CENTER);
    } else {
        return -20.0f * (float)(POS_CENTER - pos) / (float)(POS_CENTER - POS_LEFT);
    }
}

void wifiInit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiRetryMs = millis();
    Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);
}

void wifiUpdate() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiReady) {
            lidarUdp.begin(UDP_LOCAL_PORT_LIDAR);
            imuUdp.begin(UDP_LOCAL_PORT_IMU);
            servoUdp.begin(UDP_LOCAL_PORT_SERVO);
            wifiReady = true;
            Serial.printf("[WIFI] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[UDP] Lidar -> %s:%d\n", UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_LIDAR);
            Serial.printf("[UDP] IMU   -> %s:%d\n", UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_IMU);
            Serial.printf("[UDP] Servo -> %s:%d\n", UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_SERVO);
        }
        return;
    }

    if (wifiReady) {
        wifiReady = false;
        Serial.println("[WIFI] Lost connection");
    }

    if (millis() - lastWifiRetryMs > 10000) {
        Serial.println("[WIFI] Retrying...");
        WiFi.disconnect(true, false);
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        lastWifiRetryMs = millis();
    }
}

void sendUdpPacket(WiFiUDP& udp, uint16_t remotePort, const uint8_t* data, size_t len) {
    if (!wifiReady) return;

    if (udp.beginPacket(UDP_REMOTE_IP, remotePort)) {
        udp.write(data, len);
        udp.endPacket();
    }
}

// ===================== SERVO =====================
void commandServoPhase(uint8_t p) {
    switch (p) {
        case 0:
            Serial.println("[SERVO] -> center");
            st.WritePosEx(SERVO_ID, POS_CENTER, SERVO_SPEED, SERVO_ACCEL);
            break;
        case 1:
            Serial.println("[SERVO] -> right");
            st.WritePosEx(SERVO_ID, POS_RIGHT, SERVO_SPEED, SERVO_ACCEL);
            break;
        case 2:
            Serial.println("[SERVO] -> center");
            st.WritePosEx(SERVO_ID, POS_CENTER, SERVO_SPEED, SERVO_ACCEL);
            break;
        default:
            Serial.println("[SERVO] -> left");
            st.WritePosEx(SERVO_ID, POS_LEFT, SERVO_SPEED, SERVO_ACCEL);
            break;
    }
}

void setupServo() {
    Serial.println("[SERVO] init");

    Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    st.pSerial = &Serial1;

    delay(1000);

    int ping = st.Ping(SERVO_ID);
    Serial.printf("[SERVO] ping=%d\n", ping);

    commandServoPhase(servoPhase);
    lastServoMoveMs = millis();
}

void sendServoTelemetry(float angleDeg, int pos, uint8_t phase) {
    if (!wifiReady) return;

    ServoPacket pkt;
    pkt.t_us = micros();
    pkt.angle_deg = angleDeg;
    pkt.pos = (int16_t)pos;
    pkt.phase = phase;
    pkt.reserved[0] = 0;

    sendUdpPacket(servoUdp, UDP_REMOTE_PORT_SERVO, (const uint8_t*)&pkt, sizeof(pkt));
}

void updateServo() {
    if (millis() - lastServoMoveMs >= SERVO_DWELL_MS) {
        servoPhase = (servoPhase + 1) % 4;
        commandServoPhase(servoPhase);
        lastServoMoveMs = millis();
    }

    if (millis() - lastServoFeedbackMs >= 20) {
        lastServoFeedbackMs = millis();

        if (st.FeedBack(SERVO_ID) != -1) {
            int pos = st.ReadPos(-1);
            currentServoDeg = servoPosToDeg(pos);

            if (millis() - lastServoSendMs >= SERVO_SEND_INTERVAL_MS) {
                lastServoSendMs = millis();
                sendServoTelemetry(currentServoDeg, pos, servoPhase);
            }
        }
    }
}

// ===================== LIDAR =====================
void processLidarPoint(float dist, float angle, byte quality) {
    if (dist <= 0) return;

    lidarPointCount++;

    uint32_t now_us  = micros();
    uint32_t angle_i = (uint32_t)(angle * 100.0f);
    uint32_t dist_i  = (uint32_t)dist;

    size_t offset = currentLidarPoints * LIDAR_POINT_SIZE;
    memcpy(&currentLidarBuf[offset + 0],  &now_us,  4);
    memcpy(&currentLidarBuf[offset + 8],  &angle_i, 4);
    memcpy(&currentLidarBuf[offset + 12], &dist_i,  4);
    currentLidarBuf[offset + 16] = quality;
    currentLidarBuf[offset + 17] = 0;
    currentLidarBuf[offset + 18] = 0;
    currentLidarBuf[offset + 19] = 0;

    currentLidarPoints++;

    if (currentLidarPoints >= POINTS_PER_PACKET) {
        sendLidarBuf = currentLidarBuf;
        currentLidarBuf = (currentLidarBuf == lidarBufA) ? lidarBufB : lidarBufA;
        currentLidarPoints = 0;
        sendLidarNow = true;
    }
}

void setupLidar() {
    Serial.println("[LIDAR] init");

    pinMode(LIDAR_MOTOR_PIN, OUTPUT);
    digitalWrite(LIDAR_MOTOR_PIN, HIGH);
    delay(LIDAR_SPINUP_MS);

    Serial2.setRxBufferSize(2048);
    lidar.init(LIDAR_RX_PIN, LIDAR_TX_PIN);

    lidar.printLidarInfo();

    if (lidar.connectionCheck()) {
        Serial.println("[LIDAR] connectionCheck OK");
    } else {
        Serial.println("[LIDAR] connectionCheck FAILED");
    }

    lidar.postParseCallback = [](RPlidar* ptr,
                                 uint16_t dist,
                                 uint16_t angle_q6,
                                 uint8_t newRotFlag,
                                 int8_t quality) {
        (void)ptr;
        (void)newRotFlag;

        if (dist > 0) {
            processLidarPoint((float)dist, angle_q6 / 64.0f, (byte)quality);
        }
    };

    Serial.println("[LIDAR] startExpressScan(DENSE)...");
    if (!lidar.startExpressScan(2)) {
        Serial.println("[LIDAR] express failed -> standard scan");
        if (!lidar.startStandardScan()) {
            Serial.println("[LIDAR] standard scan failed");
        }
    } else {
        Serial.println("[LIDAR] express started");
    }
}

void updateLidar() {
    int decode_count = 0;
    while (decode_count < 100) {
        int n = lidar.handleData(false, false);
        if (n <= 0) break;
        decode_count++;
    }

    if (sendLidarNow && sendLidarBuf != nullptr) {
        sendUdpPacket(lidarUdp, UDP_REMOTE_PORT_LIDAR, sendLidarBuf, LIDAR_PAYLOAD_SIZE);
        sendLidarNow = false;
    }
}

// ===================== IMU =====================
void setupImu() {
    Serial.println("[IMU] init");
    Wire.begin(IMU_SDA, IMU_SCL);

    if (!mpu.begin()) {
        Serial.println("[IMU] MPU6050 init failed!");
        return;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[IMU] MPU6050 initialized");
}

void updateImu() {
    if (millis() - lastImuReadMs >= IMU_READ_INTERVAL_MS) {
        lastImuReadMs = millis();
        mpu.getEvent(&a, &g, &temp);
    }

    if (millis() - lastImuSendMs >= IMU_SEND_INTERVAL_MS) {
        lastImuSendMs = millis();

        uint8_t buf[28];
        uint32_t now_us = micros();

        memcpy(&buf[0],  &now_us,             4);
        memcpy(&buf[4],  &a.acceleration.x,   4);
        memcpy(&buf[8],  &a.acceleration.y,   4);
        memcpy(&buf[12], &a.acceleration.z,   4);
        memcpy(&buf[16], &g.gyro.x,           4);
        memcpy(&buf[20], &g.gyro.y,           4);
        memcpy(&buf[24], &g.gyro.z,           4);

        sendUdpPacket(imuUdp, UDP_REMOTE_PORT_IMU, buf, sizeof(buf));

        static unsigned long lastImuPrintMs = 0;
        if (millis() - lastImuPrintMs >= 500) {
            lastImuPrintMs = millis();
            Serial.printf("[IMU] ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f\n",
                          a.acceleration.x, a.acceleration.y, a.acceleration.z,
                          g.gyro.x, g.gyro.y, g.gyro.z);
        }
    }
}

// ===================== ARDUINO =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("[BOOT] Servo + Lidar + IMU + UDP");

    setupServo();
    setupLidar();
    setupImu();
    wifiInit();
}

void loop() {
    wifiUpdate();
    updateServo();
    updateLidar();
    // updateImu();
}