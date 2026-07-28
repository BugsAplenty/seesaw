#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SCServo.h>
#include <thijs_rplidar.h>

// ===================== USER CONFIG =====================
#define UDP_REMOTE_IP         IPAddress(255, 255, 255, 255)
#define UDP_LOCAL_PORT_LIDAR  8888
#define UDP_REMOTE_PORT_LIDAR 12345

// ---------------- SERVO ----------------
SCSCL sc;

#define SERVO_RX_PIN 18
#define SERVO_TX_PIN 19
#define SERVO_ID     1

#define POS_START    600
#define POS_END      900
#define SERVO_SPEED  200

// ---------------- LIDAR ----------------
#define LIDAR_MOTOR_PIN 26
#define LIDAR_RX_PIN    25
#define LIDAR_TX_PIN    14
#define LIDAR_SPINUP_MS 700

RPlidar lidar(Serial2);

// ---------------- UDP / BUFFERS ----------------
WiFiUDP lidarUdp;

// 16-byte raw packed point:
//   uint32_t t_us
//   uint16_t servo_raw
//   uint16_t angle_q6
//   uint16_t dist_mm
//   uint8_t  new_rot_flag
//   uint8_t  quality
//   uint32_t reserved
#define LIDAR_POINT_SIZE 16
#define POINTS_PER_PACKET 90
#define LIDAR_PAYLOAD_SIZE (POINTS_PER_PACKET * LIDAR_POINT_SIZE)
#define LIDAR_FLUSH_INTERVAL_US 8000

uint8_t lidarBufA[LIDAR_PAYLOAD_SIZE];
uint8_t lidarBufB[LIDAR_PAYLOAD_SIZE];
uint8_t* currentLidarBuf = lidarBufA;
uint8_t* sendLidarBuf = nullptr;

size_t currentLidarPoints = 0;
volatile bool sendLidarNow = false;
bool wifiReady = false;

// ---------------- STATE ----------------
unsigned long lastLidarPrintMs = 0;
unsigned long lastServoFeedbackMs = 0;
unsigned long lastWifiRetryMs = 0;

uint8_t servoPhase = 0;
volatile uint32_t lidarPointCount = 0;
volatile uint32_t lidarPacketCount = 0;
volatile uint32_t lidarFlushPartialCount = 0;
volatile uint32_t lidarHandleCalls = 0;
volatile uint32_t lidarHandlePositive = 0;

int currentServoPos = POS_START;
uint32_t lastLidarFlushUs = 0;

// ===================== HELPERS =====================
uint16_t clampServoRaw(int pos) {
    if (pos < 0) return 0;
    if (pos > 1023) return 1023;
    return (uint16_t)pos;
}

void wifiInit() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiRetryMs = millis();
    Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);
}

void wifiUpdate() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiReady) {
            lidarUdp.begin(UDP_LOCAL_PORT_LIDAR);
            wifiReady = true;
            Serial.printf("[WIFI] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[UDP] Lidar -> %s:%d\n", UDP_REMOTE_IP.toString().c_str(), UDP_REMOTE_PORT_LIDAR);
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

void flushLidarPacket() {
    if (currentLidarPoints == 0) return;

    sendUdpPacket(lidarUdp, UDP_REMOTE_PORT_LIDAR, currentLidarBuf, currentLidarPoints * LIDAR_POINT_SIZE);
    lidarPacketCount++;

    currentLidarBuf = (currentLidarBuf == lidarBufA) ? lidarBufB : lidarBufA;
    currentLidarPoints = 0;
    lastLidarFlushUs = micros();
}

void queueFullLidarPacket() {
    sendLidarBuf = currentLidarBuf;
    currentLidarBuf = (currentLidarBuf == lidarBufA) ? lidarBufB : lidarBufA;
    currentLidarPoints = 0;
    sendLidarNow = true;
    lastLidarFlushUs = micros();
}

void sendQueuedLidarPacket() {
    if (sendLidarNow && sendLidarBuf != nullptr) {
        sendUdpPacket(lidarUdp, UDP_REMOTE_PORT_LIDAR, sendLidarBuf, LIDAR_PAYLOAD_SIZE);
        lidarPacketCount++;
        sendLidarNow = false;
        sendLidarBuf = nullptr;
    }
}

// ===================== SERVO =====================
void commandServoPhase(uint8_t p) {
    if (p == 0) {
        Serial.println("[SERVO] -> left");
        sc.WritePos(SERVO_ID, POS_START, 0, SERVO_SPEED);
    } else {
        Serial.println("[SERVO] -> right");
        sc.WritePos(SERVO_ID, POS_END, 0, SERVO_SPEED);
    }
}

void setupServo() {
    Serial.println("[SERVO] init");

    Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    sc.pSerial = &Serial1;

    delay(1000);

    int ping = sc.Ping(SERVO_ID);
    Serial.printf("[SERVO] ping=%d\n", ping);

    commandServoPhase(servoPhase);
}

void updateServo() {
    if (millis() - lastServoFeedbackMs >= 20) {
        lastServoFeedbackMs = millis();

        if (sc.FeedBack(SERVO_ID) != -1) {
            int pos = sc.ReadPos(-1);
            int isMoving = sc.ReadMove(-1);

            currentServoPos = pos;

            if (isMoving == 0) {
                servoPhase = (servoPhase == 0) ? 1 : 0;
                commandServoPhase(servoPhase);
            }
        }
    }
}

// ===================== LIDAR =====================
void processLidarPoint(uint16_t dist_mm, uint16_t angle_q6, uint8_t newRotFlag, uint8_t quality) {
    if (dist_mm == 0) return;

    lidarPointCount++;

    const uint32_t now_us = micros();
    const uint16_t servo_raw = clampServoRaw(currentServoPos);

    const size_t offset = currentLidarPoints * LIDAR_POINT_SIZE;
    memcpy(&currentLidarBuf[offset + 0],  &now_us,    4);
    memcpy(&currentLidarBuf[offset + 4],  &servo_raw, 2);
    memcpy(&currentLidarBuf[offset + 6],  &angle_q6,  2);
    memcpy(&currentLidarBuf[offset + 8],  &dist_mm,   2);

    currentLidarBuf[offset + 10] = newRotFlag;
    currentLidarBuf[offset + 11] = quality;

    uint32_t reserved = 0;
    memcpy(&currentLidarBuf[offset + 12], &reserved, 4);

    currentLidarPoints++;

    if (currentLidarPoints >= POINTS_PER_PACKET) {
        queueFullLidarPacket();
    }
}

void setupLidar() {
    Serial.println("[LIDAR] init");

    pinMode(LIDAR_MOTOR_PIN, OUTPUT);
    digitalWrite(LIDAR_MOTOR_PIN, HIGH);
    delay(LIDAR_SPINUP_MS);

    Serial2.setRxBufferSize(4096);
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
        processLidarPoint(dist, angle_q6, newRotFlag, (uint8_t)quality);
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

    lastLidarFlushUs = micros();
}

void updateLidar() {
    int decode_count = 0;
    while (decode_count < 400) {
        lidarHandleCalls++;
        int n = lidar.handleData(false, false);
        if (n <= 0) break;
        lidarHandlePositive++;
        decode_count++;
    }

    sendQueuedLidarPacket();

    const uint32_t now_us = micros();
    if (currentLidarPoints > 0 && (uint32_t)(now_us - lastLidarFlushUs) >= LIDAR_FLUSH_INTERVAL_US) {
        flushLidarPacket();
        lidarFlushPartialCount++;
    }

    if (millis() - lastLidarPrintMs >= 1000) {
        lastLidarPrintMs = millis();
        Serial.printf("[LIDAR] pts=%lu pkts=%lu partial=%lu handle=%lu pos=%lu queued=%u servo_pos=%d phase=%u\n",
                      (unsigned long)lidarPointCount,
                      (unsigned long)lidarPacketCount,
                      (unsigned long)lidarFlushPartialCount,
                      (unsigned long)lidarHandleCalls,
                      (unsigned long)lidarHandlePositive,
                      (unsigned)currentLidarPoints,
                      currentServoPos,
                      servoPhase);
    }
}

// ===================== ARDUINO =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("[BOOT] Servo + Lidar + UDP");

    setupServo();
    setupLidar();
    wifiInit();
}

void loop() {
    wifiUpdate();
    updateServo();
    updateLidar();
}
