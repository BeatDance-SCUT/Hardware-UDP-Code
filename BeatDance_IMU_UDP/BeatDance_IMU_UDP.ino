/*
 * BeatDance: ESP32-S3 Super Mini + GY-9250 board (detected WHO_AM_I=0x70)
 * Read MPU6500/MPU9250 6-axis data over I2C and send UDP JSON via Wi-Fi.
 * Arduino ESP32 core: WiFi.h / WiFiUdp.h / Wire.h (no 3rd-party libs)
 *
 * Wiring: IMU VCC->3V3, GND->GND, SDA->GPIO8, SCL->GPIO9
 *          AD0->GND recommended, NCS->3V3 if not pulled up on board.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======= CHANGE THESE THREE ITEMS =======
const char *WIFI_SSID = "BeatDance-SCUT";
const char *WIFI_PASSWORD = "test1234";
const IPAddress PC_IP(10, 58, 46, 100);  // computer's LAN IPv4
// ========================================

constexpr uint16_t PC_PORT = 5005;
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;
constexpr uint8_t IMU_ADDR = 0x68;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;  // 50 Hz
const char *DEVICE_ID = "esp32s3-imu-01";

WiFiUDP udp;
uint32_t sequenceNumber = 0;
uint32_t lastSampleMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t lastReportMs = 0;
uint32_t sentCount = 0;
uint32_t readErrors = 0;
uint8_t imuWhoAmI = 0;
bool wasConnected = false;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *out, uint8_t count) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t received = Wire.requestFrom(IMU_ADDR, count, true);
  if (received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) out[i] = Wire.read();
  return true;
}

int16_t readBE16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

bool initializeIMU() {
  if (!readRegs(0x75, &imuWhoAmI, 1)) return false;
  Serial.printf("IMU WHO_AM_I = 0x%02X\n", imuWhoAmI);
  if (imuWhoAmI != 0x70 && imuWhoAmI != 0x71) {
    Serial.println("Unsupported IMU ID (expected MPU6500=0x70 or MPU9250=0x71)");
    return false;
  }

  // Reset and wake up.
  if (!writeReg(0x6B, 0x80)) return false;
  delay(100);
  if (!writeReg(0x6B, 0x01)) return false;  // gyro clock
  if (!writeReg(0x6C, 0x00)) return false;  // all axes active
  delay(10);

  // Set filtered output rate to ~50 Hz: 1 kHz / (1 + 19).
  if (!writeReg(0x1A, 0x04)) return false;  // gyro DLPF ~20 Hz
  if (!writeReg(0x1D, 0x04)) return false;  // accel DLPF ~20 Hz
  if (!writeReg(0x19, 19)) return false;

  // +/-4 g => 8192 LSB/g; +/-500 deg/s => 65.5 LSB/(deg/s).
  if (!writeReg(0x1C, 0x08)) return false;
  if (!writeReg(0x1B, 0x08)) return false;
  return true;
}

void maintainWiFi(uint32_t nowMs) {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    Serial.print("Wi-Fi connected, ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Sending UDP to ");
    Serial.print(PC_IP);
    Serial.printf(":%u\n", PC_PORT);
  } else if (!connected && wasConnected) {
    Serial.println("Wi-Fi disconnected; reconnecting...");
  }
  wasConnected = connected;

  if (!connected && (uint32_t)(nowMs - lastWifiRetryMs) >= 5000) {
    lastWifiRetryMs = nowMs;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nBeatDance IMU -> UDP JSON (6 axes / 50 Hz)");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  if (!initializeIMU()) {
    Serial.println("IMU initialization failed. Check 3V3/GND/SDA/SCL.");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // reduce Wi-Fi latency
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiRetryMs = millis();
  Serial.println("Connecting to Wi-Fi...");
}

void loop() {
  const uint32_t now = millis();
  maintainWiFi(now);
  if ((uint32_t)(now - lastSampleMs) < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;

  uint8_t raw[14];
  if (!readRegs(0x3B, raw, sizeof(raw))) {
    ++readErrors;
    return;
  }

  // Raw register values are signed, big-endian integers.
  const float ax = readBE16(raw + 0) / 8192.0f;
  const float ay = readBE16(raw + 2) / 8192.0f;
  const float az = readBE16(raw + 4) / 8192.0f;
  const float tempC = readBE16(raw + 6) / 333.87f + 21.0f;
  const float gx = readBE16(raw + 8) / 65.5f;
  const float gy = readBE16(raw + 10) / 65.5f;
  const float gz = readBE16(raw + 12) / 65.5f;

  // Increment for each acquired sample, even when Wi-Fi is disconnected.
  const uint32_t seq = sequenceNumber++;
  if (WiFi.status() == WL_CONNECTED) {
    char payload[256];
    const int len = snprintf(
      payload, sizeof(payload),
      "{\"v\":1,\"device\":\"%s\",\"seq\":%lu,\"t_ms\":%lu,"
      "\"imu_id\":\"0x%02X\",\"acc_g\":[%.4f,%.4f,%.4f],"
      "\"gyro_dps\":[%.3f,%.3f,%.3f],\"temp_c\":%.2f}",
      DEVICE_ID,
      static_cast<unsigned long>(seq),
      static_cast<unsigned long>(now),
      imuWhoAmI, ax, ay, az, gx, gy, gz, tempC
    );
    if (len > 0 && len < static_cast<int>(sizeof(payload)) &&
        udp.beginPacket(PC_IP, PC_PORT)) {
      udp.write(reinterpret_cast<const uint8_t *>(payload), static_cast<size_t>(len));
      if (udp.endPacket()) ++sentCount;
    }
  }

  if ((uint32_t)(now - lastReportMs) >= 5000) {
    lastReportMs = now;
    Serial.printf("IMU samples: %lu | UDP sent: %lu | read errors: %lu | Wi-Fi: %s\n",
                  static_cast<unsigned long>(sequenceNumber),
                  static_cast<unsigned long>(sentCount),
                  static_cast<unsigned long>(readErrors),
                  WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  }
}
