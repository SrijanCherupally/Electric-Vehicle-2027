#include "sensors/imu.h"

#include <math.h>

#include "config.h"
#include "hardware.h"
#include "system/params.h"

IMU imu;

namespace {
constexpr float G_TO_MS2 = 9.80665f;
constexpr float DPS_TO_RADS = 0.017453292519943295f;
constexpr float MAX_VALID_RATE = 30.0f;  // rad/s, reject SPI garbage

// ICM-42688 bank 0 registers (datasheet DS-000347, 14.1)
constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_BANK_SEL = 0x76;
constexpr uint8_t TDK_WHO_AM_I = 0x47;
}  // namespace

IMU::IMU() : imu_(SPI, PIN_IMU_CS) {}

bool IMU::begin() {
  SPI.setRX(PIN_IMU_MISO);
  SPI.setSCK(PIN_IMU_SCK);
  SPI.setTX(PIN_IMU_MOSI);
  SPI.begin();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  delay(1);

  writeReg(REG_BANK_SEL, 0);
  const uint8_t who = readReg(REG_WHO_AM_I);
  Serial.printf("IMU: WHO_AM_I = 0x%02X (TDK ICM-42688-P = 0x%02X)\n", who, TDK_WHO_AM_I);

  int status = imu_.begin();
  if (status == -3 && IMU_ACCEPT_UNKNOWN_WHO_AM_I && who != 0x00 && who != 0xFF) {
    // The library stops at its WHO_AM_I check, after the soft reset and before
    // powering the sensors. Finish that part here, then make sure the part
    // really behaves like an ICM-42688 before trusting it.
    Serial.printf("IMU: ID 0x%02X is not TDK's, trying it as a compatible part\n", who);
    writeReg(REG_BANK_SEL, 0);
    writeReg(REG_PWR_MGMT0, 0x0F);  // gyro + accel low-noise mode
    delay(50);                      // gyro start-up ~30 ms
    status = 1;
  }
  if (status < 0) {
    Serial.printf("IMU: init failed (%d)%s\n", status,
                  status == -3 ? " (WHO_AM_I rejected; see IMU_ACCEPT_UNKNOWN_WHO_AM_I)" : "");
    initialized_ = false;
    return false;
  }
  // A ground vehicle turns slowly: the smallest range gives the best resolution.
  imu_.setGyroFS(ICM42688::dps250);
  imu_.setAccelFS(ICM42688::gpm4);
  imu_.setGyroODR(ICM42688::odr1k);
  imu_.setAccelODR(ICM42688::odr1k);

  if (who != TDK_WHO_AM_I) {
    // Plausibility check for a non-TDK ID: at rest the accelerometer reads ~1 g.
    delay(20);
    float g = 0.0f;
    if (imu_.getAGT() >= 0) {
      const float ax = imu_.accX(), ay = imu_.accY(), az = imu_.accZ();
      g = sqrtf(ax * ax + ay * ay + az * az);
    }
    if (!(g > 0.7f && g < 1.3f)) {
      Serial.printf("IMU: ID 0x%02X reads |a| = %.2f g at rest, not ICM-42688 compatible\n", who, g);
      initialized_ = false;
      return false;
    }
    Serial.printf("IMU: ID 0x%02X accepted (|a| = %.2f g)\n", who, g);
  }
  initialized_ = true;
  return true;
}

// Raw register access, used only around the library's WHO_AM_I check.
// ICM-42688 SPI: mode 0 or 3, MSB first, bit 7 of the address = read.
uint8_t IMU::readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  const uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void IMU::writeReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
}

void IMU::update() {
  if (!initialized_) return;
  if (imu_.getAGT() < 0) return;

  float g = 0.0f;
  switch (IMU_YAW_AXIS) {
    case 0: g = imu_.gyrX(); break;
    case 1: g = imu_.gyrY(); break;
    default: g = imu_.gyrZ(); break;
  }
  const float raw = IMU_YAW_SIGN * g * DPS_TO_RADS * params.gyroScale;
  if (!isfinite(raw) || fabsf(raw) > MAX_VALID_RATE) return;

  rawYaw_ = raw;
  accX_ = imu_.accX() * G_TO_MS2;

  if (calibrating_) {
    calSum_ += raw;
    ++calCount_;
    if ((int32_t)(millis() - calEndMs_) >= 0) {
      if (calCount_ > 0) bias_ = (float)(calSum_ / calCount_);
      calibrating_ = false;
    }
  }
  yawRate_ = rawYaw_ - bias_;
}

void IMU::startBiasCalibration(uint32_t durationMs) {
  calSum_ = 0.0;
  calCount_ = 0;
  calEndMs_ = millis() + durationMs;
  calibrating_ = initialized_;
}

void IMU::trackBias(float dt) {
  if (!initialized_ || calibrating_) return;
  // Only track small residuals: a real rotation must not be learned as bias.
  if (fabsf(rawYaw_ - bias_) > 0.02f) return;
  const float alpha = dt / (GYRO_BIAS_TRACK_TAU_S + dt);
  bias_ += alpha * (rawYaw_ - bias_);
}
