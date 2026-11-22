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
}  // namespace

IMU::IMU() : imu_(SPI, PIN_IMU_CS) {}

bool IMU::begin() {
  SPI.setRX(PIN_IMU_MISO);
  SPI.setSCK(PIN_IMU_SCK);
  SPI.setTX(PIN_IMU_MOSI);
  SPI.begin();

  const int status = imu_.begin();
  if (status < 0) {
    Serial.printf("IMU: init failed (%d)\n", status);
    initialized_ = false;
    return false;
  }
  // A ground vehicle turns slowly: the smallest range gives the best resolution.
  imu_.setGyroFS(ICM42688::dps250);
  imu_.setAccelFS(ICM42688::gpm4);
  imu_.setGyroODR(ICM42688::odr1k);
  imu_.setAccelODR(ICM42688::odr1k);
  initialized_ = true;
  return true;
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
