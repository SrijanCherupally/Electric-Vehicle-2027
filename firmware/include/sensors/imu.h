#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>

#include "ICM42688.h"

// ICM-42688-P on SPI0. Only yaw rate is used for driving; accel is kept for
// telemetry / future use. Units at this boundary: rad/s and m/s^2.
class IMU {
 public:
  IMU();

  bool begin();
  void update();  // one SPI read, call at CONTROL_HZ

  // Yaw rate about the configured axis, bias removed, CCW positive. rad/s.
  float yawRate() const { return yawRate_; }
  float rawYawRate() const { return rawYaw_; }
  float accelForward() const { return accX_; }
  float bias() const { return bias_; }
  bool isInitialized() const { return initialized_; }

  // Bias calibration: keep the car still. start, then call update() until done.
  void startBiasCalibration(uint32_t durationMs);
  bool biasCalibrationDone() const { return !calibrating_; }
  // Slow bias tracking while known to be stationary (dt in s).
  void trackBias(float dt);

 private:
  static uint8_t readReg(uint8_t reg);
  static void writeReg(uint8_t reg, uint8_t value);

  ICM42688 imu_;
  bool initialized_ = false;
  float rawYaw_ = 0.0f;
  float yawRate_ = 0.0f;
  float accX_ = 0.0f;
  float bias_ = 0.0f;

  bool calibrating_ = false;
  uint32_t calEndMs_ = 0;
  double calSum_ = 0.0;
  uint32_t calCount_ = 0;
};

extern IMU imu;

#endif  // IMU_H
