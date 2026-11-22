#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <Arduino.h>

// Phase current + battery voltage sampling on the RP2350 ADC.
//
// Owned by core 1: the ADC has one input mux, so a single core must do every
// conversion. Phases A and B are read per motor (C = -A - B). sample() must be
// called right after ThreePhasePwm::waitForPeriodStart(), while the low-side
// FETs are on and the DRV8317 sense amplifiers are valid.
class CurrentSense {
 public:
  static constexpr uint8_t MOTORS = 2;

  void begin();
  void sample();  // ~8 us: M1A, M2A, M1B, M2B
  void calibrateOffsets(uint16_t samples);  // drivers awake, zero voltage vector
  float phaseA(uint8_t motor) const { return amps(motor, 0); }
  float phaseB(uint8_t motor) const { return amps(motor, 1); }

  float readBatteryVolts();  // one conversion, ~2 us

 private:
  float amps(uint8_t motor, uint8_t phase) const;
  uint16_t raw_[MOTORS][2] = {{0, 0}, {0, 0}};
  float offset_[MOTORS][2] = {{2048.0f, 2048.0f}, {2048.0f, 2048.0f}};
};

#endif  // CURRENT_SENSE_H
