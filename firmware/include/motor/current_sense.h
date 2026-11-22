#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <Arduino.h>

#include "motor/drv8317.h"

// Phase current + battery voltage sampling on the RP2350 ADC.
//
// Owned by core 1: the ADC has one input mux, so a single core must do every
// conversion. sample() must be called right after
// ThreePhasePwm::waitForPeriodStart(), at the PWM valley, where every low-side
// FET is on and the DRV8317 low-side sense amplifiers are valid.
//
// A phase's low-side window is only (1 - duty) * T wide, centred on the valley,
// and four conversions take ~8 us. So per motor:
//   - only the two phases with the lowest duty (widest windows) are converted,
//     and the third is derived (ia + ib + ic = 0);
//   - the narrower of the two is converted first;
//   - each conversion is checked against its phase's window using the PWM
//     counter. If either misses, that motor's sample is dropped and the last
//     valid currents are held (valid() returns false for this period).
class CurrentSense {
 public:
  static constexpr uint8_t MOTORS = 2;

  void begin();
  void sample(const ThreePhasePwm* pwm);    // pwm[MOTORS], duties in force this period
  void calibrateOffsets(uint16_t samples);  // drivers awake, all phases at 50% duty
  float phaseA(uint8_t motor) const { return amps_[motor][0]; }
  float phaseB(uint8_t motor) const { return amps_[motor][1]; }
  bool valid(uint8_t motor) const { return valid_[motor]; }  // last sample() was usable

  float readBatteryVolts();  // one conversion, ~2 us

 private:
  float toAmps(uint8_t motor, uint8_t phase, uint16_t raw) const;
  float amps_[MOTORS][3] = {{0, 0, 0}, {0, 0, 0}};
  bool valid_[MOTORS] = {false, false};
  float offset_[MOTORS][3] = {{2048.0f, 2048.0f, 2048.0f}, {2048.0f, 2048.0f, 2048.0f}};
};

#endif  // CURRENT_SENSE_H
