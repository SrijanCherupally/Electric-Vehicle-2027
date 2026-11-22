#ifndef DRV8317_H
#define DRV8317_H

#include <Arduino.h>

// DRV8317H control pins. Both drivers share nSLEEP and nFAULT on this board,
// so these are static helpers rather than per-driver objects.
class Drv8317 {
 public:
  static void begin();          // nSLEEP low (asleep), nFAULT input
  static void setAwake(bool awake);
  static bool isAwake();
  static bool faultActive();    // nFAULT pulled low by either driver
  static void clearFault();     // nSLEEP reset pulse, leaves the driver awake

 private:
  static bool awake_;
};

// One motor's three half-bridge PWM outputs (INHA/B/C).
//
// All slices run center-aligned (phase-correct) at the same TOP and are started
// together, so every phase of both motors is in phase. Outputs are inverted so
// that the PWM counter wrap (count = 0) is the middle of the low-side-on window:
// that is where the DRV8317 low-side current sense amplifiers are valid.
class ThreePhasePwm {
 public:
  ThreePhasePwm(uint pinA, uint pinB, uint pinC);

  void begin();                                // configure pins and slices (not started)
  void setDuties(float a, float b, float c);   // high-side duty 0..1 per phase
  void setNeutral() { setDuties(0.5f, 0.5f, 0.5f); }  // zero voltage vector = brake

  // Compare level of a phase as last written (0..TOP+1). Levels latch at the
  // PWM wrap, so right after waitForPeriodStart() this is the level in force.
  // The low side of that phase is on while counter() < level.
  uint16_t level(uint8_t phase) const { return level_[phase]; }

  static void startAll();             // start every configured slice in phase
  static void waitForPeriodStart();   // block until the next PWM wrap
  static uint32_t counter();          // shared counter, counts up from 0 after the wrap
  static uint32_t top() { return top_; }
  static float frequencyHz();

 private:
  uint pins_[3];
  uint slice_[3];
  uint chan_[3];
  uint16_t level_[3] = {0, 0, 0};

  static uint32_t sliceMask_;
  static uint32_t top_;
  static int syncSlice_;
};

#endif  // DRV8317_H
