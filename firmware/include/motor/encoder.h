#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <hardware/pio.h>

// Quadrature (A/B) decoder running on a PIO state machine, so no edges are
// missed at any motor speed and the CPU never takes encoder interrupts.
//
// The decoder reads two CONSECUTIVE pins starting at pinBase. The PIO program
// is the classic pico-examples jump-table decoder; it uses computed jumps, so
// it must be loaded at instruction offset 0 of whichever PIO block is free.
class QuadratureEncoder {
 public:
  explicit QuadratureEncoder(uint pinBase) : pinBase_(pinBase) {}

  bool begin();
  int32_t read();  // raw x4 count, direction as wired
  bool ok() const { return sm_ >= 0; }

 private:
  uint pinBase_;
  PIO pio_ = nullptr;
  int sm_ = -1;
};

#endif  // ENCODER_H
