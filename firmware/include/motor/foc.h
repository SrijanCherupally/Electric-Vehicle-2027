#ifndef FOC_H
#define FOC_H

#include <Arduino.h>

#include "motor/drv8317.h"
#include "motor/encoder.h"

// ---------------------------------------------------------------------------
// FOC math (pure functions, no hardware)
// ---------------------------------------------------------------------------
namespace foc {

struct AlphaBeta {
  float alpha;
  float beta;
};

struct DQ {
  float d;
  float q;
};

float wrapAngle(float a);  // -> [0, 2pi)

// Clarke transform with ia + ib + ic = 0.
AlphaBeta clarke(float ia, float ib);
DQ park(AlphaBeta x, float sinTheta, float cosTheta);
AlphaBeta inversePark(DQ x, float sinTheta, float cosTheta);

// Space-vector PWM via midpoint clamping: returns duties 0..1 for a, b, c.
void svpwm(AlphaBeta v, float vbus, float& da, float& db, float& dc);

}  // namespace foc

// ---------------------------------------------------------------------------
// One BLDC motor under field-oriented control (runs on core 1)
// ---------------------------------------------------------------------------

enum class AlignResult : uint8_t {
  NotRun = 0,
  Ok,
  EncoderMissing,   // PIO decoder could not start
  NoMotion,         // encoder did not move during the sweep
  PolePairMismatch  // moved, but not CPR / pole pairs counts per e-rev
};

const char* alignResultName(AlignResult r);

class FocMotor {
 public:
  FocMotor(ThreePhasePwm& pwm, QuadratureEncoder& encoder) : pwm_(pwm), enc_(encoder) {}

  void begin();

  // Blocking (~2 s): finds encoder direction and the electrical zero.
  // The wheel turns up to one electrical revolution; lift it or let it roll.
  AlignResult align(float vbus);
  bool aligned() const { return aligned_; }

  // Command: volts on q (voltage mode) or amps on q (current mode).
  void setTarget(float target) { target_ = target; }

  // One commutation step. Reads the encoder, so call every FOC cycle even
  // when braking to keep position() fresh.
  void update(float vbus, float ia, float ib, bool drive);
  void brake();  // zero voltage vector: phases shorted through the low sides

  // Motor-positive position: positive target makes this count up.
  int32_t position() const { return position_; }
  float iq() const { return iqFilt_; }
  float id() const { return idFilt_; }
  float uq() const { return uq_; }
  int8_t direction() const { return dir_; }
  int32_t countsPerElectricalRev() const { return countsPerErev_; }

 private:
  float electricalAngle(int32_t raw) const;
  void applyVoltage(float ud, float uq, float angle, float vbus);
  void holdVector(float volts, float angle, float vbus);

  ThreePhasePwm& pwm_;
  QuadratureEncoder& enc_;

  bool aligned_ = false;
  int8_t dir_ = 1;
  int32_t zeroOffset_ = 0;
  int32_t countsPerErev_ = 0;
  int32_t position_ = 0;

  float target_ = 0.0f;
  float uq_ = 0.0f;
  float iqFilt_ = 0.0f;
  float idFilt_ = 0.0f;

  // Current-mode PI state
  float iqInt_ = 0.0f;
  float idInt_ = 0.0f;
};

#endif  // FOC_H
