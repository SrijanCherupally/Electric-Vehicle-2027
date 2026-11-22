#include "motor/foc.h"

#include <math.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------
namespace foc {

namespace {
constexpr float TWO_PI_F = 6.28318530718f;
constexpr float SQRT3_F = 1.73205080757f;
constexpr float INV_SQRT3_F = 0.57735026919f;
}  // namespace

float wrapAngle(float a) {
  a = fmodf(a, TWO_PI_F);
  return (a < 0.0f) ? a + TWO_PI_F : a;
}

AlphaBeta clarke(float ia, float ib) { return {ia, (ia + 2.0f * ib) * INV_SQRT3_F}; }

DQ park(AlphaBeta x, float s, float c) {
  return {x.alpha * c + x.beta * s, -x.alpha * s + x.beta * c};
}

AlphaBeta inversePark(DQ x, float s, float c) {
  return {x.d * c - x.q * s, x.d * s + x.q * c};
}

void svpwm(AlphaBeta v, float vbus, float& da, float& db, float& dc) {
  const float ua = v.alpha;
  const float ub = -0.5f * v.alpha + 0.5f * SQRT3_F * v.beta;
  const float uc = -0.5f * v.alpha - 0.5f * SQRT3_F * v.beta;
  // Shift the three phase voltages so they sit centered in the bus:
  // equivalent to space-vector modulation, 15% more range than sine PWM.
  const float hi = fmaxf(ua, fmaxf(ub, uc));
  const float lo = fminf(ua, fminf(ub, uc));
  const float mid = 0.5f * (hi + lo);
  const float inv = (vbus > 0.1f) ? 1.0f / vbus : 0.0f;
  da = 0.5f + (ua - mid) * inv;
  db = 0.5f + (ub - mid) * inv;
  dc = 0.5f + (uc - mid) * inv;
}

}  // namespace foc

// ---------------------------------------------------------------------------
// FocMotor
// ---------------------------------------------------------------------------

namespace {
constexpr float TWO_PI_F = 6.28318530718f;
constexpr float INV_SQRT3_F = 0.57735026919f;
constexpr float CURRENT_FILTER_ALPHA = 0.05f;
constexpr int ALIGN_STEPS = 200;

inline float clampf(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
}  // namespace

const char* alignResultName(AlignResult r) {
  switch (r) {
    case AlignResult::NotRun: return "not run";
    case AlignResult::Ok: return "ok";
    case AlignResult::EncoderMissing: return "encoder PIO failed";
    case AlignResult::NoMotion: return "no encoder motion (check encoder cable / motor phases)";
    case AlignResult::PolePairMismatch: return "counts per e-rev wrong (check MOTOR_POLE_PAIRS / ENCODER_CPR)";
  }
  return "?";
}

void FocMotor::begin() {
  pwm_.begin();
  aligned_ = false;
  countsPerErev_ = ENCODER_CPR / MOTOR_POLE_PAIRS;
}

float FocMotor::electricalAngle(int32_t raw) const {
  // Reduce in integer counts first so the angle stays exact over many turns.
  int32_t m = (int32_t)dir_ * (raw - zeroOffset_);
  m %= ENCODER_CPR;
  if (m < 0) m += ENCODER_CPR;
  const float mech = (float)m * (TWO_PI_F / (float)ENCODER_CPR);
  return foc::wrapAngle(mech * (float)MOTOR_POLE_PAIRS);
}

void FocMotor::applyVoltage(float ud, float uq, float angle, float vbus) {
  float s, c;
  s = sinf(angle);
  c = cosf(angle);
  float da, db, dc;
  foc::svpwm(foc::inversePark({ud, uq}, s, c), vbus, da, db, dc);
  pwm_.setDuties(da, db, dc);
}

void FocMotor::holdVector(float volts, float angle, float vbus) {
  applyVoltage(volts, 0.0f, angle, vbus);  // all voltage on d at "angle"
}

void FocMotor::brake() {
  pwm_.setNeutral();
  uq_ = 0.0f;
  iqInt_ = 0.0f;
  idInt_ = 0.0f;
}

AlignResult FocMotor::align(float vbus) {
  aligned_ = false;
  if (!enc_.ok()) return AlignResult::EncoderMissing;

  const uint32_t stepMs = (FOC_ALIGN_SWEEP_MS / ALIGN_STEPS) > 0 ? FOC_ALIGN_SWEEP_MS / ALIGN_STEPS : 1;

  // 1. Pull the rotor to electrical angle 0.
  holdVector(FOC_ALIGN_VOLTAGE_V, 0.0f, vbus);
  delay(FOC_ALIGN_SETTLE_MS);
  const int32_t start = enc_.read();

  // 2. Sweep one electrical revolution forward, then back.
  for (int i = 1; i <= ALIGN_STEPS; ++i) {
    holdVector(FOC_ALIGN_VOLTAGE_V, TWO_PI_F * i / ALIGN_STEPS, vbus);
    delay(stepMs);
  }
  const int32_t mid = enc_.read();
  for (int i = ALIGN_STEPS - 1; i >= 0; --i) {
    holdVector(FOC_ALIGN_VOLTAGE_V, TWO_PI_F * i / ALIGN_STEPS, vbus);
    delay(stepMs);
  }

  // 3. Settle at angle 0 and record it as the electrical zero.
  delay(FOC_ALIGN_SETTLE_MS);
  const int32_t zero = enc_.read();
  brake();

  const int32_t delta = mid - start;
  const float expected = (float)ENCODER_CPR / (float)MOTOR_POLE_PAIRS;
  const float moved = fabsf((float)delta);
  if (moved < 0.1f * expected) return AlignResult::NoMotion;
  if (moved < FOC_ALIGN_MIN_FRACTION * expected || moved > expected / FOC_ALIGN_MIN_FRACTION) {
    return AlignResult::PolePairMismatch;
  }

  dir_ = (delta > 0) ? 1 : -1;
  zeroOffset_ = zero;
  countsPerErev_ = delta;
  aligned_ = true;
  return AlignResult::Ok;
}

void FocMotor::update(float vbus, float ia, float ib, bool drive) {
  const int32_t raw = enc_.read();
  position_ = aligned_ ? (int32_t)dir_ * (raw - zeroOffset_) : raw;

  if (!aligned_) {
    brake();
    return;
  }

  const float theta = electricalAngle(raw);
  float s, c;
  s = sinf(theta);
  c = cosf(theta);

#if CURRENT_SENSE_ENABLED
  const foc::DQ i = foc::park(foc::clarke(ia, ib), s, c);
  iqFilt_ += CURRENT_FILTER_ALPHA * (i.q - iqFilt_);
  idFilt_ += CURRENT_FILTER_ALPHA * (i.d - idFilt_);
#else
  (void)ia;
  (void)ib;
#endif

  if (!drive) {
    brake();
    return;
  }

  const float vLimit = vbus * INV_SQRT3_F;  // max undistorted phase amplitude
  float ud = 0.0f;
  float uq = 0.0f;

#if FOC_CURRENT_MODE && CURRENT_SENSE_ENABLED
  // dq current loop: Id -> 0, Iq -> target (amps).
  const float dt = (float)FOC_LOOP_DIVIDER / ThreePhasePwm::frequencyHz();
  const float iqRef = clampf(target_, FOC_CURRENT_LIMIT_A);
  const float eq = iqRef - i.q;
  const float ed = 0.0f - i.d;
  iqInt_ = clampf(iqInt_ + FOC_CURRENT_KI * eq * dt, vLimit);
  idInt_ = clampf(idInt_ + FOC_CURRENT_KI * ed * dt, vLimit);
  uq = FOC_CURRENT_KP * eq + iqInt_ + FOC_PHASE_RESISTANCE_OHM * iqRef;
  ud = FOC_CURRENT_KP * ed + idInt_;
  // Limit the vector magnitude, keeping its direction.
  const float mag = sqrtf(ud * ud + uq * uq);
  if (mag > vLimit) {
    ud *= vLimit / mag;
    uq *= vLimit / mag;
  }
#else
  // Voltage mode: Uq is the command. Back-EMF is handled by the wheel loop
  // feedforward on core 0.
  uq = clampf(target_, vLimit);
#endif

  uq_ = uq;
  applyVoltage(ud, uq, theta, vbus);
}
