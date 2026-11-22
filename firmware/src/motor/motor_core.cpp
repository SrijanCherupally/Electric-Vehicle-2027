#include "motor/motor_core.h"

#include "config.h"
#include "hardware.h"
#include "motor/current_sense.h"
#include "motor/drv8317.h"
#include "motor/encoder.h"

namespace motorcore {

namespace {

// ---------- Hardware (core 1 only) ----------
ThreePhasePwm g_pwm[MOTOR_COUNT] = {
    ThreePhasePwm(PIN_M1_INA, PIN_M1_INB, PIN_M1_INC),
    ThreePhasePwm(PIN_M2_INA, PIN_M2_INB, PIN_M2_INC),
};
QuadratureEncoder g_enc[MOTOR_COUNT] = {
    QuadratureEncoder(PIN_M1_ENC_B),  // B is the lower pin, see hardware.h
    QuadratureEncoder(PIN_M2_ENC_B),
};
FocMotor g_motor[MOTOR_COUNT] = {
    FocMotor(g_pwm[0], g_enc[0]),
    FocMotor(g_pwm[1], g_enc[1]),
};
CurrentSense g_sense;

// ---------- Shared state (core 0 <-> core 1) ----------
volatile bool s_ready = false;
volatile DriveMode s_requestedMode = DriveMode::Off;
volatile DriveMode s_activeMode = DriveMode::Off;
volatile float s_command[MOTOR_COUNT] = {0.0f, 0.0f};
volatile uint32_t s_commandSeq = 0;
volatile uint32_t s_alignRequestSeq = 0;
volatile AlignStatus s_alignStatus = AlignStatus::NotAligned;
volatile AlignResult s_alignResult[MOTOR_COUNT] = {AlignResult::NotRun, AlignResult::NotRun};
volatile int32_t s_alignCounts[MOTOR_COUNT] = {0, 0};
volatile int32_t s_position[MOTOR_COUNT] = {0, 0};
volatile float s_iq[MOTOR_COUNT] = {0.0f, 0.0f};
volatile float s_uq[MOTOR_COUNT] = {0.0f, 0.0f};
volatile float s_vbus = VBAT_NOMINAL_V;
volatile float s_loopRate = 0.0f;
volatile MotorFault s_fault = MotorFault::None;
volatile uint32_t s_clearFaultSeq = 0;

// ---------- core 1 bookkeeping ----------
uint32_t c1_alignHandledSeq = 0;
uint32_t c1_clearHandledSeq = 0;
uint32_t c1_lastCommandSeq = 0;
uint32_t c1_lastCommandMs = 0;
uint32_t c1_overcurrentSinceMs = 0;
uint32_t c1_divider = 0;
uint32_t c1_vbatDivider = 0;
uint32_t c1_rateCount = 0;
uint32_t c1_rateStartUs = 0;
uint8_t c1_nfaultCount = 0;

void latchFault(MotorFault f) {
  if (s_fault == MotorFault::None) s_fault = f;
  for (auto& m : g_motor) m.brake();
  Drv8317::setAwake(false);
  s_activeMode = DriveMode::Off;
}

void applyMode(DriveMode m) {
  if (m == s_activeMode) return;
  if (m == DriveMode::Off) {
    for (auto& mot : g_motor) mot.brake();
    Drv8317::setAwake(false);
  } else {
    if (!Drv8317::isAwake()) Drv8317::setAwake(true);
    c1_lastCommandMs = millis();  // fresh timeout window
  }
  s_activeMode = m;
}

void runAlignment() {
  s_alignStatus = AlignStatus::Aligning;
  Drv8317::setAwake(true);
  const float vbus = s_vbus;
  bool allOk = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const AlignResult r = g_motor[i].align(vbus);
    s_alignResult[i] = r;
    s_alignCounts[i] = g_motor[i].countsPerElectricalRev();
    allOk = allOk && (r == AlignResult::Ok);
  }
  s_alignStatus = allOk ? AlignStatus::Aligned : AlignStatus::Failed;
  // Leave the motors braked; core 0 decides what happens next.
  s_activeMode = DriveMode::Brake;
}

void updateLoopRate() {
  ++c1_rateCount;
  const uint32_t now = micros();
  const uint32_t elapsed = now - c1_rateStartUs;
  if (elapsed >= 500000u) {
    s_loopRate = (float)c1_rateCount * 1e6f / (float)elapsed;
    c1_rateCount = 0;
    c1_rateStartUs = now;
  }
}

}  // namespace

// ===========================================================================
// core 1
// ===========================================================================

void setup() {
  Drv8317::begin();
  g_sense.begin();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    g_enc[i].begin();
    g_motor[i].begin();
  }
  ThreePhasePwm::startAll();

  // Battery voltage before anything needs it.
  float v = 0.0f;
  for (int i = 0; i < 16; ++i) v += g_sense.readBatteryVolts();
  s_vbus = v / 16.0f;

  // Zero-current offsets: drivers awake, all phases at 50% duty.
  Drv8317::setAwake(true);
  delay(5);
  g_sense.calibrateOffsets(CURRENT_OFFSET_SAMPLES);
  Drv8317::setAwake(false);

  // MT6701 A/B outputs are not valid until ~50 ms after power-up.
  while (millis() < MT6701_STARTUP_MS) {
    tight_loop_contents();
  }

  c1_rateStartUs = micros();
  s_ready = true;
}

void loop() {
  // ---- One-shot requests from core 0 ----
  if (s_clearFaultSeq != c1_clearHandledSeq) {
    c1_clearHandledSeq = s_clearFaultSeq;
    Drv8317::clearFault();
    Drv8317::setAwake(false);
    s_activeMode = DriveMode::Off;
    s_requestedMode = DriveMode::Off;
    c1_nfaultCount = 0;
    c1_overcurrentSinceMs = 0;
    s_fault = MotorFault::None;
  }
  if (s_alignRequestSeq != c1_alignHandledSeq) {
    c1_alignHandledSeq = s_alignRequestSeq;
    if (s_fault == MotorFault::None) {
      runAlignment();
    } else {
      s_alignStatus = AlignStatus::Failed;
    }
  }

  // ---- Lock to the PWM period (center of the low-side window) ----
  ThreePhasePwm::waitForPeriodStart();
  if (++c1_divider < FOC_LOOP_DIVIDER) return;
  c1_divider = 0;

#if CURRENT_SENSE_ENABLED
  g_sense.sample();
#endif

  if (++c1_vbatDivider >= 64) {
    c1_vbatDivider = 0;
    s_vbus = s_vbus + VBAT_FILTER_ALPHA * (g_sense.readBatteryVolts() - s_vbus);
  }

  // ---- Mode + safety ----
  if (s_fault == MotorFault::None) {
    applyMode(s_requestedMode);
  }

  if (Drv8317::isAwake()) {
    c1_nfaultCount = Drv8317::faultActive() ? (uint8_t)(c1_nfaultCount + 1) : 0;
    if (c1_nfaultCount >= 3) latchFault(MotorFault::DriverFault);
  }

  const uint32_t nowMs = millis();
  if (s_commandSeq != c1_lastCommandSeq) {
    c1_lastCommandSeq = s_commandSeq;
    c1_lastCommandMs = nowMs;
  }
  bool drive = (s_activeMode == DriveMode::Run);
  if (drive && (nowMs - c1_lastCommandMs) > COMMAND_TIMEOUT_MS) {
    latchFault(MotorFault::CommandTimeout);
    drive = false;
  }

  // ---- FOC ----
  const float vbus = s_vbus;
  bool overcurrent = false;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    FocMotor& m = g_motor[i];
    m.setTarget(s_command[i]);
    m.update(vbus, g_sense.phaseA(i), g_sense.phaseB(i), drive);
    s_position[i] = m.position();
    s_iq[i] = m.iq();
    s_uq[i] = m.uq();
    if (fabsf(m.iq()) > OVERCURRENT_A) overcurrent = true;
  }

#if CURRENT_SENSE_ENABLED
  if (OVERCURRENT_TIME_MS > 0 && drive) {
    if (!overcurrent) {
      c1_overcurrentSinceMs = 0;
    } else if (c1_overcurrentSinceMs == 0) {
      c1_overcurrentSinceMs = nowMs;
    } else if (nowMs - c1_overcurrentSinceMs > OVERCURRENT_TIME_MS) {
      latchFault(MotorFault::Overcurrent);
    }
  }
#else
  (void)overcurrent;
#endif

  updateLoopRate();
}

// ===========================================================================
// core 0 API
// ===========================================================================

bool ready() { return s_ready; }

void setMode(DriveMode m) { s_requestedMode = m; }
DriveMode mode() { return s_activeMode; }

void setCommands(float m1, float m2) {
  s_command[0] = m1;
  s_command[1] = m2;
  s_commandSeq = s_commandSeq + 1;
}

void requestAlign() {
  s_alignStatus = AlignStatus::Aligning;
  s_alignRequestSeq = s_alignRequestSeq + 1;
}

AlignStatus alignStatus() { return s_alignStatus; }
AlignResult alignResult(uint8_t m) { return m < MOTOR_COUNT ? s_alignResult[m] : AlignResult::NotRun; }
int32_t alignCountsPerErev(uint8_t m) { return m < MOTOR_COUNT ? s_alignCounts[m] : 0; }

int32_t position(uint8_t m) { return m < MOTOR_COUNT ? s_position[m] : 0; }
float iq(uint8_t m) { return m < MOTOR_COUNT ? s_iq[m] : 0.0f; }
float command(uint8_t m) { return m < MOTOR_COUNT ? s_uq[m] : 0.0f; }
float busVoltage() { return s_vbus; }
float loopRateHz() { return s_loopRate; }

MotorFault fault() { return s_fault; }

const char* faultName(MotorFault f) {
  switch (f) {
    case MotorFault::None: return "none";
    case MotorFault::DriverFault: return "DRV8317 nFAULT";
    case MotorFault::Overcurrent: return "overcurrent";
    case MotorFault::CommandTimeout: return "command timeout";
  }
  return "?";
}

void clearFault() {
  s_requestedMode = DriveMode::Off;
  s_clearFaultSeq = s_clearFaultSeq + 1;
}

}  // namespace motorcore
