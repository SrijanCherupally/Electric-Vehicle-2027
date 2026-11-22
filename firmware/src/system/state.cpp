#include "system/state.h"

#include <math.h>
#include <stdio.h>

#include "config.h"
#include "control/controller.h"
#include "control/localizer.h"
#include "motor/drivetrain.h"
#include "motor/motor_core.h"
#include "sensors/battery.h"
#include "sensors/imu.h"
#include "system/params.h"
#include "system/telemetry.h"
#include "system/ui.h"

namespace state {

namespace {

State g_state = State::Boot;
RunPlan g_plan;
char g_fault[96] = "";
uint32_t g_enterMs = 0;

// Run bookkeeping
uint32_t g_runStartUs = 0;
float g_runT = 0.0f;
bool g_holding = false;
float g_holdStartT = 0.0f;
int8_t g_lastDir = 1;

// Motor test
float g_testVolts = 0.0f;
float g_testSeconds = 0.0f;
bool g_testKv = false;
double g_testSumL = 0.0, g_testSumR = 0.0;
uint32_t g_testSamples = 0;

void enter(State s) {
  if (s == g_state) return;
  g_state = s;
  g_enterMs = millis();
  // Drop button edges from the previous state, so a press made while booting
  // or aligning can never start a run the moment READY is reached.
  startButton.pressed();
  startButton.longPressed();
  Serial.printf("STATE: %s\n", name(s));
}

float secondsInState() { return (millis() - g_enterMs) * 0.001f; }

void enterFault(const char* reason) {
  snprintf(g_fault, sizeof(g_fault), "%s", reason);
  motorcore::setMode(DriveMode::Off);
  telemetry::logStop();
  Serial.printf("FAULT: %s\n", g_fault);
  enter(State::Fault);
}

void enterReady() {
  motorcore::setMode(DriveMode::Brake);
  localizer::reset();
  controller::reset();
  enter(State::Ready);
}

void startRun() {
  localizer::reset();
  controller::reset();
  telemetry::logStart();
  g_runStartUs = micros();
  g_runT = 0.0f;
  g_holding = false;
  g_lastDir = 1;
  drivetrain.zeroCommands();
  motorcore::setMode(DriveMode::Run);
  enter(State::Running);
}

void finishRun(const char* why) {
  motorcore::setMode(DriveMode::Brake);
  telemetry::logStop();
  const Pose& p = localizer::pose();
  const float dx = p.x - g_plan.xFinal;
  Serial.printf("RUN END (%s): t=%.3f s (plan %.3f, target %.2f)\n", why, g_runT,
                g_plan.plannedTime, params.targetTime);
  Serial.printf("  x=%.4f m (target %.4f, err %+.1f cm), y=%+.1f cm, heading=%+.2f deg\n", p.x,
                g_plan.xFinal, dx * 100.0f, p.y * 100.0f, p.heading * RAD_TO_DEG);
  Serial.printf("  est. vehicle distance %.1f cm, %u log samples ('dump' to export)\n",
                sqrtf(dx * dx + p.y * p.y) * 100.0f, (unsigned)telemetry::logCount());
  enter(State::Finished);
}

void updateRunning(float dt) {
  g_runT = (micros() - g_runStartUs) * 1e-6f;

  // Abort: a fresh press after the lockout (the start press itself is ignored).
  if (startButton.pressed() && g_runT > ABORT_LOCKOUT_S) {
    finishRun("aborted by button");
    return;
  }

  const MotionProfile& prof = g_plan.profile;
  if (!g_holding && g_runT < prof.duration()) {
    const ControllerDebug& d = controller::update(prof, g_runT, dt);
    if (d.ref.dir != 0) g_lastDir = d.ref.dir;
    if (fabsf(d.trackingError) > MAX_TRACKING_ERROR_M) {
      char msg[96];
      snprintf(msg, sizeof(msg), "tracking error %.2f m at t=%.2f s (stalled or slipping?)",
               d.trackingError, g_runT);
      enterFault(msg);
      return;
    }
  } else {
    if (!g_holding) {
      g_holding = true;
      g_holdStartT = g_runT;
    }
    controller::hold(prof.endPosition(), g_lastDir, dt);
    if (g_runT - g_holdStartT >= FINISH_SETTLE_S) {
      finishRun("complete");
      return;
    }
  }
  telemetry::logSample(g_runT);

  if (g_runT > MAX_RUN_TIME_S) finishRun("max run time");
}

void updateTest(float dt) {
  (void)dt;
  const float t = secondsInState();
  drivetrain.driveOpenLoop(g_testVolts, g_testVolts);
  if (t > 0.5f * g_testSeconds) {  // measure after it spins up
    g_testSumL += drivetrain.left().velocity();
    g_testSumR += drivetrain.right().velocity();
    ++g_testSamples;
  }
  if (t >= g_testSeconds) {
    motorcore::setMode(DriveMode::Brake);
    const float vl = g_testSamples ? (float)(g_testSumL / g_testSamples) : 0.0f;
    const float vr = g_testSamples ? (float)(g_testSumR / g_testSamples) : 0.0f;
    Serial.printf("TEST: %.2f V -> left %.3f m/s, right %.3f m/s (iq %.3f / %.3f A)\n",
                  g_testVolts, vl, vr, motorcore::iq(LEFT_MOTOR_INDEX), motorcore::iq(RIGHT_MOTOR_INDEX));
    if (vl * g_testVolts < 0.0f || vr * g_testVolts < 0.0f) {
      Serial.println("  WARNING: a wheel spun backwards; flip LEFT/RIGHT_WHEEL_DIR in config.h");
    }
    if (g_testKv) {
      const float vAvg = 0.5f * (fabsf(vl) + fabsf(vr));
      if (vAvg > 0.02f) {
        const float kff = fabsf(g_testVolts) / vAvg;
        params.kffVel = kff;
        Serial.printf("  kff_vel set to %.3f V/(m/s) ('save' to keep)\n", kff);
        replan();
      } else {
        Serial.println("  wheels did not spin; kff_vel unchanged");
      }
    }
    enterReady();
  }
}

}  // namespace

// ===========================================================================

void init() {
  g_state = State::Boot;
  g_enterMs = millis();
  replan();
}

void update(float dt) {
  // Motor core faults preempt everything.
  const MotorFault mf = motorcore::fault();
  if (mf != MotorFault::None && g_state != State::Fault) {
    char msg[64];
    snprintf(msg, sizeof(msg), "motor core: %s", motorcore::faultName(mf));
    enterFault(msg);
  }

  switch (g_state) {
    case State::Boot:
      statusLed.set(LedPattern::FastBlink);
      if (motorcore::ready()) {
        Serial.println("ALIGN: aligning both motors (wheels will twitch)...");
        motorcore::requestAlign();
        enter(State::Aligning);
      }
      break;

    case State::Aligning: {
      statusLed.set(LedPattern::FastBlink);
      const AlignStatus a = motorcore::alignStatus();
      if (a == AlignStatus::Aligned) {
        motorcore::setMode(DriveMode::Brake);
        imu.startBiasCalibration(GYRO_CAL_MS);
        Serial.println("ALIGN: ok. Calibrating gyro, keep the car still...");
        enter(State::Calibrating);
      } else if (a == AlignStatus::Failed) {
        char msg[96];
        snprintf(msg, sizeof(msg), "align failed: M1 %s / M2 %s",
                 alignResultName(motorcore::alignResult(0)), alignResultName(motorcore::alignResult(1)));
        enterFault(msg);
      }
      break;
    }

    case State::Calibrating:
      statusLed.set(LedPattern::On);
      if (imu.biasCalibrationDone()) {
        Serial.printf("GYRO: bias %.5f rad/s%s\n", imu.bias(),
                      imu.isInitialized() ? "" : " (IMU missing: wheel-only heading)");
        replan();
        printRunPlan(g_plan, Serial);
        enterReady();
      }
      break;

    case State::Ready:
      statusLed.set(g_plan.valid ? LedPattern::SlowBlink : LedPattern::Strobe);
      if (fabsf(drivetrain.speed()) < 0.005f) imu.trackBias(dt);
      if (startButton.pressed()) {
        if (g_plan.valid) {
          startRun();
        } else {
          Serial.printf("START refused: plan invalid (%s)\n", g_plan.error);
        }
      }
      break;

    case State::Running:
      statusLed.set(LedPattern::On);
      updateRunning(dt);
      break;

    case State::Finished:
      statusLed.set(LedPattern::DoubleBlink);
      if (startButton.pressed()) enterReady();
      break;

    case State::Test:
      statusLed.set(LedPattern::FastBlink);
      updateTest(dt);
      break;

    case State::Fault:
      statusLed.set(LedPattern::Strobe);
      motorcore::setMode(DriveMode::Off);
      startButton.pressed();  // swallow short presses
      if (startButton.longPressed()) clearFault();
      break;
  }

  // Ignore presses in states that do not use the button.
  if (g_state == State::Boot || g_state == State::Aligning || g_state == State::Calibrating ||
      g_state == State::Test) {
    startButton.pressed();
  }

  static uint32_t lastBatteryWarnMs = 0;
  if (battery::low() && g_state == State::Ready && millis() - lastBatteryWarnMs > 10000) {
    lastBatteryWarnMs = millis();
    Serial.printf("WARNING: battery low (%.2f V)\n", battery::voltage());
  }
}

State current() { return g_state; }

const char* name(State s) {
  switch (s) {
    case State::Boot: return "BOOT";
    case State::Aligning: return "ALIGNING";
    case State::Calibrating: return "CALIBRATING";
    case State::Ready: return "READY";
    case State::Running: return "RUNNING";
    case State::Finished: return "FINISHED";
    case State::Test: return "TEST";
    case State::Fault: return "FAULT";
  }
  return "?";
}

const char* faultReason() { return g_fault; }

const RunPlan& plan() { return g_plan; }

void replan() {
  if (g_state == State::Running) return;  // never swap the profile mid-run
  buildRunPlan(params, g_plan);
}

bool requestStart() {
  if (g_state != State::Ready || !g_plan.valid) return false;
  startRun();
  return true;
}

void requestAbort() {
  if (g_state == State::Running) {
    finishRun("aborted by command");
  } else if (g_state == State::Test) {
    enterReady();
  }
}

bool requestRealign() {
  if (g_state == State::Running || g_state == State::Test) return false;
  motorcore::setMode(DriveMode::Off);
  enter(State::Boot);
  return true;
}

bool requestGyroCal() {
  if (g_state != State::Ready) return false;
  imu.startBiasCalibration(GYRO_CAL_MS);
  enter(State::Calibrating);
  return true;
}

bool requestMotorTest(float volts, float seconds, bool kvCalibrate) {
  if (g_state != State::Ready) return false;
  g_testVolts = volts;
  g_testSeconds = seconds;
  g_testKv = kvCalibrate;
  g_testSumL = g_testSumR = 0.0;
  g_testSamples = 0;
  localizer::reset();
  drivetrain.zeroCommands();
  motorcore::setMode(DriveMode::Run);
  enter(State::Test);
  return true;
}

void clearFault() {
  if (g_state != State::Fault) return;
  Serial.println("FAULT cleared, re-aligning");
  g_fault[0] = '\0';
  motorcore::clearFault();
  enter(State::Boot);
}

void printStatus(Print& out) {
  const Pose& p = localizer::pose();
  out.printf("state %s%s%s\n", name(g_state), g_state == State::Fault ? ": " : "",
             g_state == State::Fault ? g_fault : "");
  out.printf("battery %.2f V (%.3f V/cell)%s\n", battery::voltage(), battery::cellVoltage(),
             battery::low() ? " LOW" : "");
  out.printf("foc loop %.0f Hz, align %s / %s, motor fault: %s\n", motorcore::loopRateHz(),
             alignResultName(motorcore::alignResult(0)), alignResultName(motorcore::alignResult(1)),
             motorcore::faultName(motorcore::fault()));
  out.printf("pose x=%.4f y=%.4f heading=%.2f deg, speed %.3f m/s\n", p.x, p.y,
             p.heading * RAD_TO_DEG, p.speed);
  out.printf("motor counts M1 %ld, M2 %ld; iq %.3f / %.3f A\n", (long)motorcore::position(0),
             (long)motorcore::position(1), motorcore::iq(0), motorcore::iq(1));
  out.printf("imu %s, gyro bias %.5f rad/s, yaw rate %.4f rad/s\n",
             imu.isInitialized() ? "ok" : "MISSING", imu.bias(), imu.yawRate());
  out.printf("plan %s, %.3f s planned for %.3f s budget\n", g_plan.valid ? "valid" : g_plan.error,
             g_plan.plannedTime, g_plan.timeBudget);
}

}  // namespace state
