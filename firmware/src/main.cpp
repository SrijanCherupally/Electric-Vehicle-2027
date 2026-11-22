// EV Controller 2027 firmware (Science Olympiad Div C Electric Vehicle)
//
// Core 0: params, serial commands, IMU, localization, motion control, state machine.
// Core 1: FOC motor engine (PWM, current sense, encoders) for both motors.
//
// See firmware/README.md for setup, tuning and the competition-day checklist.

#include <Arduino.h>

#include "config.h"
#include "control/localizer.h"
#include "motor/drivetrain.h"
#include "motor/motor_core.h"
#include "sensors/imu.h"
#include "system/commands.h"
#include "system/params.h"
#include "system/state.h"
#include "system/telemetry.h"
#include "system/ui.h"

namespace {

constexpr uint32_t CONTROL_PERIOD_US = 1000000u / CONTROL_HZ;
uint32_t g_nextTickUs = 0;
uint32_t g_lastTickUs = 0;

void controlTick() {
  const uint32_t now = micros();
  float dt = (now - g_lastTickUs) * 1e-6f;
  g_lastTickUs = now;
  if (dt <= 0.0f || dt > 0.05f) dt = CONTROL_DT_S;  // first tick / after a stall

  startButton.update();
  imu.update();
  drivetrain.update(dt);
  localizer::update(dt);
  state::update(dt);
  telemetry::update();
  statusLed.update();
}

}  // namespace

// ============================== core 0 ==============================

void setup() {
  Serial.begin(SERIAL_BAUD);
  statusLed.begin();
  startButton.begin();

  // Give a USB host a moment to attach so boot messages are visible,
  // without ever blocking an untethered competition boot.
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 1500) {
    statusLed.set(LedPattern::FastBlink);
    statusLed.update();
    delay(10);
  }

  Serial.println("\n==== EV Controller 2027 ====");
  paramsInit();
  if (!imu.begin()) {
    Serial.println("IMU missing: heading will come from the wheels only");
  }
  state::init();
  commands::printHelp(Serial);

  g_lastTickUs = micros();
  g_nextTickUs = g_lastTickUs + CONTROL_PERIOD_US;
}

void loop() {
  commands::poll();

  // Fixed-rate control tick. If we fall behind, skip ahead instead of bursting.
  const uint32_t now = micros();
  if ((int32_t)(now - g_nextTickUs) >= 0) {
    g_nextTickUs += CONTROL_PERIOD_US;
    if ((int32_t)(now - g_nextTickUs) > (int32_t)(5 * CONTROL_PERIOD_US)) {
      g_nextTickUs = now + CONTROL_PERIOD_US;
    }
    controlTick();
  }
}

// ============================== core 1 ==============================

void setup1() { motorcore::setup(); }

void loop1() { motorcore::loop(); }
