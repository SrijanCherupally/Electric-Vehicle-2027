#include "system/ui.h"

#include "config.h"
#include "hardware.h"

StartButton startButton;
StatusLed statusLed;

// ---------------------------------------------------------------------------
// StartButton
// ---------------------------------------------------------------------------

void StartButton::begin() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);  // R18 is the real pull-up
  raw_ = stable_ = (digitalRead(PIN_START_BTN) == LOW);
  changedMs_ = millis();
  // Held at boot: do not treat it as a press.
  longFired_ = stable_;
}

void StartButton::update() {
  const uint32_t now = millis();
  const bool down = (digitalRead(PIN_START_BTN) == LOW);
  if (down != raw_) {
    raw_ = down;
    changedMs_ = now;
  }
  if (raw_ != stable_ && (now - changedMs_) >= BUTTON_DEBOUNCE_MS) {
    stable_ = raw_;
    if (stable_) {
      pressEdge_ = true;
      downMs_ = now;
      longFired_ = false;
    }
  }
  if (stable_ && !longFired_ && (now - downMs_) >= BUTTON_LONG_PRESS_MS) {
    longEdge_ = true;
    longFired_ = true;
  }
}

bool StartButton::pressed() {
  const bool e = pressEdge_;
  pressEdge_ = false;
  return e;
}

bool StartButton::longPressed() {
  const bool e = longEdge_;
  longEdge_ = false;
  return e;
}

// ---------------------------------------------------------------------------
// StatusLed
// ---------------------------------------------------------------------------

void StatusLed::begin() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
}

void StatusLed::update() {
  const uint32_t t = millis();
  bool on = false;
  switch (pattern_) {
    case LedPattern::Off: on = false; break;
    case LedPattern::On: on = true; break;
    case LedPattern::SlowBlink: on = (t % 1000) < 100; break;
    case LedPattern::FastBlink: on = (t % 200) < 100; break;
    case LedPattern::DoubleBlink: {
      const uint32_t p = t % 1200;
      on = p < 100 || (p >= 250 && p < 350);
      break;
    }
    case LedPattern::Strobe: on = (t % 125) < 60; break;
  }
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}
