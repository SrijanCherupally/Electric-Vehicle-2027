#ifndef UI_H
#define UI_H

#include <Arduino.h>

// START button (SW2, pencil start) and the status LED.

class StartButton {
 public:
  void begin();
  void update();  // call every control tick

  bool pressed();      // debounced press edge (consumed on read)
  bool longPressed();  // fired once after BUTTON_LONG_PRESS_MS held (consumed)
  bool isDown() const { return stable_; }

 private:
  bool raw_ = false;
  bool stable_ = false;
  uint32_t changedMs_ = 0;
  uint32_t downMs_ = 0;
  bool pressEdge_ = false;
  bool longEdge_ = false;
  bool longFired_ = false;
};

enum class LedPattern : uint8_t {
  Off,
  On,
  SlowBlink,    // ready
  FastBlink,    // aligning / busy
  DoubleBlink,  // finished
  Strobe        // fault
};

class StatusLed {
 public:
  void begin();
  void set(LedPattern p) { pattern_ = p; }
  void update();

 private:
  LedPattern pattern_ = LedPattern::Off;
};

extern StartButton startButton;
extern StatusLed statusLed;

#endif  // UI_H
