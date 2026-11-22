#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

#include "control/run_plan.h"

//   BOOT -> ALIGNING -> CALIBRATING -> READY --(pencil press)--> RUNNING
//                                       ^                          |
//                                       +---(press)--- FINISHED <--+
//   any -> FAULT (driver fault, tracking error, align failure)
//   FAULT --(long press or "clear")--> BOOT (re-align)
//   READY --("mtest"/"kvcal")--> TEST -> READY
enum class State : uint8_t {
  Boot,
  Aligning,
  Calibrating,
  Ready,
  Running,
  Finished,
  Test,
  Fault
};

namespace state {

void init();
void update(float dt);  // core 0, CONTROL_HZ

State current();
const char* name(State s);
const char* faultReason();

const RunPlan& plan();
void replan();  // call after changing params

// Requests (from the CLI; the button is handled inside update()).
bool requestStart();  // same as the pencil press, READY only
void requestAbort();
bool requestRealign();
bool requestGyroCal();
bool requestMotorTest(float volts, float seconds, bool kvCalibrate);
void clearFault();

void printStatus(Print& out);

}  // namespace state

#endif  // STATE_H
