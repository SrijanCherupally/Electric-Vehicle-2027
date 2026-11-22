#ifndef RUN_PLAN_H
#define RUN_PLAN_H

#include <Arduino.h>

#include "control/trajectory.h"
#include "system/params.h"

// The 2027 run, as positions of the Measurement Point (MP) from the Start Point:
//
//   0 ------ xContact ====push==== xPush
//                                     \___ reverse ___ xFinal (= Target Point)
//
//   xContact: pusher meets the bottle (bottle center - MP-to-bottle offset)
//   xPush:    bottle's near side is bottle_margin past the Bottle Line
//   xFinal:   target distance + distance trim
//
// The planner picks one cruise speed so the whole run (start delay + push +
// dwell + reverse) takes target_time + time_trim.

struct RunPlan {
  bool valid = false;     // geometry makes sense; the run can start
  bool onTime = false;    // the time budget is achievable under max_speed
  char error[96] = {0};   // why it is invalid
  char warning[96] = {0}; // non-fatal note (rule range, time not achievable)

  float xContact = 0.0f;
  float xPush = 0.0f;
  float xFinal = 0.0f;
  float bottleLinePos = 0.0f;
  float bottleFinalCenter = 0.0f;

  float timeBudget = 0.0f;
  float cruiseSpeed = 0.0f;
  float plannedTime = 0.0f;
  float forwardTime = 0.0f;
  float reverseTime = 0.0f;
  float pathLength = 0.0f;

  MotionProfile profile;
};

bool buildRunPlan(const Params& p, RunPlan& out);
void printRunPlan(const RunPlan& plan, Print& out);

#endif  // RUN_PLAN_H
