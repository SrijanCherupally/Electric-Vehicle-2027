#include "control/run_plan.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

namespace {

// Builds the full profile for one cruise speed. Returns false on a planner error.
bool buildProfile(const Params& p, const RunPlan& geo, float cruise, MotionProfile& prof,
                  float* forwardTime, float* reverseTime) {
  prof.reset(0.0f);
  if (!prof.addDwell(p.startDelay)) return false;

  // ---- Forward: approach, meet the bottle slowly, push it out ----
  MoveZone zones[3];
  int n = 0;
  const float contactCap = (p.contactSpeed > 0.0f) ? fminf(p.contactSpeed, cruise) : cruise;
  const float slowStart = fmaxf(0.0f, geo.xContact - p.contactZoneBefore);
  const float slowEnd = fminf(geo.xPush, geo.xContact + p.contactZoneAfter);
  if (p.contactSpeed > 0.0f && slowEnd > slowStart) {
    if (slowStart > 0.0f) zones[n++] = {slowStart, cruise};
    zones[n++] = {slowEnd, contactCap};
    if (geo.xPush > slowEnd) zones[n++] = {geo.xPush, cruise};
  } else {
    zones[n++] = {geo.xPush, cruise};
  }
  const float t0 = prof.duration();
  if (!prof.addMove(zones, n, p.accel)) return false;
  *forwardTime = prof.duration() - t0;

  if (!prof.addDwell(p.reverseDwell)) return false;

  // ---- Reverse: back the MP onto the Target Point ----
  const MoveZone back = {geo.xFinal, cruise};
  const float t1 = prof.duration();
  if (!prof.addMove(&back, 1, p.accel)) return false;
  *reverseTime = prof.duration() - t1;
  return true;
}

void setMsg(char* dst, size_t n, const char* msg) {
  strncpy(dst, msg, n - 1);
  dst[n - 1] = '\0';
}

}  // namespace

bool buildRunPlan(const Params& p, RunPlan& out) {
  out.valid = false;
  out.onTime = false;
  out.error[0] = '\0';
  out.warning[0] = '\0';

  // ---- Geometry ----
  const float r = 0.5f * BOTTLE_DIAMETER_M;
  out.xContact = p.bottleStartDistance - p.mpToBottleCenter;
  out.bottleLinePos = p.targetDistance + p.bottleLineDistance;
  out.bottleFinalCenter = out.bottleLinePos + r + p.bottleMargin;
  out.xPush = out.bottleFinalCenter - p.mpToBottleCenter;
  out.xFinal = p.targetDistance + p.distanceTrim;
  out.timeBudget = p.targetTime + p.timeTrim;

  if (out.xContact <= 0.0f) {
    setMsg(out.error, sizeof(out.error), "bottle_start is inside the pusher (bottle_start <= mp_to_bottle)");
    return false;
  }
  if (out.xPush <= out.xContact) {
    setMsg(out.error, sizeof(out.error), "push end is before bottle contact");
    return false;
  }
  if (out.xPush <= out.xFinal) {
    setMsg(out.error, sizeof(out.error), "push end is before the target (bottle_line too small)");
    return false;
  }
  out.pathLength = out.xPush + (out.xPush - out.xFinal);

  // ---- Rule range warnings (still runnable) ----
  if (p.targetDistance < RULE_TARGET_DISTANCE_MIN_M || p.targetDistance > RULE_TARGET_DISTANCE_MAX_M) {
    setMsg(out.warning, sizeof(out.warning), "target_distance outside the 5-9 m rule range");
  } else if (p.targetTime < RULE_TARGET_TIME_MIN_S || p.targetTime > RULE_TARGET_TIME_MAX_S) {
    setMsg(out.warning, sizeof(out.warning), "target_time outside the 10-20 s rule range");
  } else if (p.bottleLineDistance < RULE_BOTTLE_LINE_MIN_M || p.bottleLineDistance > RULE_BOTTLE_LINE_MAX_M) {
    setMsg(out.warning, sizeof(out.warning), "bottle_line outside the 1-3 m rule range");
  }

  // ---- Solve for the cruise speed that meets the time budget ----
  // Run time decreases monotonically with cruise speed, so bisect.
  float fwd = 0.0f, rev = 0.0f;
  float hi = p.maxSpeed;
  float lo = fminf(MIN_CRUISE_SPEED_MPS, hi);

  if (!buildProfile(p, out, hi, out.profile, &fwd, &rev)) {
    setMsg(out.error, sizeof(out.error), "profile build failed");
    return false;
  }
  if (out.profile.duration() > out.timeBudget) {
    // Too slow even at max speed: run flat out and say so.
    out.onTime = false;
    out.cruiseSpeed = hi;
    if (out.warning[0] == '\0') {
      snprintf(out.warning, sizeof(out.warning), "cannot finish in %.2f s at max_speed (needs %.2f s)",
               out.timeBudget, out.profile.duration());
    }
  } else {
    for (int i = 0; i < 40; ++i) {
      const float mid = 0.5f * (lo + hi);
      MotionProfile& trial = out.profile;
      if (!buildProfile(p, out, mid, trial, &fwd, &rev)) break;
      if (trial.duration() > out.timeBudget) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    out.cruiseSpeed = hi;
    out.onTime = true;
  }

  if (!buildProfile(p, out, out.cruiseSpeed, out.profile, &fwd, &rev)) {
    setMsg(out.error, sizeof(out.error), "profile build failed");
    return false;
  }
  // If even the slowest cruise finishes early, wait at the push point.
  if (out.onTime && out.profile.duration() < out.timeBudget - 0.005f) {
    Params slower = p;
    slower.reverseDwell += out.timeBudget - out.profile.duration();
    buildProfile(slower, out, out.cruiseSpeed, out.profile, &fwd, &rev);
  }

  out.forwardTime = fwd;
  out.reverseTime = rev;
  out.plannedTime = out.profile.duration();
  out.valid = true;
  return true;
}

void printRunPlan(const RunPlan& plan, Print& out) {
  out.println("---- run plan ----");
  if (!plan.valid) {
    out.printf("INVALID: %s\n", plan.error);
    return;
  }
  out.printf("  bottle contact  MP at %7.3f m\n", plan.xContact);
  out.printf("  bottle line        at %7.3f m (bottle center ends at %.3f m)\n", plan.bottleLinePos,
             plan.bottleFinalCenter);
  out.printf("  push end        MP at %7.3f m\n", plan.xPush);
  out.printf("  final stop      MP at %7.3f m\n", plan.xFinal);
  out.printf("  path length         %7.3f m\n", plan.pathLength);
  out.printf("  cruise speed        %7.3f m/s (peak %.3f)\n", plan.cruiseSpeed, plan.profile.peakSpeed());
  out.printf("  time: forward %.2f s, reverse %.2f s, total %.3f s (budget %.3f s) %s\n",
             plan.forwardTime, plan.reverseTime, plan.plannedTime, plan.timeBudget,
             plan.onTime ? "OK" : "LATE");
  if (plan.warning[0]) out.printf("  WARNING: %s\n", plan.warning);
}
