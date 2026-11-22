#include "control/controller.h"

#include <math.h>

#include "control/localizer.h"
#include "motor/drivetrain.h"
#include "system/params.h"

namespace controller {

namespace {

ControllerDebug g_dbg = {};
int8_t g_lastDir = 1;

inline float clampf(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }

inline float wrapPi(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

void track(const ProfileSample& ref, float dt) {
  const Pose& p = localizer::pose();
  if (ref.dir != 0) g_lastDir = ref.dir;

  // Along-track
  const float err = ref.pos - p.x;
  const float v = ref.vel + params.kpPos * err;

  // Lateral -> heading target. Reversing flips how heading moves y.
  const float headingTarget = clampf(-params.kpLateral * p.y * (float)g_lastDir, params.maxHeadingCorr);
  const float hErr = wrapPi(headingTarget - p.heading);
  const float omega = params.kpHeading * hErr - params.kdHeading * p.yawRate;

  const float half = 0.5f * params.trackWidth;
  const float aRot = 0.0f;  // no rotational accel feedforward
  drivetrain.driveWheels(v - omega * half, v + omega * half, ref.acc - aRot, ref.acc + aRot, dt);

  g_dbg.ref = ref;
  g_dbg.vCmd = v;
  g_dbg.omegaCmd = omega;
  g_dbg.headingTarget = headingTarget;
  g_dbg.trackingError = err;
}

}  // namespace

void reset() {
  g_dbg = {};
  g_lastDir = 1;
}

const ControllerDebug& update(const MotionProfile& profile, float t, float dt) {
  track(profile.sample(t), dt);
  return g_dbg;
}

const ControllerDebug& hold(float x, int8_t lastDir, float dt) {
  if (lastDir != 0) g_lastDir = lastDir;
  track({x, 0.0f, 0.0f, 0, true}, dt);
  return g_dbg;
}

const ControllerDebug& debug() { return g_dbg; }

}  // namespace controller
