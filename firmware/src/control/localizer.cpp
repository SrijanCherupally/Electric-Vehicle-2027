#include "control/localizer.h"

#include <math.h>

#include "config.h"
#include "motor/drivetrain.h"
#include "sensors/imu.h"
#include "system/params.h"

namespace localizer {

namespace {
Pose g_pose = {0, 0, 0, 0, 0, 0};
float g_lastX = 0.0f;
float g_lastWheelDiff = 0.0f;
}  // namespace

void reset() {
  drivetrain.reset();
  g_pose = {0, 0, 0, 0, 0, 0};
  g_lastX = 0.0f;
  g_lastWheelDiff = 0.0f;
}

bool usingGyro() { return imu.isInitialized(); }

void update(float dt) {
  const float x = drivetrain.distance();
  const float dx = x - g_lastX;
  g_lastX = x;

  // Heading increment from the wheels (slips while pushing, but drift-free).
  const float wheelDiff = drivetrain.right().position() - drivetrain.left().position();
  const float dHeadingEnc = (wheelDiff - g_lastWheelDiff) / params.trackWidth;
  g_lastWheelDiff = wheelDiff;

  float dHeading;
  float yawRate;
  if (usingGyro()) {
    yawRate = imu.yawRate();
    const float w = HEADING_ENCODER_WEIGHT;
    dHeading = (1.0f - w) * yawRate * dt + w * dHeadingEnc;
  } else {
    // IMU missing: fall back to wheels only so the car still drives.
    dHeading = dHeadingEnc;
    yawRate = (dt > 0.0f) ? dHeadingEnc / dt : 0.0f;
  }

  // Midpoint integration: project each wheel-travel step onto the track.
  const float midHeading = g_pose.heading + 0.5f * dHeading;
  g_pose.heading += dHeading;
  g_pose.x += dx * cosf(midHeading);
  g_pose.y += dx * sinf(midHeading);
  g_pose.path = x;
  g_pose.yawRate = yawRate;
  g_pose.speed = drivetrain.speed();
}

const Pose& pose() { return g_pose; }

}  // namespace localizer
