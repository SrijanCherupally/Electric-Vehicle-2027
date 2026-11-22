#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "control/trajectory.h"

// Trajectory tracking controller (core 0, CONTROL_HZ).
//
//   along-track:  v = v_ref + kp_pos * (x_ref - x)
//   heading:      heading target = -kp_lateral * y * direction (clamped),
//                 omega = kp_heading * error - kd_heading * yaw_rate
//   wheels:       v_left/right = v -/+ omega * track_width / 2
//                 -> per-wheel speed loops in Drivetrain
struct ControllerDebug {
  ProfileSample ref;
  float vCmd;
  float omegaCmd;
  float headingTarget;
  float trackingError;
};

namespace controller {

void reset();
// t = seconds since the run started. Returns the reference used.
const ControllerDebug& update(const MotionProfile& profile, float t, float dt);
// Hold a fixed point (after the profile ends).
const ControllerDebug& hold(float x, int8_t lastDir, float dt);
const ControllerDebug& debug();

}  // namespace controller

#endif  // CONTROLLER_H
