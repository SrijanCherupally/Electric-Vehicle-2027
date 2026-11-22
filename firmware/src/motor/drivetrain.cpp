#include "motor/drivetrain.h"

#include <math.h>

#include "config.h"
#include "motor/motor_core.h"
#include "system/params.h"

Drivetrain drivetrain;

namespace {

inline float clampf(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }

// Usable command magnitude: user limit, and never more than the bus can make.
float commandLimit() {
  const float busLimit = foc::maxVoltage(motorcore::busVoltage());
  return fminf(params.motorVoltageLimit, busLimit);
}

}  // namespace

// ---------------------------------------------------------------------------
// Wheel
// ---------------------------------------------------------------------------

void Wheel::reset() {
  zeroCount_ = motorcore::position(motor_);
  lastCount_ = zeroCount_;
  first_ = false;
  pos_ = 0.0f;
  vel_ = 0.0f;
  integ_ = 0.0f;
}

void Wheel::update(float dt, float metersPerCount) {
  const int32_t count = motorcore::position(motor_);
  if (first_) {
    zeroCount_ = lastCount_ = count;
    first_ = false;
  }
  const float newPos = dir_ * (float)(count - zeroCount_) * metersPerCount;
  if (dt > 0.0f) {
    const float raw = (newPos - pos_) / dt;
    // One-pole low-pass on the finite difference.
    const float alpha = 1.0f - expf(-2.0f * (float)M_PI * VEL_FILTER_HZ * dt);
    vel_ += alpha * (raw - vel_);
  }
  pos_ = newPos;
  lastCount_ = count;
}

float Wheel::speedLoop(float vTarget, float aTarget, float dt) {
  const float limit = commandLimit();
  const float err = vTarget - vel_;
  integ_ = clampf(integ_ + params.kiVel * err * dt, limit);
  const float u = params.kffVel * vTarget + params.kffAcc * aTarget + params.kpVel * err + integ_;
  return clampf(u, limit);
}

// ---------------------------------------------------------------------------
// Drivetrain
// ---------------------------------------------------------------------------

Drivetrain::Drivetrain() : left_(LEFT_MOTOR_INDEX, LEFT_WHEEL_DIR), right_(RIGHT_MOTOR_INDEX, RIGHT_WHEEL_DIR) {}

float Drivetrain::metersPerCount() const {
  return (float)M_PI * params.wheelDiameter / ((float)ENCODER_CPR * GEAR_RATIO);
}

void Drivetrain::reset() {
  left_.reset();
  right_.reset();
}

void Drivetrain::update(float dt) {
  const float mpc = metersPerCount();
  left_.update(dt, mpc);
  right_.update(dt, mpc);
}

void Drivetrain::driveWheels(float vLeft, float vRight, float aLeft, float aRight, float dt) {
  send(left_.speedLoop(vLeft, aLeft, dt), right_.speedLoop(vRight, aRight, dt));
}

void Drivetrain::driveOpenLoop(float leftCmd, float rightCmd) {
  const float limit = commandLimit();
  send(clampf(leftCmd, limit), clampf(rightCmd, limit));
}

void Drivetrain::zeroCommands() { send(0.0f, 0.0f); }

void Drivetrain::send(float leftCmd, float rightCmd) {
  lastLeft_ = leftCmd;
  lastRight_ = rightCmd;
  float m[motorcore::MOTOR_COUNT] = {0.0f, 0.0f};
  m[left_.motorIndex()] = left_.toMotorFrame(leftCmd);
  m[right_.motorIndex()] = right_.toMotorFrame(rightCmd);
  motorcore::setCommands(m[0], m[1]);
}
