#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

// Core 0 view of the two drive wheels: turns motor counts into wheel meters,
// runs the per-wheel speed loops, and sends commands to the motor core.
// Everything here is in "vehicle forward" units (m, m/s): the left/right
// mapping and mounting signs from config.h are applied once, here.

class Wheel {
 public:
  Wheel(uint8_t motorIndex, float dirSign) : motor_(motorIndex), dir_(dirSign) {}

  void reset();                      // zero distance + loop state
  void update(float dt, float metersPerCount);
  float speedLoop(float vTarget, float aTarget, float dt);  // -> motor-frame command

  float position() const { return pos_; }  // m, forward positive
  float velocity() const { return vel_; }  // m/s, filtered
  uint8_t motorIndex() const { return motor_; }
  float toMotorFrame(float cmd) const { return dir_ * cmd; }

 private:
  uint8_t motor_;
  float dir_;
  int32_t zeroCount_ = 0;
  int32_t lastCount_ = 0;
  bool first_ = true;
  float pos_ = 0.0f;
  float vel_ = 0.0f;
  float integ_ = 0.0f;
};

class Drivetrain {
 public:
  Drivetrain();

  void reset();  // zero both wheels
  void update(float dt);

  // Closed loop: wheel speed + accel targets (m/s, m/s^2).
  void driveWheels(float vLeft, float vRight, float aLeft, float aRight, float dt);
  // Open loop: same command to both wheels, vehicle-forward volts.
  void driveOpenLoop(float leftCmd, float rightCmd);
  void zeroCommands();

  const Wheel& left() const { return left_; }
  const Wheel& right() const { return right_; }
  float distance() const { return 0.5f * (left_.position() + right_.position()); }
  float speed() const { return 0.5f * (left_.velocity() + right_.velocity()); }
  float lastLeftCmd() const { return lastLeft_; }
  float lastRightCmd() const { return lastRight_; }
  float metersPerCount() const;

 private:
  void send(float leftCmd, float rightCmd);
  Wheel left_;
  Wheel right_;
  float lastLeft_ = 0.0f;
  float lastRight_ = 0.0f;
};

extern Drivetrain drivetrain;

#endif  // DRIVETRAIN_H
