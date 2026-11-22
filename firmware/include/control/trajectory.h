#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>

// 1-D motion profile along the track: a list of trapezoidal segments plus
// dwells. Analytic, so sample(t) is exact and cheap at any rate.
//
// A "move" goes from the current end position to a target and starts and ends
// at rest. It can be split into zones with different speed caps (e.g. slow
// down to meet the bottle); the planner joins zones without stopping, using a
// forward / backward pass so every speed change respects the acceleration.

struct ProfileSample {
  float pos;   // m
  float vel;   // m/s
  float acc;   // m/s^2
  int8_t dir;  // +1 forward move, -1 reverse move, 0 dwell
  bool done;   // t is past the end of the profile
};

struct MoveZone {
  float endPos;  // absolute position where this zone ends
  float vCap;    // speed cap in this zone, m/s (> 0)
};

class MotionProfile {
 public:
  static constexpr int MAX_SEGMENTS = 16;
  static constexpr int MAX_ZONES = 6;

  void reset(float startPos);
  bool addDwell(float seconds);
  bool addMove(const MoveZone* zones, int count, float accel);

  ProfileSample sample(float t) const;
  float duration() const { return duration_; }
  float endPosition() const { return endPos_; }
  int segmentCount() const { return count_; }
  float peakSpeed() const;

 private:
  struct Segment {
    float t0;        // start time
    float duration;  // total time
    float p0;        // start position
    int8_t dir;      // +1 / -1 / 0 (dwell)
    float v0, vPeak, v1, accel;
    float tAcc, tCruise, tDec;
    float dAcc, dCruise;
  };

  bool addTrapezoid(float length, int8_t dir, float v0, float vCap, float v1, float accel);

  Segment seg_[MAX_SEGMENTS];
  int count_ = 0;
  float duration_ = 0.0f;
  float endPos_ = 0.0f;
  mutable int cursor_ = 0;
};

#endif  // TRAJECTORY_H
