#include "control/trajectory.h"

#include <math.h>

void MotionProfile::reset(float startPos) {
  count_ = 0;
  duration_ = 0.0f;
  endPos_ = startPos;
  cursor_ = 0;
}

bool MotionProfile::addDwell(float seconds) {
  if (seconds <= 0.0f) return true;
  if (count_ >= MAX_SEGMENTS) return false;
  Segment& s = seg_[count_++];
  s = Segment{};
  s.t0 = duration_;
  s.duration = seconds;
  s.p0 = endPos_;
  s.dir = 0;
  duration_ += seconds;
  return true;
}

bool MotionProfile::addTrapezoid(float length, int8_t dir, float v0, float vCap, float v1,
                                 float a) {
  if (length <= 1e-6f) return true;
  if (count_ >= MAX_SEGMENTS || a <= 0.0f || vCap <= 0.0f) return false;

  // Peak speed: cap, unless the segment is too short to reach it.
  float vp = vCap;
  float dAcc = (vp * vp - v0 * v0) / (2.0f * a);
  float dDec = (vp * vp - v1 * v1) / (2.0f * a);
  if (dAcc + dDec > length) {
    vp = sqrtf((2.0f * a * length + v0 * v0 + v1 * v1) * 0.5f);
    vp = fmaxf(vp, fmaxf(v0, v1));
    dAcc = (vp * vp - v0 * v0) / (2.0f * a);
    dDec = (vp * vp - v1 * v1) / (2.0f * a);
  }
  const float dCruise = fmaxf(0.0f, length - dAcc - dDec);

  Segment& s = seg_[count_++];
  s.t0 = duration_;
  s.p0 = endPos_;
  s.dir = dir;
  s.v0 = v0;
  s.vPeak = vp;
  s.v1 = v1;
  s.accel = a;
  s.tAcc = (vp - v0) / a;
  s.tDec = (vp - v1) / a;
  s.tCruise = (vp > 1e-6f) ? dCruise / vp : 0.0f;
  s.dAcc = dAcc;
  s.dCruise = dCruise;
  s.duration = s.tAcc + s.tCruise + s.tDec;

  duration_ += s.duration;
  endPos_ += dir * length;
  return true;
}

bool MotionProfile::addMove(const MoveZone* zones, int n, float a) {
  if (n <= 0 || n > MAX_ZONES || a <= 0.0f) return false;

  const float start = endPos_;
  const float target = zones[n - 1].endPos;
  const int8_t dir = (target >= start) ? 1 : -1;

  // Zone lengths along the direction of travel.
  float len[MAX_ZONES];
  float prev = start;
  for (int i = 0; i < n; ++i) {
    len[i] = fmaxf(0.0f, dir * (zones[i].endPos - prev));
    prev = zones[i].endPos;
  }

  // Boundary speeds: v[0] = v[n] = 0, interior = min of the adjacent caps,
  // then limit by what acceleration can reach from either side.
  float v[MAX_ZONES + 1];
  v[0] = 0.0f;
  v[n] = 0.0f;
  for (int i = 1; i < n; ++i) v[i] = fminf(zones[i - 1].vCap, zones[i].vCap);
  for (int i = 0; i < n; ++i) v[i + 1] = fminf(v[i + 1], sqrtf(v[i] * v[i] + 2.0f * a * len[i]));
  for (int i = n - 1; i >= 0; --i) v[i] = fminf(v[i], sqrtf(v[i + 1] * v[i + 1] + 2.0f * a * len[i]));

  for (int i = 0; i < n; ++i) {
    if (!addTrapezoid(len[i], dir, v[i], zones[i].vCap, v[i + 1], a)) return false;
  }
  return true;
}

float MotionProfile::peakSpeed() const {
  float vp = 0.0f;
  for (int i = 0; i < count_; ++i) vp = fmaxf(vp, seg_[i].vPeak);
  return vp;
}

ProfileSample MotionProfile::sample(float t) const {
  if (count_ == 0) return {endPos_, 0.0f, 0.0f, 0, true};
  if (t >= duration_) return {endPos_, 0.0f, 0.0f, 0, true};
  if (t < 0.0f) t = 0.0f;

  // Time only moves forward during a run, so start from the last segment used.
  if (cursor_ >= count_ || seg_[cursor_].t0 > t) cursor_ = 0;
  while (cursor_ < count_ - 1 && t >= seg_[cursor_].t0 + seg_[cursor_].duration) ++cursor_;
  const Segment& s = seg_[cursor_];

  if (s.dir == 0) return {s.p0, 0.0f, 0.0f, 0, false};

  const float tau = t - s.t0;
  float d, v, acc;
  if (tau < s.tAcc) {
    v = s.v0 + s.accel * tau;
    d = s.v0 * tau + 0.5f * s.accel * tau * tau;
    acc = s.accel;
  } else if (tau < s.tAcc + s.tCruise) {
    const float tc = tau - s.tAcc;
    v = s.vPeak;
    d = s.dAcc + s.vPeak * tc;
    acc = 0.0f;
  } else {
    float td = tau - s.tAcc - s.tCruise;
    if (td > s.tDec) td = s.tDec;
    v = s.vPeak - s.accel * td;
    d = s.dAcc + s.dCruise + s.vPeak * td - 0.5f * s.accel * td * td;
    acc = -s.accel;
  }
  return {s.p0 + s.dir * d, s.dir * v, s.dir * acc, s.dir, false};
}
