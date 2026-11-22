#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Arduino.h>

// Dead reckoning from the two motor encoders + gyro (no odometry pods yet).
//
// Frame: x = along the track from the Start Point (MP), y = left of the
// imaginary center line, heading = CCW positive, 0 = facing the Target Point.
struct Pose {
  float x;        // m (average wheel travel)
  float y;        // m
  float heading;  // rad
  float yawRate;  // rad/s
  float speed;    // m/s
};

namespace localizer {

void reset();
void update(float dt);
const Pose& pose();
bool usingGyro();

}  // namespace localizer

#endif  // LOCALIZER_H
