#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>

// Runtime-tunable parameters. Defaults come from config.h; values can be
// changed over USB serial ("set <name> <value>") and stored in flash ("save").
//
// To add a parameter: add a field here, add one row to the table in
// params.cpp. Nothing else. Saved values are matched by name, so adding,
// removing or reordering rows never scrambles old saves.
struct Params {
  // Event
  float targetDistance;       // m
  float targetTime;           // s
  float bottleLineDistance;   // m beyond Target Point
  float bottleStartDistance;  // m from Start Point (bottle center)

  // Trims
  float distanceTrim;  // m
  float timeTrim;      // s
  float bottleMargin;  // m
  float startDelay;    // s
  float reverseDwell;  // s

  // Geometry
  float wheelDiameter;     // m
  float trackWidth;        // m
  float mpToBottleCenter;  // m

  // Motion
  float maxSpeed;           // m/s
  float accel;              // m/s^2
  float contactSpeed;       // m/s
  float contactZoneBefore;  // m
  float contactZoneAfter;   // m

  // Gains
  float kpPos;
  float kpVel;
  float kiVel;
  float kffVel;
  float kffAcc;
  float kpHeading;
  float kdHeading;
  float kpLateral;
  float maxHeadingCorr;
  float motorVoltageLimit;
  float gyroScale;
};

struct ParamInfo {
  const char* name;
  float* value;
  float defaultValue;
  float minValue;
  float maxValue;
  const char* unit;
  const char* group;
  const char* help;
};

extern Params params;

void paramsInit();      // defaults, then overlay whatever is saved in flash
void paramsDefaults();  // RAM only; "save" to persist
bool paramsSave();
bool paramsLoad();

size_t paramsCount();
const ParamInfo& paramsAt(size_t index);
const ParamInfo* paramsFind(const char* name);
// Returns nullptr on success, otherwise an error message.
const char* paramsSet(const char* name, float value);
void paramsPrint(Print& out);

#endif  // PARAMS_H
