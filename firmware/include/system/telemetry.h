#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Live telemetry (Teleplot format ">name:value", toggle with "tele") and a RAM
// run log that is dumped as CSV after a run ("dump"). The log never prints
// during a run, so serial traffic cannot disturb timing.
namespace telemetry {

void setLive(bool on);
bool live();
void update();  // call every control tick; rate-limits itself

void logStart();
void logStop();
void logSample(float t);  // rate-limited to LOG_HZ
void logDump(Print& out);
size_t logCount();

}  // namespace telemetry

#endif  // TELEMETRY_H
