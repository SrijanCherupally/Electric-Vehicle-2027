#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Live telemetry (Teleplot format ">name:value", toggle with "tele") and a RAM
// run log that is dumped as CSV after a run ("dump", refused while moving).
// Live output never blocks: chunks that do not fit in the USB TX FIFO are
// dropped, so a host that is attached but not reading cannot stall the tick.
namespace telemetry {

void setLive(bool on);
bool live();
void update();  // call every control tick; rate-limits itself
uint32_t droppedFrames();  // live chunks dropped because the USB FIFO was full

void logStart();
void logStop();
void logSample(float t);  // rate-limited to LOG_HZ
void logDump(Print& out);
size_t logCount();

}  // namespace telemetry

#endif  // TELEMETRY_H
