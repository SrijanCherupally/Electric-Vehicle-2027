#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// Battery pack monitor (8x AA). The ADC conversion itself happens on core 1
// (it owns the ADC); this is the core 0 view with low-voltage logic.
namespace battery {

float voltage();       // filtered pack voltage, V
float cellVoltage();   // voltage() / 8
bool low();            // below VBAT_LOW_WARN_V, with hysteresis

}  // namespace battery

#endif  // BATTERY_H
