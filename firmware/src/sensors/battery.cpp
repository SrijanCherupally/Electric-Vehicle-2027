#include "sensors/battery.h"

#include "config.h"
#include "motor/motor_core.h"

namespace battery {

namespace {
constexpr float CELLS = 8.0f;
constexpr float HYSTERESIS_V = 0.3f;
bool g_low = false;
}  // namespace

float voltage() { return motorcore::busVoltage(); }

float cellVoltage() { return voltage() / CELLS; }

bool low() {
  const float v = voltage();
  if (g_low && v > VBAT_LOW_WARN_V + HYSTERESIS_V) g_low = false;
  if (!g_low && v < VBAT_LOW_WARN_V) g_low = true;
  return g_low;
}

}  // namespace battery
