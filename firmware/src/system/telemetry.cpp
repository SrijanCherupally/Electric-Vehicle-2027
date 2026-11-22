#include "system/telemetry.h"

#include "config.h"
#include "control/controller.h"
#include "control/localizer.h"
#include "motor/drivetrain.h"
#include "motor/motor_core.h"
#include "sensors/battery.h"

namespace telemetry {

namespace {

struct LogRow {
  float t;
  float xRef, x;
  float vRef, v;
  float heading, y;
  float vLeft, vRight;
  float uLeft, uRight;
  float vbat;
};

LogRow g_log[LOG_MAX_SAMPLES];
size_t g_count = 0;
bool g_logging = false;
uint32_t g_lastLogUs = 0;

bool g_live = false;
uint32_t g_lastLiveUs = 0;

}  // namespace

void setLive(bool on) { g_live = on; }
bool live() { return g_live; }

void update() {
  if (!g_live) return;
  const uint32_t now = micros();
  if (now - g_lastLiveUs < 1000000u / TELEMETRY_HZ) return;
  g_lastLiveUs = now;

  const Pose& p = localizer::pose();
  const ControllerDebug& d = controller::debug();
  Serial.printf(">x:%.4f\n>x_ref:%.4f\n>y:%.4f\n>heading_deg:%.2f\n", p.x, d.ref.pos, p.y,
                p.heading * RAD_TO_DEG);
  Serial.printf(">v:%.3f\n>v_ref:%.3f\n>v_left:%.3f\n>v_right:%.3f\n", p.speed, d.ref.vel,
                drivetrain.left().velocity(), drivetrain.right().velocity());
  Serial.printf(">u_left:%.2f\n>u_right:%.2f\n>iq1:%.3f\n>iq2:%.3f\n>vbat:%.2f\n",
                drivetrain.lastLeftCmd(), drivetrain.lastRightCmd(), motorcore::iq(0),
                motorcore::iq(1), battery::voltage());
}

void logStart() {
  g_count = 0;
  g_logging = true;
  g_lastLogUs = micros() - 1000000u;  // first sample immediately
}

void logStop() { g_logging = false; }

void logSample(float t) {
  if (!g_logging || g_count >= LOG_MAX_SAMPLES) return;
  const uint32_t now = micros();
  if (now - g_lastLogUs < 1000000u / LOG_HZ) return;
  g_lastLogUs = now;

  const Pose& p = localizer::pose();
  const ControllerDebug& d = controller::debug();
  g_log[g_count++] = {t,
                      d.ref.pos,
                      p.x,
                      d.ref.vel,
                      p.speed,
                      p.heading,
                      p.y,
                      drivetrain.left().velocity(),
                      drivetrain.right().velocity(),
                      drivetrain.lastLeftCmd(),
                      drivetrain.lastRightCmd(),
                      battery::voltage()};
}

void logDump(Print& out) {
  out.println("t,x_ref,x,v_ref,v,heading_deg,y,v_left,v_right,u_left,u_right,vbat");
  for (size_t i = 0; i < g_count; ++i) {
    const LogRow& r = g_log[i];
    out.printf("%.3f,%.4f,%.4f,%.3f,%.3f,%.2f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f\n", r.t, r.xRef, r.x,
               r.vRef, r.v, r.heading * RAD_TO_DEG, r.y, r.vLeft, r.vRight, r.uLeft, r.uRight,
               r.vbat);
  }
  out.printf("# %u samples\n", (unsigned)g_count);
}

size_t logCount() { return g_count; }

}  // namespace telemetry
