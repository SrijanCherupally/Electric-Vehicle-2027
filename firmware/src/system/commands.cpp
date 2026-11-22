#include "system/commands.h"

#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "motor/motor_core.h"
#include "system/params.h"
#include "system/state.h"
#include "system/telemetry.h"

namespace commands {

namespace {

constexpr size_t LINE_MAX = 96;
constexpr int ARGS_MAX = 4;
char g_line[LINE_MAX];
size_t g_len = 0;

bool parseFloat(const char* s, float& out) {
  if (s == nullptr) return false;
  char* end = nullptr;
  out = strtof(s, &end);
  return end != s && *end == '\0';
}

bool notWhileRunning() {
  const State s = state::current();
  if (s == State::Running || s == State::Test) {
    Serial.println("busy: not allowed while the car is moving ('stop' first)");
    return true;
  }
  return false;
}

void cmdSet(int argc, char** argv) {
  float v;
  if (argc != 3 || !parseFloat(argv[2], v)) {
    Serial.println("usage: set <name> <value>   e.g. set target_distance 7.25");
    return;
  }
  if (notWhileRunning()) return;
  const char* err = paramsSet(argv[1], v);
  if (err) {
    const ParamInfo* p = paramsFind(argv[1]);
    if (p) {
      Serial.printf("error: %s (range %.3f .. %.3f %s)\n", err, p->minValue, p->maxValue, p->unit);
    } else {
      Serial.printf("error: %s\n", err);
    }
    return;
  }
  const ParamInfo* p = paramsFind(argv[1]);
  Serial.printf("%s = %.4f %s (not saved, 'save' to keep)\n", p->name, *p->value, p->unit);
  state::replan();
  const RunPlan& plan = state::plan();
  if (!plan.valid) {
    Serial.printf("WARNING: plan now invalid: %s\n", plan.error);
  } else {
    Serial.printf("plan: cruise %.3f m/s, %.3f s %s\n", plan.cruiseSpeed, plan.plannedTime,
                  plan.onTime ? "" : "(LATE)");
    if (plan.warning[0]) Serial.printf("WARNING: %s\n", plan.warning);
  }
}

void cmdGet(int argc, char** argv) {
  if (argc != 2) {
    Serial.println("usage: get <name>");
    return;
  }
  const ParamInfo* p = paramsFind(argv[1]);
  if (!p) {
    Serial.println("unknown parameter");
    return;
  }
  Serial.printf("%s = %.4f %s (default %.4f) %s\n", p->name, *p->value, p->unit, p->defaultValue,
                p->help);
}

void cmdLevel(int argc, char** argv) {
  if (argc != 2) {
    Serial.println("usage: level regional|state|national");
    return;
  }
  if (notWhileRunning()) return;
  float d;
  if (strncasecmp(argv[1], "reg", 3) == 0 || strncasecmp(argv[1], "inv", 3) == 0) {
    d = BOTTLE_START_REGIONAL_M;
  } else if (strncasecmp(argv[1], "sta", 3) == 0) {
    d = BOTTLE_START_STATE_M;
  } else if (strncasecmp(argv[1], "nat", 3) == 0) {
    d = BOTTLE_START_NATIONAL_M;
  } else {
    Serial.println("level must be regional, state or national");
    return;
  }
  params.bottleStartDistance = d;
  state::replan();
  Serial.printf("bottle_start = %.2f m\n", d);
  printRunPlan(state::plan(), Serial);
}

void cmdMotorTest(int argc, char** argv, bool kv) {
  float volts = kv ? 2.0f : 1.0f;
  float seconds = 3.0f;
  if (argc >= 2 && !parseFloat(argv[1], volts)) {
    Serial.println("usage: mtest [volts] [seconds]  |  kvcal [volts] [seconds]");
    return;
  }
  if (argc >= 3 && !parseFloat(argv[2], seconds)) {
    Serial.println("bad seconds");
    return;
  }
  if (fabsf(volts) > params.motorVoltageLimit) volts = copysignf(params.motorVoltageLimit, volts);
  if (seconds < 0.5f) seconds = 0.5f;
  if (seconds > 10.0f) seconds = 10.0f;
  if (!state::requestMotorTest(volts, seconds, kv)) {
    Serial.println("motor test needs READY state");
    return;
  }
  Serial.printf("%s: %.2f V for %.1f s. WHEELS OFF THE GROUND.\n", kv ? "kvcal" : "mtest", volts,
                seconds);
}

void execute(int argc, char** argv) {
  const char* c = argv[0];
  if (!strcasecmp(c, "help") || !strcmp(c, "?")) {
    printHelp(Serial);
  } else if (!strcasecmp(c, "status") || !strcasecmp(c, "s")) {
    state::printStatus(Serial);
  } else if (!strcasecmp(c, "params") || !strcasecmp(c, "p")) {
    paramsPrint(Serial);
  } else if (!strcasecmp(c, "set")) {
    cmdSet(argc, argv);
  } else if (!strcasecmp(c, "get")) {
    cmdGet(argc, argv);
  } else if (!strcasecmp(c, "save")) {
    if (notWhileRunning()) return;
    Serial.println(paramsSave() ? "saved to flash" : "save FAILED");
  } else if (!strcasecmp(c, "load")) {
    if (notWhileRunning()) return;
    Serial.println(paramsLoad() ? "loaded from flash" : "nothing saved (using defaults)");
    state::replan();
  } else if (!strcasecmp(c, "defaults")) {
    if (notWhileRunning()) return;
    paramsDefaults();
    state::replan();
    Serial.println("defaults restored in RAM ('save' to persist)");
  } else if (!strcasecmp(c, "level")) {
    cmdLevel(argc, argv);
  } else if (!strcasecmp(c, "plan")) {
    printRunPlan(state::plan(), Serial);
  } else if (!strcasecmp(c, "go")) {
    Serial.println(state::requestStart() ? "GO" : "cannot start (needs READY + valid plan)");
  } else if (!strcasecmp(c, "stop") || !strcasecmp(c, "x")) {
    state::requestAbort();
  } else if (!strcasecmp(c, "align")) {
    Serial.println(state::requestRealign() ? "re-aligning" : "busy");
  } else if (!strcasecmp(c, "gyrocal")) {
    Serial.println(state::requestGyroCal() ? "calibrating gyro, keep still" : "needs READY");
  } else if (!strcasecmp(c, "clear")) {
    state::clearFault();
  } else if (!strcasecmp(c, "tele")) {
    bool on = !telemetry::live();
    if (argc >= 2) on = !strcasecmp(argv[1], "on") || !strcmp(argv[1], "1");
    telemetry::setLive(on);
    Serial.printf("live telemetry %s\n", on ? "on" : "off");
  } else if (!strcasecmp(c, "dump")) {
    if (notWhileRunning()) return;  // ~100 kB of output would stall the control tick
    telemetry::logDump(Serial);
  } else if (!strcasecmp(c, "mtest")) {
    cmdMotorTest(argc, argv, false);
  } else if (!strcasecmp(c, "kvcal")) {
    cmdMotorTest(argc, argv, true);
  } else {
    Serial.printf("unknown command '%s' (type 'help')\n", c);
  }
}

void handleLine(char* line) {
  char* argv[ARGS_MAX];
  int argc = 0;
  for (char* tok = strtok(line, " \t"); tok && argc < ARGS_MAX; tok = strtok(nullptr, " \t")) {
    argv[argc++] = tok;
  }
  if (argc > 0) execute(argc, argv);
}

}  // namespace

void poll() {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (g_len > 0) {
        g_line[g_len] = '\0';
        Serial.printf("> %s\n", g_line);
        handleLine(g_line);
        g_len = 0;
      }
    } else if (ch == 8 || ch == 127) {
      if (g_len > 0) --g_len;
    } else if (g_len < LINE_MAX - 1 && ch >= 32) {
      g_line[g_len++] = (char)ch;
    }
  }
}

void printHelp(Print& out) {
  out.println(
      "---- EV 2027 commands ----\n"
      "  status | s              state, battery, pose, faults\n"
      "  params | p              list every tunable parameter\n"
      "  set <name> <value>      change a parameter (replans immediately)\n"
      "  get <name>              show one parameter\n"
      "  save | load | defaults  flash persistence\n"
      "  level reg|state|nat     set bottle start (2 / 3 / 4 m)\n"
      "  plan                    print the computed run\n"
      "  go                      start a run (bench testing; pencil button does the same)\n"
      "  stop | x                abort run / test\n"
      "  align                   redo FOC alignment\n"
      "  gyrocal                 redo gyro bias (keep still)\n"
      "  clear                   clear a fault (then re-aligns)\n"
      "  tele [on|off]           live Teleplot telemetry\n"
      "  dump                    print last run log as CSV\n"
      "  mtest [V] [s]           open-loop spin test (wheels up)\n"
      "  kvcal [V] [s]           measure kff_vel (wheels up)\n"
      "Competition: set target_distance / target_time / bottle_line, level, plan, save.");
}

}  // namespace commands
