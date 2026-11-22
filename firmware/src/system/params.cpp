#include "system/params.h"

#include <EEPROM.h>
#include <string.h>

#include "config.h"

Params params;

namespace {

// clang-format off
const ParamInfo TABLE[] = {
  // name                 field                          default                  min     max     unit     group      help
  {"target_distance",     &params.targetDistance,       TARGET_DISTANCE_M,       0.5f,   12.0f,  "m",     "event",   "Start Point -> Target Point"},
  {"target_time",         &params.targetTime,           TARGET_TIME_S,           2.0f,   40.0f,  "s",     "event",   "Target Time"},
  {"bottle_line",         &params.bottleLineDistance,   BOTTLE_LINE_DISTANCE_M,  0.2f,   4.0f,   "m",     "event",   "Bottle Line beyond Target Point"},
  {"bottle_start",        &params.bottleStartDistance,  BOTTLE_START_DISTANCE_M, 0.3f,   8.0f,   "m",     "event",   "bottle center from Start (2/3/4 m)"},

  {"distance_trim",       &params.distanceTrim,         DISTANCE_TRIM_M,         -0.5f,  0.5f,   "m",     "trim",    "+ = stop farther"},
  {"time_trim",           &params.timeTrim,             TIME_TRIM_S,             -3.0f,  3.0f,   "s",     "trim",    "+ = finish later"},
  {"bottle_margin",       &params.bottleMargin,         BOTTLE_MARGIN_M,         -0.1f,  0.3f,   "m",     "trim",    "bottle near side past the line"},
  {"start_delay",         &params.startDelay,           START_DELAY_S,           0.0f,   5.0f,   "s",     "trim",    "pause after pencil press"},
  {"reverse_dwell",       &params.reverseDwell,         REVERSE_DWELL_S,         0.0f,   5.0f,   "s",     "trim",    "pause before backing up"},

  {"wheel_dia",           &params.wheelDiameter,        WHEEL_DIAMETER_M,        0.01f,  0.3f,   "m",     "geometry","effective wheel diameter"},
  {"track_width",         &params.trackWidth,           TRACK_WIDTH_M,           0.03f,  0.35f,  "m",     "geometry","wheel to wheel"},
  {"mp_to_bottle",        &params.mpToBottleCenter,     MP_TO_BOTTLE_CENTER_M,   0.05f,  0.8f,   "m",     "geometry","MP -> seated bottle center"},

  {"max_speed",           &params.maxSpeed,             MAX_SPEED_MPS,           0.1f,   5.0f,   "m/s",   "motion",  "cruise ceiling"},
  {"accel",               &params.accel,                ACCEL_MPS2,              0.05f,  10.0f,  "m/s^2", "motion",  "accel / decel"},
  {"contact_speed",       &params.contactSpeed,         CONTACT_SPEED_MPS,       0.0f,   5.0f,   "m/s",   "motion",  "speed cap at bottle contact, 0 = off"},
  {"contact_before",      &params.contactZoneBefore,    CONTACT_ZONE_BEFORE_M,   0.0f,   1.0f,   "m",     "motion",  "slow zone before contact"},
  {"contact_after",       &params.contactZoneAfter,     CONTACT_ZONE_AFTER_M,    0.0f,   1.0f,   "m",     "motion",  "slow zone after contact"},

  {"kp_pos",              &params.kpPos,                KP_POS,                  0.0f,   50.0f,  "1/s",   "gains",   "position -> speed"},
  {"kp_vel",              &params.kpVel,                KP_VEL,                  0.0f,   100.0f, "V/(m/s)","gains",  "wheel speed P"},
  {"ki_vel",              &params.kiVel,                KI_VEL,                  0.0f,   1000.0f,"V/m",   "gains",   "wheel speed I"},
  {"kff_vel",             &params.kffVel,               KFF_VEL,                 0.0f,   50.0f,  "V/(m/s)","gains",  "back-EMF feedforward"},
  {"kff_acc",             &params.kffAcc,               KFF_ACC,                 0.0f,   20.0f,  "V/(m/s^2)","gains","accel feedforward"},
  {"kp_heading",          &params.kpHeading,            KP_HEADING,              0.0f,   50.0f,  "1/s",   "gains",   "heading P"},
  {"kd_heading",          &params.kdHeading,            KD_HEADING,              0.0f,   5.0f,   "",      "gains",   "yaw rate damping"},
  {"kp_lateral",          &params.kpLateral,            KP_LATERAL,              0.0f,   50.0f,  "rad/m", "gains",   "lateral drift -> heading"},
  {"max_heading_corr",    &params.maxHeadingCorr,       MAX_HEADING_CORR_RAD,    0.0f,   1.0f,   "rad",   "gains",   "lateral correction clamp"},
  {"volt_limit",          &params.motorVoltageLimit,    MOTOR_VOLTAGE_LIMIT_V,   0.5f,   14.0f,  "V",     "gains",   "per-motor command limit"},
  {"gyro_scale",          &params.gyroScale,            GYRO_SCALE,              0.8f,   1.2f,   "",      "gains",   "yaw rate scale trim"},
};
// clang-format on

constexpr size_t COUNT = sizeof(TABLE) / sizeof(TABLE[0]);

// ---- Flash layout (EEPROM emulation, last flash sector) ----
constexpr uint32_t MAGIC = 0x45563237;  // "EV27"
constexpr size_t EEPROM_BYTES = 1024;

struct Header {
  uint32_t magic;
  uint32_t count;
  uint32_t checksum;
};

struct Entry {
  uint32_t nameHash;
  float value;
};

static_assert(sizeof(Header) + COUNT * sizeof(Entry) <= EEPROM_BYTES, "params do not fit");

uint32_t fnv1a(const char* s) {
  uint32_t h = 2166136261u;
  while (*s) {
    h ^= (uint8_t)*s++;
    h *= 16777619u;
  }
  return h;
}

uint32_t checksum(const Entry* e, size_t n) {
  uint32_t h = 2166136261u;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(e);
  for (size_t i = 0; i < n * sizeof(Entry); ++i) {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

bool eepromStarted = false;
void ensureEeprom() {
  if (!eepromStarted) {
    EEPROM.begin(EEPROM_BYTES);
    eepromStarted = true;
  }
}

}  // namespace

void paramsDefaults() {
  for (const ParamInfo& p : TABLE) *p.value = p.defaultValue;
}

void paramsInit() {
  paramsDefaults();
  paramsLoad();
}

bool paramsSave() {
  ensureEeprom();
  Entry entries[COUNT];
  for (size_t i = 0; i < COUNT; ++i) {
    entries[i].nameHash = fnv1a(TABLE[i].name);
    entries[i].value = *TABLE[i].value;
  }
  Header h{MAGIC, (uint32_t)COUNT, checksum(entries, COUNT)};
  EEPROM.put(0, h);
  for (size_t i = 0; i < COUNT; ++i) {
    EEPROM.put(sizeof(Header) + i * sizeof(Entry), entries[i]);
  }
  return EEPROM.commit();
}

bool paramsLoad() {
  ensureEeprom();
  Header h;
  EEPROM.get(0, h);
  if (h.magic != MAGIC || h.count == 0 || h.count > (EEPROM_BYTES - sizeof(Header)) / sizeof(Entry)) {
    return false;
  }
  static Entry stored[(EEPROM_BYTES - sizeof(Header)) / sizeof(Entry)];
  for (uint32_t i = 0; i < h.count; ++i) {
    EEPROM.get(sizeof(Header) + i * sizeof(Entry), stored[i]);
  }
  if (checksum(stored, h.count) != h.checksum) return false;

  for (uint32_t i = 0; i < h.count; ++i) {
    for (const ParamInfo& p : TABLE) {
      if (fnv1a(p.name) == stored[i].nameHash) {
        const float v = stored[i].value;
        if (isfinite(v) && v >= p.minValue && v <= p.maxValue) *p.value = v;
        break;
      }
    }
  }
  return true;
}

size_t paramsCount() { return COUNT; }
const ParamInfo& paramsAt(size_t index) { return TABLE[index < COUNT ? index : 0]; }

const ParamInfo* paramsFind(const char* name) {
  for (const ParamInfo& p : TABLE) {
    if (strcasecmp(p.name, name) == 0) return &p;
  }
  return nullptr;
}

const char* paramsSet(const char* name, float value) {
  const ParamInfo* p = paramsFind(name);
  if (p == nullptr) return "unknown parameter (type 'params' for the list)";
  if (!isfinite(value)) return "not a number";
  if (value < p->minValue || value > p->maxValue) return "out of range";
  *p->value = value;
  return nullptr;
}

void paramsPrint(Print& out) {
  const char* group = "";
  for (const ParamInfo& p : TABLE) {
    if (strcmp(group, p.group) != 0) {
      group = p.group;
      out.printf("[%s]\n", group);
    }
    const bool changed = fabsf(*p.value - p.defaultValue) > 1e-6f;
    out.printf("  %-18s %10.4f %-9s %s%s\n", p.name, *p.value, p.unit, p.help,
               changed ? "  *" : "");
  }
  out.println("  (* = differs from config.h default)");
}
