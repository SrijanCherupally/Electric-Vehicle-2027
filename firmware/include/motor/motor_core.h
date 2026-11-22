#ifndef MOTOR_CORE_H
#define MOTOR_CORE_H

#include <Arduino.h>

#include "motor/foc.h"

// The motor engine runs on core 1. It owns the DRV8317s, PWM, ADC and encoders,
// and runs FOC for both motors in lock-step with the PWM.
//
// Core 0 talks to it only through the functions below. Every shared value is a
// single aligned 32-bit word, so reads and writes are atomic across cores.

enum class DriveMode : uint8_t {
  Off = 0,  // drivers asleep, motors coast
  Brake,    // drivers awake, zero voltage vector (phases shorted = dynamic brake)
  Run       // FOC active, following setCommands()
};

enum class AlignStatus : uint8_t { NotAligned = 0, Aligning, Aligned, Failed };

enum class MotorFault : uint8_t {
  None = 0,
  DriverFault,     // DRV8317 pulled nFAULT low (OCP / OTP / UVLO)
  Overcurrent,     // software |Iq| limit
  CommandTimeout   // core 0 stopped sending commands while in Run
};

namespace motorcore {

constexpr uint8_t MOTOR_COUNT = 2;

// ----- core 1 -----
void setup();
void loop();

// ----- core 0 -----
bool ready();  // core 1 finished hardware init + current offset calibration

void setMode(DriveMode mode);
DriveMode mode();

// Motor-frame commands (V in voltage mode, A in current mode). Positive
// command increases position(m). Call at the control rate while in Run.
void setCommands(float m1, float m2);

void requestAlign();  // both motors, runs asynchronously on core 1
AlignStatus alignStatus();
AlignResult alignResult(uint8_t motor);
int32_t alignCountsPerErev(uint8_t motor);  // measured during the last alignment

int32_t position(uint8_t motor);  // motor-positive counts
float iq(uint8_t motor);          // filtered, A
float command(uint8_t motor);     // last applied Uq, V
float busVoltage();               // filtered battery voltage, V
float loopRateHz();               // measured FOC rate

MotorFault fault();
const char* faultName(MotorFault f);
void clearFault();  // also returns to Off

}  // namespace motorcore

#endif  // MOTOR_CORE_H
