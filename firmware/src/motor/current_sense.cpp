#include "motor/current_sense.h"

#include <hardware/adc.h>

#include "config.h"
#include "hardware.h"
#include "motor/drv8317.h"

namespace {

// ADC input number = GPIO - 40 on RP2350B.
constexpr uint ADC_M1_A = PIN_M1_SOA - ADC_FIRST_PIN;
constexpr uint ADC_M1_B = PIN_M1_SOB - ADC_FIRST_PIN;
constexpr uint ADC_M2_A = PIN_M2_SOA - ADC_FIRST_PIN;
constexpr uint ADC_M2_B = PIN_M2_SOB - ADC_FIRST_PIN;
constexpr uint ADC_VBAT = PIN_VBAT_SENSE - ADC_FIRST_PIN;

constexpr float VOLTS_PER_COUNT = ADC_REFERENCE_V / ADC_MAX_COUNTS;

inline uint16_t convert(uint input) {
  adc_select_input(input);
  return adc_read();
}

}  // namespace

void CurrentSense::begin() {
  adc_init();
  const uint pins[] = {PIN_M1_SOA, PIN_M1_SOB, PIN_M1_SOC, PIN_VBAT_SENSE,
                       PIN_M2_SOA, PIN_M2_SOB, PIN_M2_SOC};
  for (uint pin : pins) {
    adc_gpio_init(pin);
  }
}

void CurrentSense::sample() {
  // Interleave motors so both get samples as close to the window center as possible.
  raw_[0][0] = convert(ADC_M1_A);
  raw_[1][0] = convert(ADC_M2_A);
  raw_[0][1] = convert(ADC_M1_B);
  raw_[1][1] = convert(ADC_M2_B);
}

void CurrentSense::calibrateOffsets(uint16_t samples) {
  float sum[MOTORS][2] = {{0, 0}, {0, 0}};
  for (uint16_t i = 0; i < samples; ++i) {
    ThreePhasePwm::waitForPeriodStart();
    sample();
    for (uint8_t m = 0; m < MOTORS; ++m) {
      sum[m][0] += raw_[m][0];
      sum[m][1] += raw_[m][1];
    }
  }
  for (uint8_t m = 0; m < MOTORS; ++m) {
    offset_[m][0] = sum[m][0] / samples;
    offset_[m][1] = sum[m][1] / samples;
  }
}

float CurrentSense::amps(uint8_t motor, uint8_t phase) const {
  const float volts = ((float)raw_[motor][phase] - offset_[motor][phase]) * VOLTS_PER_COUNT;
  return CSA_SIGN * volts / CSA_GAIN_V_PER_A;
}

float CurrentSense::readBatteryVolts() {
  const float sense = (float)convert(ADC_VBAT) * VOLTS_PER_COUNT;
  return sense * VBAT_DIVIDER_RATIO;
}
