#include "motor/current_sense.h"

#include <hardware/adc.h>
#include <hardware/clocks.h>

#include "config.h"
#include "hardware.h"

namespace {

// ADC input number = GPIO - 40 on RP2350B.
constexpr uint ADC_PHASE[CurrentSense::MOTORS][3] = {
    {PIN_M1_SOA - ADC_FIRST_PIN, PIN_M1_SOB - ADC_FIRST_PIN, PIN_M1_SOC - ADC_FIRST_PIN},
    {PIN_M2_SOA - ADC_FIRST_PIN, PIN_M2_SOB - ADC_FIRST_PIN, PIN_M2_SOC - ADC_FIRST_PIN},
};
constexpr uint ADC_VBAT = PIN_VBAT_SENSE - ADC_FIRST_PIN;

constexpr float VOLTS_PER_COUNT = ADC_REFERENCE_V / ADC_MAX_COUNTS;

uint32_t g_marginCounts = 0;  // CSA_SAMPLE_MARGIN_US in PWM clocks

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
  // PWM runs at clkdiv 1, so the counter ticks at clk_sys.
  g_marginCounts = (uint32_t)(CSA_SAMPLE_MARGIN_US * 1e-6f * (float)clock_get_hz(clk_sys) + 0.5f);
}

void CurrentSense::sample(const ThreePhasePwm* pwm) {
  // Per motor: skip the phase with the highest duty (lowest level = narrowest
  // window); convert the narrower of the other two first.
  uint8_t first[MOTORS], second[MOTORS], skip[MOTORS];
  for (uint8_t m = 0; m < MOTORS; ++m) {
    uint8_t s = 0;
    if (pwm[m].level(1) < pwm[m].level(s)) s = 1;
    if (pwm[m].level(2) < pwm[m].level(s)) s = 2;
    const uint8_t a = (s == 0) ? 1 : 0;
    const uint8_t b = (s == 2) ? 1 : 2;
    skip[m] = s;
    if (pwm[m].level(a) <= pwm[m].level(b)) {
      first[m] = a;
      second[m] = b;
    } else {
      first[m] = b;
      second[m] = a;
    }
  }

  // Slot order: M1 first, M2 first, M1 second, M2 second. The counter read
  // right after each conversion bounds when that conversion's sample was taken.
  uint16_t raw[MOTORS][3];
  bool ok[MOTORS] = {true, true};
  const uint8_t slotMotor[4] = {0, 1, 0, 1};
  for (uint8_t k = 0; k < 4; ++k) {
    const uint8_t m = slotMotor[k];
    const uint8_t ph = (k < 2) ? first[m] : second[m];
    raw[m][ph] = convert(ADC_PHASE[m][ph]);
    const uint32_t c = ThreePhasePwm::counter();
    if (c + g_marginCounts >= pwm[m].level(ph)) ok[m] = false;
  }

  for (uint8_t m = 0; m < MOTORS; ++m) {
    valid_[m] = ok[m];
    if (!ok[m]) continue;  // hold the last valid currents
    const float i1 = toAmps(m, first[m], raw[m][first[m]]);
    const float i2 = toAmps(m, second[m], raw[m][second[m]]);
    amps_[m][first[m]] = i1;
    amps_[m][second[m]] = i2;
    amps_[m][skip[m]] = -(i1 + i2);
  }
}

void CurrentSense::calibrateOffsets(uint16_t samples) {
  // All phases sit at 50% duty (12.5 us half-window at 20 kHz): one motor's
  // three phases (~6 us) fit in one valley, so alternate motors per period.
  float sum[MOTORS][3] = {{0, 0, 0}, {0, 0, 0}};
  for (uint16_t i = 0; i < samples; ++i) {
    for (uint8_t m = 0; m < MOTORS; ++m) {
      ThreePhasePwm::waitForPeriodStart();
      for (uint8_t ph = 0; ph < 3; ++ph) sum[m][ph] += convert(ADC_PHASE[m][ph]);
    }
  }
  for (uint8_t m = 0; m < MOTORS; ++m) {
    for (uint8_t ph = 0; ph < 3; ++ph) offset_[m][ph] = sum[m][ph] / samples;
    amps_[m][0] = amps_[m][1] = amps_[m][2] = 0.0f;
    valid_[m] = false;
  }
}

float CurrentSense::toAmps(uint8_t motor, uint8_t phase, uint16_t raw) const {
  const float volts = ((float)raw - offset_[motor][phase]) * VOLTS_PER_COUNT;
  return CSA_SIGN * volts / CSA_GAIN_V_PER_A;
}

float CurrentSense::readBatteryVolts() {
  const float sense = (float)convert(ADC_VBAT) * VOLTS_PER_COUNT;
  return sense * VBAT_DIVIDER_RATIO;
}
