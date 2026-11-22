#include "motor/drv8317.h"

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#include "config.h"
#include "hardware.h"

// ---------------------------------------------------------------------------
// Drv8317
// ---------------------------------------------------------------------------

bool Drv8317::awake_ = false;

void Drv8317::begin() {
  gpio_init(PIN_DRV_NSLEEP);
  gpio_set_dir(PIN_DRV_NSLEEP, GPIO_OUT);
  gpio_put(PIN_DRV_NSLEEP, 0);
  awake_ = false;

  gpio_init(PIN_DRV_NFAULT);
  gpio_set_dir(PIN_DRV_NFAULT, GPIO_IN);
  gpio_pull_up(PIN_DRV_NFAULT);  // board has R13 too; harmless
}

void Drv8317::setAwake(bool awake) {
  gpio_put(PIN_DRV_NSLEEP, awake ? 1 : 0);
  if (awake && !awake_) {
    delay(2);  // tWAKE: charge pump + CSA startup (datasheet ~1 ms)
  }
  awake_ = awake;
}

bool Drv8317::isAwake() { return awake_; }

bool Drv8317::faultActive() { return gpio_get(PIN_DRV_NFAULT) == 0; }

void Drv8317::clearFault() {
  // A short nSLEEP low pulse clears latched faults without a full sleep cycle.
  gpio_put(PIN_DRV_NSLEEP, 0);
  delayMicroseconds(20);
  gpio_put(PIN_DRV_NSLEEP, 1);
  delay(2);
  awake_ = true;
}

// ---------------------------------------------------------------------------
// ThreePhasePwm
// ---------------------------------------------------------------------------

uint32_t ThreePhasePwm::sliceMask_ = 0;
uint32_t ThreePhasePwm::top_ = 0;
int ThreePhasePwm::syncSlice_ = -1;

ThreePhasePwm::ThreePhasePwm(uint pinA, uint pinB, uint pinC)
    : pins_{pinA, pinB, pinC}, slice_{0, 0, 0}, chan_{0, 0, 0} {}

void ThreePhasePwm::begin() {
  if (top_ == 0) {
    // Phase-correct: one PWM period = 2 * (TOP + 1) clocks.
    top_ = clock_get_hz(clk_sys) / (2u * PWM_FREQUENCY_HZ) - 1u;
  }

  for (int i = 0; i < 3; ++i) {
    slice_[i] = pwm_gpio_to_slice_num(pins_[i]);
    chan_[i] = pwm_gpio_to_channel(pins_[i]);

    // Slices are shared between motors (e.g. GPIO36 / GPIO37 are slice 10 A/B),
    // so configure each slice only once.
    if ((sliceMask_ & (1u << slice_[i])) == 0) {
      pwm_config cfg = pwm_get_default_config();
      pwm_config_set_phase_correct(&cfg, true);
      pwm_config_set_clkdiv_int(&cfg, 1);
      pwm_config_set_wrap(&cfg, top_);
      pwm_config_set_output_polarity(&cfg, true, true);  // see header
      pwm_init(slice_[i], &cfg, false);
      pwm_set_counter(slice_[i], 0);
      sliceMask_ |= 1u << slice_[i];
    }
    gpio_set_function(pins_[i], GPIO_FUNC_PWM);
  }

  if (syncSlice_ < 0) {
    syncSlice_ = slice_[0];
  }
  setNeutral();
}

void ThreePhasePwm::setDuties(float a, float b, float c) {
  const float duty[3] = {a, b, c};
  const float span = (float)(top_ + 1u);
  for (int i = 0; i < 3; ++i) {
    float d = duty[i];
    if (d < 0.0f) d = 0.0f;
    if (d > PWM_MAX_DUTY) d = PWM_MAX_DUTY;
    // Inverted output: high while counter >= level, so level = (1 - duty) * span.
    uint16_t level = (uint16_t)((1.0f - d) * span + 0.5f);
    pwm_set_chan_level(slice_[i], chan_[i], level);
  }
}

void ThreePhasePwm::startAll() {
  for (uint s = 0; s < NUM_PWM_SLICES; ++s) {
    if (sliceMask_ & (1u << s)) {
      pwm_set_counter(s, 0);
    }
  }
  // One register write enables every slice on the same clock edge.
  hw_set_bits(&pwm_hw->en, sliceMask_);
}

void ThreePhasePwm::waitForPeriodStart() {
  if (syncSlice_ < 0) return;
  const uint32_t bit = 1u << syncSlice_;
  pwm_hw->intr = bit;  // clear, so we always sync to a fresh wrap
  while ((pwm_hw->intr & bit) == 0) {
    tight_loop_contents();
  }
}

float ThreePhasePwm::frequencyHz() {
  if (top_ == 0) return 0.0f;
  return (float)clock_get_hz(clk_sys) / (2.0f * (float)(top_ + 1u));
}
