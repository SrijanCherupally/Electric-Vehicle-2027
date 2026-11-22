#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

// =============================================================================
// EV 2027 configuration
//
// Two kinds of settings live here:
//   [P] = default for a runtime parameter. Can be changed over USB without
//         reflashing ("set target_distance 7.25", then "save"). The value saved
//         in flash wins over the number here; "defaults" restores these.
//   (no tag) = compile-time constant. Change here and reflash.
//
// Units: meters, seconds, radians, volts unless the name says otherwise.
// All distances along the track are measured from the Start Point to the
// vehicle's Measurement Point (MP, the rearmost point of the vehicle).
// =============================================================================

// ---------- Event values (announced at the tournament) ----------
#define TARGET_DISTANCE_M 7.00f        // [P] 5.00 - 9.00 m, Start Point -> Target Point
#define TARGET_TIME_S 15.0f            // [P] 10.0 - 20.0 s, 0.5 s steps
#define BOTTLE_LINE_DISTANCE_M 2.00f   // [P] 1.00 - 3.00 m beyond the Target Point
// Bottle center, measured from the Start Point. Fixed by tournament level:
#define BOTTLE_START_REGIONAL_M 2.0f
#define BOTTLE_START_STATE_M 3.0f
#define BOTTLE_START_NATIONAL_M 4.0f
#define BOTTLE_START_DISTANCE_M BOTTLE_START_REGIONAL_M  // [P] ("level state" sets it)

// Rule limits, used only to warn when a value is outside the legal range.
#define RULE_TARGET_DISTANCE_MIN_M 5.0f
#define RULE_TARGET_DISTANCE_MAX_M 9.0f
#define RULE_TARGET_TIME_MIN_S 10.0f
#define RULE_TARGET_TIME_MAX_S 20.0f
#define RULE_BOTTLE_LINE_MIN_M 1.0f
#define RULE_BOTTLE_LINE_MAX_M 3.0f

// ---------- Run trims (tune these from test runs) ----------
#define DISTANCE_TRIM_M 0.000f   // [P] + = stop farther from the start
#define TIME_TRIM_S 0.00f        // [P] + = finish later (compensates settle/stop lag)
#define BOTTLE_MARGIN_M 0.030f   // [P] how far past the Bottle Line the bottle's near side ends
#define START_DELAY_S 0.30f      // [P] pause after the pencil press (counts toward run time)
#define REVERSE_DWELL_S 0.20f    // [P] pause between pushing and backing up

// ---------- Vehicle geometry ----------
#define WHEEL_DIAMETER_M 0.0600f        // [P] calibrate: roll a known distance, see README
#define TRACK_WIDTH_M 0.150f            // [P] wheel contact center to center
#define MP_TO_BOTTLE_CENTER_M 0.300f    // [P] MP -> center of a bottle seated in the pusher
#define BOTTLE_DIAMETER_M 0.065f        // rules: 6 - 7 cm

// ---------- Motion profile ----------
#define MAX_SPEED_MPS 1.50f          // [P] cruise speed ceiling (planner picks the actual cruise)
#define ACCEL_MPS2 0.80f             // [P] keep below ~2 m/s^2 so the bottle does not slide off
#define CONTACT_SPEED_MPS 0.30f      // [P] speed cap while meeting the bottle (0 = no cap)
#define CONTACT_ZONE_BEFORE_M 0.10f  // [P] start of the slow zone before contact
#define CONTACT_ZONE_AFTER_M 0.05f   // [P] end of the slow zone after contact
#define MIN_CRUISE_SPEED_MPS 0.05f   // planner lower bound

// ---------- Control gains (core 0, CONTROL_HZ) ----------
// Wheel loop output is volts in FOC voltage mode, amps in current mode.
#define KP_POS 4.0f              // [P] (m/s) per m of along-track error
#define KP_VEL 4.0f              // [P] V per (m/s) of wheel speed error
#define KI_VEL 40.0f             // [P] V per m of integrated speed error
#define KFF_VEL 3.0f             // [P] V per (m/s), back-EMF feedforward ("kvcal" measures it)
#define KFF_ACC 0.30f            // [P] V per (m/s^2)
#define KP_HEADING 5.0f          // [P] (rad/s) per rad of heading error
#define KD_HEADING 0.10f         // [P] per (rad/s) of yaw rate
#define KP_LATERAL 3.0f          // [P] rad of heading per m of lateral drift
#define MAX_HEADING_CORR_RAD 0.30f  // [P] lateral correction heading clamp
#define MOTOR_VOLTAGE_LIMIT_V 6.0f  // [P] per-motor |Uq| ceiling (also clamped to Vbus / sqrt(3))
#define VEL_FILTER_HZ 40.0f      // wheel speed low-pass cutoff
#define HEADING_ENCODER_WEIGHT 0.0f  // 0 = gyro only, 1 = wheel-difference only

// ---------- Drivetrain mapping ----------
// Which driver is on which side, and whether "motor positive" rolls the wheel
// forward. After FOC alignment, a positive command always increases motor
// position, so these signs are purely about how the motors are mounted.
#define LEFT_MOTOR_INDEX 0   // 0 = M1 (J3 / J5), 1 = M2 (J4 / J6)
#define RIGHT_MOTOR_INDEX 1
#define LEFT_WHEEL_DIR (+1.0f)
#define RIGHT_WHEEL_DIR (-1.0f)  // mirrored motor

// ---------- Motors + encoders ----------
#define MOTOR_POLE_PAIRS 7           // e.g. 2204 / 2804 gimbal = 7 (12N14P)
#define GEAR_RATIO 1.0f              // motor revs per wheel rev (1 = direct drive)

// MT6701QT-ACD magnetic encoder (LCSC C49257133), one per motor shaft.
// Factory-programmed: ABZ output, 1024 pulses/rev, Z width 1 LSB, CCW = count up.
// ABZ is selected in hardware (MODE tied to GND on the encoder PCB), so no
// configuration is needed or possible from this board (J5/J6 have no I2C, and
// MT6701 EEPROM writes need VDD > 4.5 V; the ports supply 3.3 V).
// Alignment prints the PPR it measures, so a mismatch is easy to spot.
#define MT6701_ABZ_PPR 1024
#define ENCODER_CPR (4 * MT6701_ABZ_PPR)  // x4 quadrature decode: counts per motor rev
#define MT6701_STARTUP_MS 100  // A/B output starts ~50 ms after power-up; wait before aligning

// ---------- FOC engine (core 1) ----------
#define PWM_FREQUENCY_HZ 20000       // DRV8317 allows up to 200 kHz
#define PWM_MAX_DUTY 0.95f           // leaves a low-side window for current sampling
#define FOC_LOOP_DIVIDER 1           // run FOC every Nth PWM period (1 = every period)
#define FOC_ALIGN_VOLTAGE_V 2.0f     // voltage vector used during alignment
#define FOC_ALIGN_SWEEP_MS 800       // one electrical revolution sweep time
#define FOC_ALIGN_SETTLE_MS 400
#define FOC_ALIGN_MIN_FRACTION 0.8f  // measured / expected counts per e-rev to accept
                                     // (0.8 still catches an MT6701 left at half resolution)
#define FOC_CURRENT_MODE 0           // 0 = voltage-mode FOC (default), 1 = dq current loop
#define FOC_CURRENT_KP 2.0f          // V per A (current mode)
#define FOC_CURRENT_KI 400.0f        // V per A*s (current mode)
#define FOC_CURRENT_LIMIT_A 1.5f     // current-mode command clamp
#define FOC_PHASE_RESISTANCE_OHM 0.0f  // >0 enables I*R feedforward in current mode

// DRV8317H current sense amplifier. GAIN pin is tied to GND on this board.
// Verify against the DRV8317 datasheet GAIN table before trusting the amps.
#define CSA_GAIN_V_PER_A 0.25f
#define CSA_SIGN (-1.0f)            // flip if measured Iq has the wrong sign
#define CURRENT_SENSE_ENABLED 1     // sample phase currents every FOC cycle
#define CURRENT_OFFSET_SAMPLES 2000
#define OVERCURRENT_A 3.0f          // |Iq| above this ...
#define OVERCURRENT_TIME_MS 50      // ... for this long latches a fault (0 = off)
#define COMMAND_TIMEOUT_MS 100      // core 1 brakes if core 0 stops sending commands

// ---------- Battery (8x AA NiMH / alkaline) ----------
#define VBAT_NOMINAL_V 9.6f
#define VBAT_LOW_WARN_V 8.0f        // warn (1.0 V/cell under no load)
#define VBAT_FILTER_ALPHA 0.02f

// ---------- IMU ----------
#define IMU_YAW_AXIS 2               // 0 = X, 1 = Y, 2 = Z (board flat: Z)
#define IMU_YAW_SIGN (+1.0f)         // + = counter-clockwise seen from above is positive
#define GYRO_SCALE 1.000f            // [P] yaw rate scale trim
#define GYRO_CAL_MS 1500             // bias average time after alignment
#define GYRO_BIAS_TRACK_TAU_S 5.0f   // slow bias tracking while parked in READY

// ---------- Run supervision ----------
#define FINISH_SETTLE_S 0.50f        // hold position after the profile ends, then brake
#define MAX_RUN_TIME_S 30.0f         // hard stop
#define MAX_TRACKING_ERROR_M 0.40f   // abort if the car falls this far behind / ahead
#define BUTTON_DEBOUNCE_MS 15
#define BUTTON_LONG_PRESS_MS 1500
#define ABORT_LOCKOUT_S 1.0f         // ignore button presses this long after start

// ---------- Loop rates + telemetry ----------
#define CONTROL_HZ 1000
#define TELEMETRY_HZ 50
#define LOG_HZ 50
#define LOG_MAX_SAMPLES 2000         // 40 s at 50 Hz
#define SERIAL_BAUD 115200

// ---------- Derived ----------
#define CONTROL_DT_S (1.0f / CONTROL_HZ)
#define VBAT_DIVIDER_RATIO \
  ((VBAT_DIVIDER_R_TOP_OHM + VBAT_DIVIDER_R_BOTTOM_OHM) / VBAT_DIVIDER_R_BOTTOM_OHM)

#endif  // CONFIG_H
