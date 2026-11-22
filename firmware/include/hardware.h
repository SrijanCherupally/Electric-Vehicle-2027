#ifndef HARDWARE_H
#define HARDWARE_H

// Pin map for EV Controller v1 (RP2350B).
// Source of truth: hardware/controller-v1/schematic/SciOly_EV_Controller_v1_Schematic.enet
// If the board is respun, update this file only; no other file hardcodes a pin.

// ---------- DRV8317H motor drivers (U4 = M1, U5 = M2) ----------
// 3x PWM mode (MODE tied to AVDD): INHx is the PWM input for each half bridge,
// INLx is tied high on the board (bridge enabled). INHx high = high side on,
// INHx low = low side on, dead time inserted by the driver.
#define PIN_DRV_NSLEEP 31  // shared by both drivers, 100k pulldown (R20): low = sleep
#define PIN_DRV_NFAULT 32  // shared open-drain, 10k pull-up (R13): low = fault

#define PIN_M1_INA 36
#define PIN_M1_INB 35
#define PIN_M1_INC 34
#define PIN_M2_INA 39
#define PIN_M2_INB 38
#define PIN_M2_INC 37

// ---------- Current sense + battery (RP2350B ADC: GPIO40-47 = ADC0-7) ----------
// DRV8317 CSAREF is tied to AVDD, so zero current reads at about AVDD / 2.
#define PIN_M1_SOA 40  // ADC0
#define PIN_M1_SOB 41  // ADC1
#define PIN_M1_SOC 42  // ADC2
#define PIN_VBAT_SENSE 43  // ADC3
#define PIN_M2_SOA 44  // ADC4
#define PIN_M2_SOB 45  // ADC5
#define PIN_M2_SOC 46  // ADC6
#define ADC_FIRST_PIN 40

// Battery divider: R3 = 100k (VBAT -> sense), R4 = 22k (sense -> GND).
#define VBAT_DIVIDER_R_TOP_OHM 100000.0f
#define VBAT_DIVIDER_R_BOTTOM_OHM 22000.0f
#define ADC_REFERENCE_V 3.3f
#define ADC_MAX_COUNTS 4095.0f

// ---------- Motor encoders (J5 = M1, J6 = M2), SH1.0: 3V3, GND, A, B, Z ----------
// MT6701 in ABZ mode (push-pull outputs at 3.3 V). Z (one pulse per rev) is
// wired but not used yet.
// On both connectors B is routed to the LOWER GPIO. The PIO decoder needs two
// consecutive pins, so it is given base = B. Count direction does not matter:
// FOC alignment measures it and every count is reported in "motor positive"
// units afterwards.
#define PIN_M1_ENC_A 11
#define PIN_M1_ENC_B 10
#define PIN_M1_ENC_Z 9
#define PIN_M2_ENC_A 27
#define PIN_M2_ENC_B 26
#define PIN_M2_ENC_Z 25

// ---------- Odometry pods (J7, J8): not used yet ----------
#define PIN_ODO1_A 23
#define PIN_ODO1_B 22
#define PIN_ODO1_Z 21
#define PIN_ODO2_A 30
#define PIN_ODO2_B 29
#define PIN_ODO2_Z 28

// ---------- IMU: ICM-42688-P on SPI0 ----------
#define PIN_IMU_MISO 16
#define PIN_IMU_SCK 18
#define PIN_IMU_MOSI 19
#define PIN_IMU_CS 20
#define PIN_IMU_INT1 13

// ---------- User interface ----------
#define PIN_START_BTN 24   // SW2 (tall tactile, pencil start), active low, 10k pull-up (R18)
#define PIN_STATUS_LED 7   // LED1, active high
// SW1 = RESET (RUN pin), SW3 = BOOT (QSPI_SS): hardware only, no GPIO.

// ---------- Expansion port J10 (3V3, GND, IO, RX, TX) ----------
#define PIN_EXP_TX 4  // UART1 TX
#define PIN_EXP_RX 5  // UART1 RX
#define PIN_EXP_IO 6

#endif  // HARDWARE_H
