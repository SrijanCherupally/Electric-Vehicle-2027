# EV Controller 2027 firmware

Firmware for the [EV Controller v1](../hardware/controller-v1) board (RP2350B + 2x DRV8317H, dual-motor FOC) for the 2027 Science Olympiad Div C Electric Vehicle event.

This version uses the motor encoders and the gyro for navigation. The odometry pods are not used yet.

## The run (2027 rules)

```
Start ──────── bottle (2/3/4 m) ════ push ════► bottle past Bottle Line
  MP=0           xContact                         xPush
                                     ◄── reverse ── MP stops on Target Point (xFinal)
```

All positions are the Measurement Point (MP, rear of the car) measured from the Start Point. The planner works out these positions from the parameters:

| | |
|---|---|
| `xContact` | `bottle_start - mp_to_bottle` (the pusher meets the bottle) |
| `xPush` | the bottle's near side ends `bottle_margin` past the Bottle Line (`target_distance + bottle_line`) |
| `xFinal` | `target_distance + distance_trim` |

It then picks one cruise speed so that start delay + push + dwell + reverse takes exactly `target_time + time_trim`. Near the bottle it slows to `contact_speed`. Run `plan` to see the numbers.

## Building / uploading

Open the repo folder in VS Code with the PlatformIO extension installed. `platformio.ini` sits at the repo root and points at `firmware/`.

- **Board:** `solderparty_rp2350_stamp_xl`. This is the earlephilhower **RP2350B** variant (48 GPIO, 16 MB flash, 12 MHz crystal), which matches this board. Do not use `rpipico2`: that is the RP2350A and cannot reach GPIO 30-47.
- **First upload:** hold BOOT (SW3), tap RESET (SW1), release BOOT. The board shows up as a USB drive. After that, normal uploads over USB work.
- **Serial monitor:** 115200 baud. Type `help`.

## Code layout

```
firmware/
  include/config.h       every tunable, with units ([P] = changeable over USB)
  include/hardware.h     pin map from the schematic netlist
  motor/                 core 1 FOC engine + core 0 wheel control
    drv8317              nSLEEP / nFAULT + phase-aligned 3-phase PWM
    encoder              PIO quadrature decoder (no interrupts)
    current_sense        ADC sampling, synced to the low-side window
    foc                  Clarke/Park/SVPWM, alignment, voltage or current mode
    motor_core           the core 1 loop + the only API core 0 uses
    drivetrain           counts -> meters, per-wheel speed loops
  sensors/
    imu                  ICM-42688-P yaw rate, bias calibration
    battery              pack voltage + low warning
  control/
    trajectory           analytic trapezoid motion profiles
    run_plan             the 2027 run: geometry + time solve
    localizer            x / y / heading from encoders + gyro
    controller           trajectory tracking (along-track, heading, lateral)
  system/
    params               runtime parameters, saved in flash
    state                BOOT -> ALIGN -> CAL -> READY -> RUN -> FINISHED / FAULT
    commands             USB command line
    telemetry            Teleplot live view + RAM run log (CSV dump)
    ui                   START button, status LED
  src/main.cpp           core 0 loop at CONTROL_HZ, core 1 runs motorcore
```

**Core split.** Core 1 runs only FOC. Each PWM period it waits for the counter wrap, samples the current, reads both encoders and commutates both motors. Core 0 runs everything else at 1 kHz. The two cores only talk through `motor/motor_core.h`.

## Bring-up checklist (do once per build)

Do this with the car on a stand, wheels off the ground.

1. **Encoders need no setup.** The encoder boards use the MT6701QT-ACD (LCSC C49257133), which comes factory-programmed: ABZ output, 1024 PPR, Z width 1 LSB. ABZ is selected in hardware (MODE tied to GND on the encoder PCB). Keep `MT6701_ABZ_PPR = 1024`.
2. **Check `config.h` against the real hardware:** `MOTOR_POLE_PAIRS`, `GEAR_RATIO`, and `CSA_GAIN_V_PER_A` (check the DRV8317 GAIN-pin table; GAIN is tied to GND on this board).
3. **Boot and watch the serial log.** It should go `ALIGN: ok` → `GYRO` → `READY`. For each motor, alignment prints a line like `ALIGN M1: ok, 585 counts/e-rev (expect 585) -> MT6701 ~1024 PPR`. If alignment fails, it tells you why:
   - **no encoder counts:** check the SH1.0 cable orientation (these cables are often reversed), the magnet gap (0.5–2 mm) and centering (within 0.3 mm), the 3V3 at the port, and the motor phases
   - **counts per e-rev wrong:** almost always `MOTOR_POLE_PAIRS` (585 counts/e-rev is what a 7-pole-pair motor gives at 1024 PPR)
4. **Check directions.** Run `mtest 1`. Both wheels should spin **forward**. If one spins backwards, flip `LEFT_WHEEL_DIR` / `RIGHT_WHEEL_DIR`. If left and right are swapped, swap `LEFT_MOTOR_INDEX` / `RIGHT_MOTOR_INDEX`.
5. **Check the gyro sign.** Run `tele on`, then turn the car counter-clockwise by hand. `heading_deg` should increase. If it doesn't, flip `IMU_YAW_SIGN`.
6. **Measure the back-EMF feedforward.** Run `kvcal 2`, then `save`. This sets `kff_vel`.

## Calibration (on the floor)

- **Wheel diameter:** drive a run, measure how far the car actually went, then set `wheel_dia` to `wheel_dia * actual / reported`.
- **Track width / gyro scale:** spin the car 10 turns by hand. `heading_deg` should read 3600; adjust `gyro_scale` until it does.
- **Distance and time trims:** after a few runs, set `distance_trim` to the average stop error and `time_trim` to the average time error.
- **Bottle:** tune `bottle_margin` (bottle ends just past the line for the -20 bonus) and `contact_speed` (bottle must stay upright and in the pusher).
- **Logs:** after any run, `dump` prints a CSV (reference vs actual position, speed, heading, motor commands) you can paste into a spreadsheet.

## Competition day

```
set target_distance 7.25
set target_time 14.5
set bottle_line 1.75
level regional          (or state / national)
plan                    check: valid, OK (not LATE)
save
```

Then put the MP over the Start Point and press START with the pencil. The LED shows the state:

| LED | Meaning |
|---|---|
| slow blink | ready |
| solid | running |
| double blink | finished |
| strobe | fault |

When a run finishes, press START again to re-arm. In a fault, hold START for 1.5 s (or type `clear`) to re-align.

Only the pencil may start the run, so `go` over USB is for bench testing only.

## Safety behavior

- The DRV8317 nFAULT pin, a software overcurrent check, and a command watchdog on core 1 all latch a fault and put the drivers to sleep.
- Falling more than `MAX_TRACKING_ERROR_M` behind or ahead of the plan aborts the run.
- A fresh START press during a run (after 1 s) aborts it.

## Not done yet

- Odometry pods (J7/J8): the pins are in `hardware.h`.
- Encoder Z index: could be used to skip the boot alignment.
- Expansion UART (J10).
