# Electric-Vehicle-2027
HW and SW for my Electric Vehicle 2027 season!

## Hardware

- [`hardware/controller-v1`](hardware/controller-v1) — main controller PCB (RP2350B + 2x DRV8317H FOC drivers, 4-layer). Schematic, PCB, and JLCPCB fab files.
- [`hardware/encoder-v1`](hardware/encoder-v1) — MT6701 magnetic ABZ encoder (17.6 mm square, 2-layer), four per car for the motors and odometry wheels.

## Firmware

- [`firmware/`](firmware) — PlatformIO project (RP2350B, dual-core: FOC on core 1, run control on core 0). `platformio.ini` is at the repo root so VS Code picks it up. See [`firmware/README.md`](firmware/README.md) for bring-up, tuning and the competition-day commands.
