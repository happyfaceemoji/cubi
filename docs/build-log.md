# Build log: real parts vs the model (16-21 Nov 2026)

## Toolchain
- Arduino IDE 2.3.11, ESP32 core 3.0.7 (Boards Manager). On Apple Silicon the 3.0.7 esptool needs Rosetta.
- Board XIAO_ESP32S3, PSRAM = OPI PSRAM, USB CDC On Boot = Enabled, 115200 baud.
- `firmware/cubi_fw/cubi_fw.ino` is a single file (headers inlined; enums `Trick`, `Act`, `Choice` at the top for the IDE's auto-prototypes). Keep it the only `.ino` in its folder: the IDE compiles every `.ino` it finds.

## Pins
Servos FL D1, FR D3, RL D8, RR D9 · ToF SDA D4, SCL D5 (VIN 3V3) · amp BCLK D6, LRC D7, DIN D10 · touch D0 · battery sense D2 (2 x 100k divider). Never use the 5V pin.

## Wiring check (ERC 39 pass / 0 fail)
- With the switch OFF and USB in, the XIAO charger output still feeds the switched rail, so D2 reads garbage. Always self-test with the switch ON.
- Optional: 100 nF from D2 to GND steadies the divider.
- Charge current ~100 mA: a full 500 mAh charge takes 5-6 h, and only with the switch ON.

## Legs and servos
- servoDeg sign was reversed on all four legs. Fixed: `sign = (i % 2 == 0) ? -1 : 1`.
- Fitting that works: send `c`, press each leg on pointing straight down, then check `f` swings toward the eyes first.
- GH-S37D measured range 425-2575 us = 180 deg. Tabs trimmed to 3.8 mm.

## Distance sensor
- Sees the desk 15-18 cm ahead (open desk 170-200 mm, designed 100-115): aimed ~12-15 deg flatter than 30. The reflex is ratio-based so it still works, but take the baseline standing on a clear desk (boot or `D`).
- Reflex needs 3 readings in a row so single 8191 dropouts don't trigger it.

## Eyes (face tracking)
- esp-dl two-stage detector (MSR01 + MNP01) on core 0, QVGA RGB565, ~11 fps.
- Camera is mounted sideways: `E` found 90 deg rotation, `j` swap needed; both saved in flash. Only ~45 deg across.
- Follow v2: start turning at 6 deg, stop within 3 deg, minimum 0.5 s per turn, chase the side you left on for 2.5 s.

## Walking
- On a rough painted desk no gait works (~70 measured 3 s walks, -12 to +13 mm, random direction).
- On printer paper, auto-tune (`T`) found: trained pattern, fast fraction 0.50, amplitude 12 deg, 2.2 Hz, no lean: +19 and +90 mm. Now the default.
- A USB cable dragging behind her cancels the step at ~100 g. Test with the cable held up or untethered.
- Mind mode on battery walks well on paper. Edge reflex on a real edge still unconfirmed.
- Steering trim `(` / `)` fixes a constant drift; saved in flash.

## Open faults (18 Nov)
- Camera probe 0x105 ESP_ERR_NOT_FOUND, mic stuck at one value, battery reads 6.27 V. Camera and mic both live on the Sense board, so its B2B connector is unseated, and the divider ground leg opened in the same closing. Physical fix: reseat, re-check divider GND, foam shim. The v2 body adds a foam pad for this.
- Until D2 is fixed the 3.6 V low-battery stop can't trigger: keep battery runs to ~30 min.
- Phone remote page timed out on first test; suspected missing U.FL antenna.
