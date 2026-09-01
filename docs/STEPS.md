# DualCore — staged build log

This sprint splits the ECU firmware across the ESP32's two cores (ECU-ADR-004):
**core 0** = Bluepad32 + gamepad (`btTask`), **core 1** = OLED + LED status
pattern (`uiTask`). It clones `BluetoothPairing` and adds the core split, then
wires the OLED to real Bluetooth events and real stick input, then adds battery
sensing with a hard power cut-off.

The user's "Core 1 / Core 2" = the ESP32's core 0 / core 1.

Each step lives on its own branch (`step-N-...`), branched off the previous
step's merged tip. Many commits per branch is fine. Merge to `main` when the
step's verification passes, then tag the merge `step-N`.

The sprint's real deliverable is the **G4 verdict** (ECU-SPEC-001 §9): does the
core split hold timing with the OLED redrawing? Recorded at the bottom.

---

## Step 1 — two tasks, one per core  · branch `step-1-two-cores`

**What changed**

- New PlatformIO project, `platformio.ini` cloned from `BluetoothPairing` (Bluepad32
  custom core + the short `core_dir` / `build_dir` paths for the Windows
  command-line-length limit) with the Adafruit SH110X / GFX libs added for the OLED.
- `src/main.cpp`: `setup()` creates two pinned FreeRTOS tasks and returns.
  - `btTask` — core 0, priority 3, 8 KB stack. Prints its core + a heartbeat
    every 1 s, toggles GPIO2.
  - `uiTask` — core 1, priority 1, 4 KB stack. Prints its core + a frame counter
    every 1 s.
  - `loop()` just `vTaskDelay`s — all work is in the two tasks.
- `ISOLATION_TEST` compile flag: when 1, `uiTask` busy-spins 3 s every ~10 s so
  you can watch `btTask` keep beating through it.

**How to verify**

```
$env:PATH += ";$env:USERPROFILE\.platformio\penv\Scripts"; pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

1. Monitor shows `bt  alive on core 0` and `ui  alive on core 1` interleaved,
   once a second each, steadily.
2. Boot lines print each task's stack high-water mark — record the numbers below
   as the baseline; both should have comfortable headroom (thousands of words).
3. Rebuild with `-D ISOLATION_TEST=1` (or flip the `#define`). During `uiTask`'s
   3 s spin, `btTask`'s `beat N` lines must keep incrementing once a second with
   no stall. Flip the spin into `btTask` to check the other direction, then set
   the flag back to 0.

**Results** _(fill in after running on hardware)_

- [ ] both tasks report the expected cores
- [ ] stack high-water: bt = ____ words, ui = ____ words
- [ ] isolation test: btTask kept beating through uiTask's spin — yes / no
- notes:

---

## Step 2 — Bluepad32 owned by the core-0 task  · branch `step-2-bluepad32`

_not started_

## Step 3 — pairing state machine (core 0) + UI mockup browser (core 1)  · branch `step-3-pairing-plus-mockup`

_not started_

## Step 4a — naive `volatile` shared struct  · branch `step-4a-volatile`

_not started_

## Step 4b — mutex-protected `RobotState` + snapshot  · branch `step-4b-mutex-snapshot`

_not started_

## Step 5 — 3S LiPo voltage sensing + hard cut-off  · branch `step-5-battery-cutoff`

_not started_

---

## G4 verdict

_Pending. After step 5, run with everything active (gamepad connected, stick
moving, OLED redrawing S4, LEDs animating, battTask sampling) and log the
`btTask` loop-interval min/max/mean over 60 s. "Split holds" feeds the initial
commit of `07_Codebase_Repository`; "needs amendment" goes back into
ECU-ADR-004 / ECU-SPEC-001 §9._
