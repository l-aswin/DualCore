# DualCore — staged build log

This sprint splits the ECU firmware across the ESP32's two cores (ECU-ADR-004):
**core 0** = Bluepad32 + gamepad (`btTask`), **core 1** = OLED + `led_pattern`
command emit (`uiTask`). It clones `BluetoothPairing` and adds the core split, then
wires the OLED to real Bluetooth events and real stick input, then adds the
`powerState` input and the CRITICAL response.

> **Rev 5 note (2026-09-01):** the WS2812B ring + buzzer moved to the TCU
> and battery sensing moved to the BMS (ECU-SPEC-001 rev 5). So `uiTask` no longer
> *animates* LEDs — it emits `RING …` / `BUZZ …` commands over UART1 — and there is
> no on-ECU `battTask`. Steps 3 and 5 below reflect that.

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

- [ ] both tasks report the expected cores - yes
- [ ] stack high-water: bt = 1667 words, ui = 624 words
- [ ] isolation test: btTask kept beating through uiTask's spin — yes
- notes:
---

## Step 2 — Bluepad32 owned by the core-0 task  · branch `step-2-bluepad32`

Branched off the `step-1` tag. Bring the real Bluepad32 stack up **inside `btTask`
on core 0** and read a gamepad. No pairing state machine yet (step 3); `uiTask`
stays the step-1 stub. The point of the step is to show the BT stack initialises
and runs on the pinned core-0 task without disturbing `uiTask` on core 1.

**What changed**

- `platformio.ini`: unchanged — the Bluepad32 custom core was already in place from
  step 1.
- `src/main.cpp`:
  - `#include <Bluepad32.h>`; `controllers[BP32_MAX_GAMEPADS]`, the
    `onConnectedController` / `onDisconnectedController` handlers, and
    `inputActive()` / `dumpGamepad()` / `STICK_DEADZONE` copied from
    `BluetoothPairing` without the NVS / pairing-window code.
  - `BP32.setup()` moved out of `setup()` and into `btTask`: `setup()` runs on the
    Arduino loopTask (core 1), and ECU-ADR-004 pins the BT stack to core 0, so init
    must happen once `btTask` is running. `btTask` then calls
    `enableVirtualDevice(false)` + `enableNewBluetoothConnections(true)` and drops
    into a fast loop of `BP32.update()`, printing `dumpGamepad()` on
    `hasData() && inputActive(ctl)`.
  - btTask loop delay 1000 ms → ~5 ms so `BP32.update()` is serviced promptly; the
    heartbeat LED + stack/core log move behind a 1 s `millis()` check.
  - `uiTask` and `loop()` untouched. `ISOLATION_TEST` kept.

**How to verify**

```
$env:PATH += ";$env:USERPROFILE\.platformio\penv\Scripts"; pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

1. Boot: Bluepad32 init logs from `btTask` on core 0, no `Guru Meditation` /
   watchdog reset, `ui  alive on core 1` keeps ticking.
2. Connect a controller — `Controller connected, slot 0`, Mode LED (GPIO2) solid,
   `dumpGamepad` streams while a stick moves and stops on release.
3. `ui  alive on core 1  (frame N)` holds its steady 1 Hz cadence through connect,
   disconnect and active gamepad input — no skipped or bunched frames.
4. Power the controller off — `Controller disconnected, slot 0`, Mode LED drops,
   `btTask` heartbeat uninterrupted.
5. Record the `bt` stack high-water with the BT stack up; confirm headroom.
6. `-D ISOLATION_TEST=1`: `uiTask`'s 3 s spin must not stall `BP32.update()`, and
   flipping the spin into `btTask` must not stall `uiTask`. Set the flag back to 0.

**Results** _(fill in after running on hardware)_

- [ yes] Bluepad32 init runs on core 0 from `btTask`, no watchdog reset
- [ yes] controller connects, `dumpGamepad` streams on input, stops on release
- [ yes] `uiTask` frame cadence unaffected by connect / disconnect / input
- [ yes] disconnect handled cleanly, heartbeat uninterrupted
- [ yes] stack high-water: bt = 1602 words, ui = 616 words
- [ yes] isolation test: BT poll and `uiTask` each ride through the other's spin
- notes:

## Step 3 — pairing state machine (core 0) + OLED / ring-command emit (core 1)  · branch `step-3-pairing-plus-mockup`

_not started._ core 1 renders the OLED screens (ECU-SPEC-002) **and** emits the
matching `RING …` command on each `linkState` change (no local pixels — the ring is
on the TCU). For this sprint the TCU end is a serial print / loopback stub.

## Step 4a — naive `volatile` shared struct  · branch `step-4a-volatile`

_not started_

## Step 4b — mutex-protected `RobotState` + snapshot  · branch `step-4b-mutex-snapshot`

_not started_

## Step 5 — `powerState` input + CRITICAL response  · branch `step-5-powerstate`

_not started._ Battery *sensing* is the BMS's now (ECU-SPEC-001 rev 5), so this step
no longer reads a divider. It feeds a simulated `powerState` (button or timer →
`WARNING` → `CRITICAL`) into `RobotState` and exercises the ECU's reaction under the
core split: `motor_control` zeros + holds, the BLE path tears down, `oled_ui` shows S0,
`led_pattern` emits `RING OFF` + `BUZZ S0`. The G4-relevant question is whether the
split holds timing while a CRITICAL event forces a screen change + motor stop at once.

---

## G4 verdict

_Pending. After step 5, run with everything active (gamepad connected, stick
moving, OLED redrawing S4, `RING …` / `BUZZ …` commands going out on UART1, a
simulated `powerState` CRITICAL forcing S0 + motor stop) and log the `btTask`
loop-interval min/max/mean over 60 s. "Split holds" feeds the initial commit of
`07_Codebase_Repository`; "needs amendment" goes back into ECU-ADR-004 /
ECU-SPEC-001 §9._
